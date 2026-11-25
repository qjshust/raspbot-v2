"""Pi 网关反向代理后端（仅标准库）。

职责：

* ``/api/*`` 普通请求：``http.client`` 转发（保留方法/路径/查询/状态码/Content-Type/二进制体），
  5 s 超时；上游连接失败抛 :class:`UpstreamUnreachable`（HTTP 层转 502 JSON）。
* SSE：``GET /api/events`` 连接后逐行读取上游事件流，由本地 HTTP 服务器重新封装为
  HTTP/1.1 chunked 响应；不转发上游逐跳头（Content-Length / Transfer-Encoding / Connection）。
* token：配置了 token 时自动加 ``X-Console-Token``（客户端自带的 token 优先级低于配置）。
* 目标切换：``set_target`` 线程安全，供 ``POST /local/config`` 即时生效。
"""

from __future__ import annotations

import http.client
import logging
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Iterable

log = logging.getLogger("console.upstream")

#: 转发时丢弃的逐跳 / 由本地重新生成的响应头
_DROP_HEADERS = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailer",
    "trailers",
    "transfer-encoding",
    "upgrade",
    "content-length",
    "date",
    "server",
}


class UpstreamError(Exception):
    """上游网关错误基类。"""

    code = "upstream_error"
    status = 502

    def __init__(self, message: str):
        super().__init__(message)
        self.message = message


class UpstreamUnreachable(UpstreamError):
    code = "upstream_unreachable"
    status = 502


class UpstreamTimeout(UpstreamError):
    code = "upstream_timeout"
    status = 504


@dataclass
class WireResponse:
    """已完整读取的上游响应。"""

    status: int
    headers: list[tuple[str, str]] = field(default_factory=list)
    body: bytes = b""

    def content_type(self) -> str:
        for key, value in self.headers:
            if key.lower() == "content-type":
                return value
        return ""


@dataclass
class WireStream:
    """SSE / 流式响应。

    ``buffered=True`` 表示上游返回的不是 SSE（例如 401 JSON 错误），此时 ``body``
    携带完整内容，由 HTTP 层按普通响应发送。
    """

    status: int
    headers: list[tuple[str, str]] = field(default_factory=list)
    chunks: Iterable[bytes] = ()
    buffered: bool = False
    body: bytes = b""

    def content_type(self) -> str:
        for key, value in self.headers:
            if key.lower() == "content-type":
                return value
        return ""


def _filter_headers(pairs: Iterable[tuple[str, str]]) -> list[tuple[str, str]]:
    out: list[tuple[str, str]] = []
    for key, value in pairs:
        lowered = key.lower()
        if lowered in _DROP_HEADERS or lowered.startswith("proxy-"):
            continue
        out.append((key, value))
    return out


def _is_timeout_error(exc: BaseException) -> bool:
    return isinstance(exc, (TimeoutError, socket.timeout))


class Upstream:
    """Pi 网关的 HTTP 代理后端。"""

    def __init__(
        self,
        host: str,
        port: int,
        token: str = "",
        timeout: float = 5.0,
        stream_timeout: float = 600.0,
    ) -> None:
        self._lock = threading.Lock()
        self._host = host
        self._port = int(port)
        self._token = token or ""
        self._timeout = float(timeout)
        self._stream_timeout = float(stream_timeout)
        self._conns: set[http.client.HTTPConnection] = set()
        self._closed = False

    # ---- 目标管理 ---------------------------------------------
    def set_target(self, host: str, port: int, token: str | None = None) -> None:
        with self._lock:
            self._host = host
            self._port = int(port)
            if token is not None:
                self._token = token
        log.info("代理目标已切换: %s:%s token_set=%s", host, port, bool(self._token_text()))

    def get_target(self) -> tuple[str, int, str]:
        with self._lock:
            return self._host, self._port, self._token

    def _token_text(self) -> str:
        with self._lock:
            return self._token

    # ---- 关闭 -------------------------------------------------
    def stop(self) -> None:
        with self._lock:
            self._closed = True
            conns = list(self._conns)
            self._conns.clear()
        for conn in conns:
            try:
                conn.close()
            except Exception:  # pragma: no cover - 关闭连接失败不影响退出
                pass

    # ---- 普通请求 ---------------------------------------------
    def _request_headers(
        self, client_headers: dict | None, body: bytes
    ) -> dict[str, str]:
        headers: dict[str, str] = {
            "Accept": "*/*",
            "User-Agent": "raspbot-pc-console/1.0",
        }
        client_headers = client_headers or {}
        content_type = client_headers.get("Content-Type") or client_headers.get("content-type")
        if body or content_type:
            headers["Content-Type"] = content_type or "application/json"
        accept = client_headers.get("Accept") or client_headers.get("accept")
        if accept:
            headers["Accept"] = accept
        token = self._token_text() or client_headers.get("X-Console-Token") or client_headers.get(
            "x-console-token"
        )
        if token:
            headers["X-Console-Token"] = token
        return headers

    def request(
        self,
        method: str,
        path: str,
        body: bytes = b"",
        headers: dict | None = None,
    ) -> WireResponse:
        """转发一次请求并完整读取响应。"""
        with self._lock:
            host, port, _ = self._host, self._port, self._token
            closed = self._closed
        if closed:
            raise UpstreamUnreachable("代理已关闭")
        conn = http.client.HTTPConnection(host, port, timeout=self._timeout)
        req_headers = self._request_headers(headers, body)
        try:
            # 连接阶段超时视为「不可达」（防火墙丢包 / 主机不存在）；读响应超时才视为 504
            try:
                conn.connect()
            except (TimeoutError, socket.timeout) as exc:
                raise UpstreamUnreachable(
                    f"无法连接 Pi 网关 {host}:{port}（连接超时 {self._timeout:g} s）"
                ) from exc
            except (
                ConnectionRefusedError,
                ConnectionResetError,
                ConnectionAbortedError,
                BrokenPipeError,
                socket.gaierror,
                OSError,
            ) as exc:
                raise UpstreamUnreachable(
                    f"无法连接 Pi 网关 {host}:{port}：{exc.__class__.__name__}: {exc}"
                ) from exc
            conn.request(method, path, body=body if body else None, headers=req_headers)
            resp = conn.getresponse()
            status = resp.status
            resp_headers = _filter_headers(resp.getheaders())
            data = resp.read()
        except (TimeoutError, socket.timeout) as exc:
            raise UpstreamTimeout(
                f"请求 Pi 网关超时（{self._timeout:g} s）：{host}:{port} {path}"
            ) from exc
        except (
            ConnectionRefusedError,
            ConnectionResetError,
            ConnectionAbortedError,
            BrokenPipeError,
            socket.gaierror,
            http.client.BadStatusLine,
            http.client.RemoteDisconnected,
            OSError,
        ) as exc:
            raise UpstreamUnreachable(
                f"无法连接 Pi 网关 {host}:{port}：{exc.__class__.__name__}: {exc}"
            ) from exc
        finally:
            try:
                conn.close()
            except Exception:
                pass
        return WireResponse(status, resp_headers, data)

    # ---- SSE --------------------------------------------------
    def event_stream(self, path: str, headers: dict | None = None) -> WireStream:
        """连接上游 SSE。

        返回 :class:`WireStream`：``buffered=True`` 表示上游没有返回事件流（按普通响应
        回传），否则 ``chunks`` 逐行产出上游字节（本地 HTTP 层负责 chunked 封装）。
        """
        with self._lock:
            host, port, _ = self._host, self._port, self._token
            closed = self._closed
        if closed:
            raise UpstreamUnreachable("代理已关闭")
        conn = http.client.HTTPConnection(host, port, timeout=self._timeout)
        req_headers = {
            "Accept": "text/event-stream",
            "Cache-Control": "no-cache",
            "User-Agent": "raspbot-pc-console/1.0",
        }
        token = self._token_text() or (headers or {}).get("X-Console-Token") or (
            headers or {}
        ).get("x-console-token")
        if token:
            req_headers["X-Console-Token"] = token
        try:
            conn.connect()
            if conn.sock is not None:
                conn.sock.settimeout(self._stream_timeout)
            conn.request("GET", path, headers=req_headers)
            resp = conn.getresponse()
        except (TimeoutError, socket.timeout) as exc:
            try:
                conn.close()
            except Exception:
                pass
            raise UpstreamUnreachable(
                f"无法连接 Pi 网关 SSE：{host}:{port} {path}（连接超时）"
            ) from exc
        except (
            ConnectionRefusedError,
            ConnectionResetError,
            ConnectionAbortedError,
            BrokenPipeError,
            socket.gaierror,
            http.client.BadStatusLine,
            http.client.RemoteDisconnected,
            OSError,
        ) as exc:
            try:
                conn.close()
            except Exception:
                pass
            raise UpstreamUnreachable(
                f"无法连接 Pi 网关 {host}:{port}：{exc.__class__.__name__}: {exc}"
            ) from exc

        status = resp.status
        resp_headers = _filter_headers(resp.getheaders())
        content_type = ""
        for key, value in resp_headers:
            if key.lower() == "content-type":
                content_type = value.lower()
                break
        if status != 200 or "text/event-stream" not in content_type:
            try:
                body = resp.read()
            except OSError:
                body = b""
            try:
                conn.close()
            except Exception:
                pass
            return WireStream(status, resp_headers, (), buffered=True, body=body)

        with self._lock:
            if self._closed:
                try:
                    conn.close()
                except Exception:
                    pass
                raise UpstreamUnreachable("代理已关闭")
            self._conns.add(conn)
        self_ref = self

        def _iter_chunks():
            try:
                while not self_ref._closed:
                    line = resp.readline()
                    if not line:
                        break
                    yield line
            except (OSError, http.client.HTTPException):
                return
            finally:
                with self_ref._lock:
                    self_ref._conns.discard(conn)
                try:
                    conn.close()
                except Exception:
                    pass

        return WireStream(status, resp_headers, _iter_chunks())


def probe_health(
    host: str, port: int, token: str = "", timeout: float = 2.0
) -> dict:
    """探测上游 ``/api/health``（``GET /local/ping`` 与 ``--fallback-demo`` 使用）。"""
    target = {"host": host, "port": int(port)}
    started = time.perf_counter()
    conn = http.client.HTTPConnection(host, int(port), timeout=timeout)
    try:
        headers = {"Accept": "application/json", "User-Agent": "raspbot-pc-console/1.0"}
        if token:
            headers["X-Console-Token"] = token
        conn.request("GET", "/api/health", headers=headers)
        resp = conn.getresponse()
        resp.read()
        latency = (time.perf_counter() - started) * 1000.0
        ok = resp.status == 200
        return {
            "reachable": ok,
            "latency_ms": round(latency, 1),
            "target": target,
            "error": None if ok else f"HTTP {resp.status}",
        }
    except Exception as exc:  # noqa: BLE001 - 探测接口需吞掉所有异常
        return {
            "reachable": False,
            "latency_ms": None,
            "target": target,
            "error": f"{exc.__class__.__name__}: {exc}",
        }
    finally:
        try:
            conn.close()
        except Exception:
            pass


__all__ = [
    "Upstream",
    "UpstreamError",
    "UpstreamUnreachable",
    "UpstreamTimeout",
    "WireResponse",
    "WireStream",
    "probe_health",
]
