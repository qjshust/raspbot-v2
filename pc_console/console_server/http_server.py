"""本地 HTTP 服务（仅标准库）。

职责（设计文档 §4.1）：

* 静态资源：``GET /`` → ``web/index.html``；只暴露 ``web/`` 下白名单后缀，
  防路径穿越、正确 MIME、``Cache-Control: no-store``；web 目录缺失/文件缺失时
  返回 404（不影响 API）。
* ``/local/config`` GET/POST、``/local/ping``：目标网关配置、连通性探测。
* ``/api/*``：真车模式反向代理到 Pi 网关；演示模式分发到内置仿真后端。
  * 普通请求保留方法/状态码/Content-Type/二进制；上游不可达 502 JSON。
  * SSE：HTTP/1.1 + ``Transfer-Encoding: chunked``，逐帧 flush，不缓冲整段，
    不带 Content-Length，附 ``X-Accel-Buffering: no``；不转发上游逐跳头。
* 可编程接口：``create_server(config, demo=...) -> ConsoleServer``，
  ``server.start() / stop() / url / port``，便于 unittest 与 tools/contract_check.py。
"""

from __future__ import annotations

import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

from .config import (
    VERSION,
    Config,
    ConfigError,
    save_config_locked,
)
from .demo import DemoBackend
from .upstream import (
    Upstream,
    UpstreamError,
    UpstreamTimeout,
    UpstreamUnreachable,
    WireResponse,
    WireStream,
    probe_health,
)

log = logging.getLogger("console.http")

MAX_JSON_BODY = 64 * 1024

#: 静态资源白名单后缀（设计文档 §4.1）
STATIC_TYPES: dict[str, str] = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".ico": "image/x-icon",
    ".json": "application/json; charset=utf-8",
    ".woff2": "font/woff2",
}

CORS_HEADERS = (
    ("Access-Control-Allow-Origin", "*"),
    ("Access-Control-Allow-Methods", "GET, POST, OPTIONS"),
    ("Access-Control-Allow-Headers", "Content-Type, X-Console-Token"),
    ("Access-Control-Max-Age", "600"),
)

#: 转发响应时丢弃的逐跳头（由本地服务器重新生成）
_DROP_RESPONSE_HEADERS = {
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


class _Abort(Exception):
    """请求已被提前应答（例如请求体过大），停止处理。"""


class _QuietThreadingHTTPServer(ThreadingHTTPServer):
    """HTTP 服务器：单连接异常（浏览器/SSE 客户端提前断开）只记 debug，不打印堆栈。"""

    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address) -> None:  # noqa: D401
        log.debug("连接异常结束 %s", client_address, exc_info=True)


class ConsoleServer:
    """PC 控制台本地 HTTP 服务。"""

    def __init__(
        self,
        config: Config,
        demo: bool = False,
        web_dir: str | Path | None = None,
        demo_maps_dir: str | Path | None = None,
        upstream_timeout: float = 5.0,
    ) -> None:
        self.config = config
        self.demo = bool(demo)
        base = Path(__file__).resolve().parent.parent
        self.web_dir = Path(web_dir) if web_dir else base / "web"
        self._shutdown = threading.Event()
        self._stopped = threading.Event()
        self._created = time.time()
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        if self.demo:
            self.backend = DemoBackend(demo_maps_dir=demo_maps_dir)
        else:
            self.backend = Upstream(
                config.pi_host,
                config.pi_port,
                config.token,
                timeout=upstream_timeout,
            )
        self._handler_class = _make_handler(self)

    # ---- 生命周期 ---------------------------------------------
    def start(self) -> int:
        """绑定端口并启动服务线程，返回实际端口（``--port 0`` 时由系统分配）。"""
        if self._httpd is not None:
            return self.port
        if self.demo:
            self.backend.start()
        httpd = _QuietThreadingHTTPServer(
            (self.config.host, self.config.port), self._handler_class
        )
        httpd.daemon_threads = True
        httpd.allow_reuse_address = True
        self._httpd = httpd
        self._thread = threading.Thread(
            target=httpd.serve_forever,
            kwargs={"poll_interval": 0.1},
            name="console-http",
            daemon=True,
        )
        self._thread.start()
        log.info(
            "本地服务已启动: http://%s:%s/ （%s）",
            self.display_host,
            self.port,
            "演示模式" if self.demo else f"代理 Pi {self.config.pi_host}:{self.config.pi_port}",
        )
        return self.port

    def stop(self) -> None:
        """优雅关闭：停止 HTTP 服务、清理 SSE 客户端与仿真线程。"""
        if self._shutdown.is_set():
            return
        self._shutdown.set()
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception:  # pragma: no cover
                pass
            try:
                self._httpd.server_close()
            except Exception:  # pragma: no cover
                pass
            self._httpd = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._thread = None
        try:
            self.backend.stop()
        except Exception:  # pragma: no cover
            log.exception("关闭后端失败")
        self._stopped.set()
        log.info("本地服务已关闭")

    def wait(self, timeout: float | None = None) -> bool:
        """阻塞直到 :meth:`stop` 被调用。"""
        return self._stopped.wait(timeout)

    # ---- 派生属性 ---------------------------------------------
    @property
    def port(self) -> int:
        if self._httpd is not None:
            return int(self._httpd.server_address[1])
        return int(self.config.port)

    @property
    def display_host(self) -> str:
        host = self.config.host
        if host in ("0.0.0.0", "::", ""):
            return "127.0.0.1"
        return host

    @property
    def url(self) -> str:
        return f"http://{self.display_host}:{self.port}/"

    @property
    def uptime(self) -> float:
        return time.time() - self._created

    # ===============================================================
    # 请求分发
    # ===============================================================
    def handle_request(self, handler: BaseHTTPRequestHandler, method: str) -> None:
        try:
            raw_path = handler.path
            split = urlsplit(raw_path)
            path = unquote(split.path or "/")
            if method == "OPTIONS":
                self._send_options(handler)
                return
            if path == "/api/events":
                if method == "GET":
                    self._handle_sse(handler, raw_path)
                elif method == "HEAD":
                    self._handle_sse(handler, raw_path, headers_only=True)
                else:
                    self._send_json(
                        handler,
                        405,
                        {
                            "error": {
                                "code": "method_not_allowed",
                                "message": "/api/events 只支持 GET",
                            }
                        },
                    )
                return
            if path.startswith("/api/"):
                self._handle_api(handler, method, raw_path)
                return
            if path.startswith("/local/"):
                self._handle_local(handler, method, path, split)
                return
            if method in ("GET", "HEAD"):
                self._serve_static(handler, method, path)
                return
            self._send_json(
                handler,
                405,
                {
                    "error": {
                        "code": "method_not_allowed",
                        "message": f"静态资源不支持 {method}",
                    }
                },
            )
        except _Abort:
            return
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return
        except Exception:  # pragma: no cover - 兜底
            log.exception("处理请求失败: %s %s", method, getattr(handler, "path", "?"))
            try:
                self._send_json(
                    handler,
                    500,
                    {
                        "error": {
                            "code": "internal_error",
                            "message": "PC 控制台内部错误，详见控制台日志",
                        }
                    },
                )
            except Exception:
                pass

    # ===============================================================
    # /local/*
    # ===============================================================
    def _handle_local(
        self,
        handler: BaseHTTPRequestHandler,
        method: str,
        path: str,
        split,
    ) -> None:
        if path == "/local/config":
            if method == "GET":
                self._local_config_get(handler)
                return
            if method == "POST":
                self._local_config_post(handler)
                return
            self._send_json(
                handler,
                405,
                {
                    "error": {
                        "code": "method_not_allowed",
                        "message": "/local/config 支持 GET/POST",
                    }
                },
            )
            return
        if path == "/local/ping":
            if method != "GET":
                self._send_json(
                    handler,
                    405,
                    {
                        "error": {
                            "code": "method_not_allowed",
                            "message": "/local/ping 只支持 GET",
                        }
                    },
                )
                return
            self._local_ping(handler)
            return
        self._send_json(
            handler,
            404,
            {"error": {"code": "not_found", "message": f"未知本地接口 {path}"}},
        )

    def _local_config_get(self, handler: BaseHTTPRequestHandler) -> None:
        self._send_json(
            handler,
            200,
            {
                "demo": self.demo,
                "version": VERSION,
                "web": {"host": self.config.host, "port": self.port},
                "target": {
                    "host": self.config.pi_host,
                    "port": self.config.pi_port,
                },
                "token_set": bool(self.config.token),
                "config_path": str(self.config.config_path),
            },
        )

    def _local_config_post(self, handler: BaseHTTPRequestHandler) -> None:
        if self.demo:
            self._send_json(
                handler,
                409,
                {
                    "error": {
                        "code": "demo_mode",
                        "message": "演示模式不能修改目标网关（请直接连接真车）",
                    }
                },
            )
            return
        data = self._read_json_body(handler)
        if data is None:
            return
        updates: dict = {}
        if "pi_host" in data or "pi_port" in data:
            if "pi_host" not in data or "pi_port" not in data:
                self._send_json(
                    handler,
                    400,
                    {
                        "error": {
                            "code": "bad_request",
                            "message": "pi_host 与 pi_port 必须同时提供",
                            "detail": {"field": "pi_host"},
                        }
                    },
                )
                return
            updates["pi_host"] = data["pi_host"]
            updates["pi_port"] = data["pi_port"]
        if "token" in data:
            updates["token"] = data["token"]
        if not updates:
            self._send_json(
                handler,
                400,
                {
                    "error": {
                        "code": "bad_request",
                        "message": "没有可更新的字段（pi_host/pi_port/token）",
                    }
                },
            )
            return
        candidate = self.config.copy()
        try:
            candidate.apply(updates)
        except ConfigError as exc:
            self._send_json(
                handler,
                400,
                {
                    "error": {
                        "code": "bad_request",
                        "message": exc.message,
                        "detail": {"field": exc.field},
                    }
                },
            )
            return
        try:
            save_config_locked(candidate)
        except OSError as exc:
            self._send_json(
                handler,
                500,
                {
                    "error": {
                        "code": "internal_error",
                        "message": f"保存配置文件失败：{exc}",
                    }
                },
            )
            return
        # 保存成功后即时生效
        self.config.pi_host = candidate.pi_host
        self.config.pi_port = candidate.pi_port
        self.config.token = candidate.token
        updater = getattr(self.backend, "set_target", None)
        if callable(updater):
            updater(self.config.pi_host, self.config.pi_port, self.config.token)
        log.info(
            "目标网关已更新为 %s:%s（token_set=%s）",
            self.config.pi_host,
            self.config.pi_port,
            bool(self.config.token),
        )
        self._send_json(
            handler,
            200,
            {
                "ok": True,
                "target": {
                    "host": self.config.pi_host,
                    "port": self.config.pi_port,
                },
                "token_set": bool(self.config.token),
                "config_path": str(self.config.config_path),
            },
        )

    def _local_ping(self, handler: BaseHTTPRequestHandler) -> None:
        if self.demo:
            self._send_json(
                handler,
                200,
                {
                    "reachable": True,
                    "latency_ms": 0.1,
                    "target": {"host": "demo", "port": 0},
                    "error": None,
                    "demo": True,
                },
            )
            return
        result = probe_health(
            self.config.pi_host,
            self.config.pi_port,
            self.config.token,
            timeout=2.0,
        )
        result["demo"] = False
        self._send_json(handler, 200, result)

    # ===============================================================
    # /api/*
    # ===============================================================
    def _handle_api(
        self, handler: BaseHTTPRequestHandler, method: str, raw_path: str
    ) -> None:
        body = b""
        if method in ("POST", "PUT", "PATCH"):
            try:
                body = self._read_body(handler)
            except _Abort:
                return
        headers = self._request_headers(handler)
        try:
            response = self.backend.request(method, raw_path, body, headers)
        except UpstreamUnreachable as exc:
            self._send_json(
                handler,
                502,
                {
                    "error": {
                        "code": "upstream_unreachable",
                        "message": str(exc),
                        "detail": {
                            "target": {
                                "host": self.config.pi_host,
                                "port": self.config.pi_port,
                            }
                        },
                    }
                },
            )
            return
        except UpstreamTimeout as exc:
            self._send_json(
                handler,
                504,
                {
                    "error": {
                        "code": "upstream_timeout",
                        "message": str(exc),
                    }
                },
            )
            return
        except UpstreamError as exc:
            self._send_json(
                handler,
                502,
                {"error": {"code": exc.code, "message": str(exc)}},
            )
            return
        self._relay_response(handler, response)

    # ---- SSE --------------------------------------------------
    def _handle_sse(
        self,
        handler: BaseHTTPRequestHandler,
        raw_path: str,
        headers_only: bool = False,
    ) -> None:
        headers = self._request_headers(handler)
        try:
            stream = self.backend.event_stream(raw_path, headers)
        except UpstreamUnreachable as exc:
            self._send_json(
                handler,
                502,
                {
                    "error": {
                        "code": "upstream_unreachable",
                        "message": str(exc),
                    }
                },
            )
            return
        except UpstreamTimeout as exc:
            self._send_json(
                handler,
                504,
                {"error": {"code": "upstream_timeout", "message": str(exc)}},
            )
            return
        except UpstreamError as exc:
            self._send_json(
                handler,
                502,
                {"error": {"code": exc.code, "message": str(exc)}},
            )
            return
        if stream.buffered:
            self._relay_response(handler, WireResponse(stream.status, stream.headers, stream.body))
            return
        if headers_only:
            # HEAD：只回响应头（SSE 的事件流主体不发送）
            handler.send_response(stream.status)
            handler.send_header("Content-Type", "text/event-stream; charset=utf-8")
            handler.send_header("Cache-Control", "no-cache")
            handler.send_header("X-Accel-Buffering", "no")
            handler.send_header("Content-Length", "0")
            for key, value in CORS_HEADERS:
                handler.send_header(key, value)
            handler.end_headers()
            close = getattr(stream.chunks, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass
            return
        self._stream_chunked(handler, stream)

    def _stream_chunked(
        self, handler: BaseHTTPRequestHandler, stream: WireStream
    ) -> None:
        handler.send_response(stream.status)
        have_content_type = False
        for key, value in self._filter_response_headers(stream.headers):
            if key.lower() == "content-type":
                have_content_type = True
            handler.send_header(key, value)
        if not have_content_type:
            handler.send_header("Content-Type", "text/event-stream; charset=utf-8")
        handler.send_header("Cache-Control", "no-cache")
        handler.send_header("X-Accel-Buffering", "no")
        handler.send_header("Connection", "close")
        for key, value in CORS_HEADERS:
            handler.send_header(key, value)
        # 浏览器 EventSource 依赖增量解析：HTTP/1.1 用 chunked，逐帧 flush
        chunked = handler.request_version != "HTTP/1.0"
        if chunked:
            handler.send_header("Transfer-Encoding", "chunked")
        handler.end_headers()
        handler.close_connection = True
        chunks = stream.chunks
        try:
            for chunk in chunks:
                if not chunk:
                    continue
                if chunked:
                    handler.wfile.write(f"{len(chunk):x}\r\n".encode("ascii"))
                    handler.wfile.write(chunk)
                    handler.wfile.write(b"\r\n")
                else:
                    handler.wfile.write(chunk)
                handler.wfile.flush()
                if self._shutdown.is_set():
                    break
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            close = getattr(chunks, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass
            if chunked:
                try:
                    handler.wfile.write(b"0\r\n\r\n")
                    handler.wfile.flush()
                except OSError:
                    pass

    # ---- 普通响应 ---------------------------------------------
    def _relay_response(
        self, handler: BaseHTTPRequestHandler, response: WireResponse
    ) -> None:
        handler.send_response(response.status)
        have_content_type = False
        have_cache = False
        for key, value in self._filter_response_headers(response.headers):
            lowered = key.lower()
            if lowered == "content-type":
                have_content_type = True
            if lowered == "cache-control":
                have_cache = True
            handler.send_header(key, value)
        if not have_content_type:
            handler.send_header("Content-Type", "application/json; charset=utf-8")
        if not have_cache:
            handler.send_header("Cache-Control", "no-store")
        handler.send_header("Content-Length", str(len(response.body)))
        for key, value in CORS_HEADERS:
            handler.send_header(key, value)
        handler.end_headers()
        if handler.command != "HEAD" and response.body:
            handler.wfile.write(response.body)

    @staticmethod
    def _filter_response_headers(
        headers: list[tuple[str, str]],
    ) -> list[tuple[str, str]]:
        out: list[tuple[str, str]] = []
        for key, value in headers:
            lowered = key.lower()
            if lowered in _DROP_RESPONSE_HEADERS or lowered.startswith("proxy-"):
                continue
            out.append((key, value))
        return out

    # ===============================================================
    # 静态资源
    # ===============================================================
    def _serve_static(
        self, handler: BaseHTTPRequestHandler, method: str, path: str
    ) -> None:
        relative = path.lstrip("/")
        if not relative:
            relative = "index.html"
        elif relative.endswith("/"):
            relative += "index.html"
        if "\x00" in relative or "\\" in relative:
            self._send_text(handler, 404, "404 Not Found")
            return
        segments = relative.split("/")
        if any(segment == ".." for segment in segments):
            log.warning("拒绝可疑静态路径: %r", path)
            self._send_text(handler, 404, "404 Not Found")
            return
        try:
            base = self.web_dir.resolve()
            target = (base / relative).resolve()
        except OSError:
            self._send_text(handler, 404, "404 Not Found")
            return
        if base != target and base not in target.parents:
            self._send_text(handler, 404, "404 Not Found")
            return
        suffix = target.suffix.lower()
        content_type = STATIC_TYPES.get(suffix)
        if content_type is None:
            self._send_text(handler, 404, "404 Not Found（后缀不在白名单内）")
            return
        if not target.is_file():
            self._send_text(handler, 404, f"404 Not Found: {relative}")
            return
        try:
            data = target.read_bytes()
        except OSError:
            self._send_text(handler, 500, "500 读取静态文件失败")
            return
        handler.send_response(200)
        handler.send_header("Content-Type", content_type)
        handler.send_header("Content-Length", str(len(data)))
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("X-Content-Type-Options", "nosniff")
        for key, value in CORS_HEADERS:
            handler.send_header(key, value)
        handler.end_headers()
        if method != "HEAD":
            handler.wfile.write(data)

    # ===============================================================
    # 请求/响应工具
    # ===============================================================
    @staticmethod
    def _request_headers(handler: BaseHTTPRequestHandler) -> dict:
        keys = (
            "Content-Type",
            "Accept",
            "X-Console-Token",
            "If-None-Match",
            "User-Agent",
        )
        out: dict = {}
        for key in keys:
            value = handler.headers.get(key)
            if value:
                out[key] = value
        return out

    def _read_body(self, handler: BaseHTTPRequestHandler) -> bytes:
        length_text = handler.headers.get("Content-Length")
        transfer_encoding = (handler.headers.get("Transfer-Encoding") or "").lower()
        if length_text is None:
            # 无 Content-Length：POST 无 body 的正常情况（curl -X POST 等）按空 body 处理
            if "chunked" in transfer_encoding:
                self._send_json(
                    handler,
                    400,
                    {
                        "error": {
                            "code": "bad_request",
                            "message": "不支持 chunked 请求体，请带 Content-Length",
                        }
                    },
                )
                raise _Abort()
            return b""
        try:
            length = int(length_text)
        except (TypeError, ValueError):
            self._send_json(
                handler,
                400,
                {"error": {"code": "bad_request", "message": "Content-Length 非法"}},
            )
            raise _Abort()
        if length < 0:
            self._send_json(
                handler,
                400,
                {"error": {"code": "bad_request", "message": "Content-Length 非法"}},
            )
            raise _Abort()
        if length > MAX_JSON_BODY:
            self._send_json(
                handler,
                413,
                {
                    "error": {
                        "code": "payload_too_large",
                        "message": f"请求体超过 {MAX_JSON_BODY // 1024} KB 限制",
                    }
                },
            )
            raise _Abort()
        data = handler.rfile.read(length)
        if len(data) != length:
            self._send_json(
                handler,
                400,
                {"error": {"code": "bad_request", "message": "请求体读取不完整"}},
            )
            raise _Abort()
        return data

    def _read_json_body(self, handler: BaseHTTPRequestHandler) -> dict | None:
        try:
            body = self._read_body(handler)
        except _Abort:
            return None
        if not body:
            return {}
        try:
            data = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._send_json(
                handler,
                400,
                {
                    "error": {
                        "code": "bad_request",
                        "message": f"请求体不是合法 JSON：{exc}",
                    }
                },
            )
            return None
        if not isinstance(data, dict):
            self._send_json(
                handler,
                400,
                {
                    "error": {
                        "code": "bad_request",
                        "message": "请求体必须是 JSON 对象",
                    }
                },
            )
            return None
        return data

    def _send_options(self, handler: BaseHTTPRequestHandler) -> None:
        handler.send_response(204)
        for key, value in CORS_HEADERS:
            handler.send_header(key, value)
        handler.send_header("Content-Length", "0")
        handler.end_headers()

    def _send_json(
        self, handler: BaseHTTPRequestHandler, status: int, obj: dict
    ) -> None:
        data = json.dumps(obj, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        handler.send_response(status)
        handler.send_header("Content-Type", "application/json; charset=utf-8")
        handler.send_header("Content-Length", str(len(data)))
        handler.send_header("Cache-Control", "no-store")
        for key, value in CORS_HEADERS:
            handler.send_header(key, value)
        handler.end_headers()
        if handler.command != "HEAD":
            handler.wfile.write(data)

    def _send_text(
        self, handler: BaseHTTPRequestHandler, status: int, text: str
    ) -> None:
        data = text.encode("utf-8")
        handler.send_response(status)
        handler.send_header("Content-Type", "text/plain; charset=utf-8")
        handler.send_header("Content-Length", str(len(data)))
        handler.send_header("Cache-Control", "no-store")
        for key, value in CORS_HEADERS:
            handler.send_header(key, value)
        handler.end_headers()
        if handler.command != "HEAD":
            handler.wfile.write(data)


def _make_handler(app: ConsoleServer):
    class ConsoleRequestHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "RaspbotConsole/1.0"
        sys_version = ""

        def log_message(self, fmt: str, *args) -> None:  # noqa: A003
            log.debug("%s - %s", self.address_string(), fmt % args)

        def _dispatch(self, method: str) -> None:
            app.handle_request(self, method)

        def do_GET(self) -> None:  # noqa: N802
            self._dispatch("GET")

        def do_HEAD(self) -> None:  # noqa: N802
            self._dispatch("HEAD")

        def do_POST(self) -> None:  # noqa: N802
            self._dispatch("POST")

        def do_OPTIONS(self) -> None:  # noqa: N802
            self._dispatch("OPTIONS")

        def do_PUT(self) -> None:  # noqa: N802
            self._dispatch("PUT")

        def do_PATCH(self) -> None:  # noqa: N802
            self._dispatch("PATCH")

        def do_DELETE(self) -> None:  # noqa: N802
            self._dispatch("DELETE")

    return ConsoleRequestHandler


def create_server(
    config: Config,
    demo: bool = False,
    web_dir: str | Path | None = None,
    demo_maps_dir: str | Path | None = None,
    upstream_timeout: float = 5.0,
) -> ConsoleServer:
    """创建（未启动的）控制台服务；调用 ``server.start()`` 后监听。

    unittest 用 ``Config(host="127.0.0.1", port=0, ...)`` + 本函数即可拿到临时端口。
    """
    return ConsoleServer(
        config,
        demo=demo,
        web_dir=web_dir,
        demo_maps_dir=demo_maps_dir,
        upstream_timeout=upstream_timeout,
    )


__all__ = ["ConsoleServer", "create_server", "MAX_JSON_BODY", "STATIC_TYPES"]
