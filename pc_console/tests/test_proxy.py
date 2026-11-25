"""真车模式反向代理测试：状态码/二进制/SSE 透传、token、502/504、目标切换。

使用本地启动的「假 Pi 网关」（ThreadingHTTPServer），不依赖真实 Pi。
"""

from __future__ import annotations

import json
import shutil
import socket
import threading
import time
import unittest
import urllib.error
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from console_server.config import Config
from console_server.http_server import create_server

_TMP_ROOT = Path(__file__).resolve().parent.parent / "_tmp"

FAKE_PNG = b"\x89PNG\r\n\x1a\nFAKE-PNG-BODY\x00\x01\x02\xff"


def make_temp_dir(prefix: str = "proxy_") -> Path:
    path = _TMP_ROOT / (prefix + uuid.uuid4().hex[:10])
    path.mkdir(parents=True, exist_ok=True)
    return path


def free_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def http_request(
    url: str, method: str = "GET", body: bytes | None = None, timeout: float = 5.0
) -> tuple[int, dict, bytes]:
    headers = {"Content-Type": "application/json"} if body is not None else {}
    request = urllib.request.Request(url, data=body, method=method, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return (
                response.status,
                {key.lower(): value for key, value in response.headers.items()},
                response.read(),
            )
    except urllib.error.HTTPError as exc:
        try:
            return (
                exc.code,
                {key.lower(): value for key, value in exc.headers.items()},
                exc.read(),
            )
        finally:
            exc.close()


class _QuietThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address) -> None:  # noqa: D401
        return


class FakeUpstream:
    """最小假 Pi 网关：记录请求，提供 JSON / 二进制 / SSE / 慢响应。"""

    def __init__(self, marker: str = "fake-pi") -> None:
        self.marker = marker
        self.requests: list[dict] = []
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, fmt, *args):  # noqa: A003
                return

            def _record(self, method: str, body: bytes = b"") -> None:
                outer.requests.append(
                    {
                        "method": method,
                        "path": self.path,
                        "headers": {k.lower(): v for k, v in self.headers.items()},
                        "body": body,
                    }
                )

            def _send_json(self, status: int, obj: dict) -> None:
                data = json.dumps(obj).encode("utf-8")
                try:
                    self.send_response(status)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                    pass

            def do_GET(self):  # noqa: N802
                self._record("GET")
                path = self.path.split("?")[0]
                if path == "/api/health":
                    self._send_json(
                        200,
                        {
                            "ok": True,
                            "name": outer.marker,
                            "version": "9.9.9",
                            "api": 1,
                            "server_time": time.time(),
                        },
                    )
                elif path == "/api/state":
                    self._send_json(200, {"api": 1, "marker": outer.marker})
                elif path == "/api/conflict":
                    self._send_json(
                        409,
                        {"error": {"code": "goal_active", "message": "已有活动目标"}},
                    )
                elif path == "/api/map.png":
                    self.send_response(200)
                    self.send_header("Content-Type", "image/png")
                    self.send_header("Content-Length", str(len(FAKE_PNG)))
                    self.end_headers()
                    self.wfile.write(FAKE_PNG)
                elif path == "/api/slow":
                    time.sleep(0.8)
                    self._send_json(200, {"ok": True})
                elif path == "/api/events":
                    self.send_response(200)
                    self.send_header(
                        "Content-Type", "text/event-stream; charset=utf-8"
                    )
                    self.send_header("Cache-Control", "no-cache")
                    self.send_header("Connection", "close")
                    self.end_headers()
                    try:
                        self.wfile.write(b"retry: 2000\n\n")
                        self.wfile.flush()
                        for index in range(3):
                            payload = json.dumps(
                                {"stamp": float(index), "frame": "map", "points": [0.0, 0.0]}
                            ).encode("utf-8")
                            self.wfile.write(b"event: scan\ndata: " + payload + b"\n\n")
                            self.wfile.flush()
                            time.sleep(0.03)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    self.close_connection = True
                else:
                    self._send_json(404, {"error": {"code": "not_found", "message": path}})

            def do_POST(self):  # noqa: N802
                length = int(self.headers.get("Content-Length") or 0)
                body = self.rfile.read(length) if length else b""
                self._record("POST", body)
                self._send_json(
                    200, {"ok": True, "name": outer.marker, "received": len(body)}
                )

        self.httpd = _QuietThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.httpd.daemon_threads = True
        self.port = int(self.httpd.server_address[1])
        self.thread = threading.Thread(
            target=self.httpd.serve_forever,
            kwargs={"poll_interval": 0.05},
            daemon=True,
        )
        self.thread.start()

    def stop(self) -> None:
        try:
            self.httpd.shutdown()
        except Exception:  # pragma: no cover
            pass
        try:
            self.httpd.server_close()
        except Exception:  # pragma: no cover
            pass
        self.thread.join(timeout=2.0)

    def find(self, path_suffix: str, method: str = "GET") -> dict | None:
        for request in self.requests:
            if request["method"] == method and request["path"].endswith(path_suffix):
                return request
        return None


class ProxyTestCase(unittest.TestCase):
    token = "sekret-token"

    def setUp(self) -> None:
        self.tmp = make_temp_dir()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.upstream = FakeUpstream("fake-pi-A")
        self.addCleanup(self.upstream.stop)
        self.config = Config(
            host="127.0.0.1",
            port=0,
            pi_host="127.0.0.1",
            pi_port=self.upstream.port,
            token=self.token,
            browser="none",
            config_path=str(self.tmp / "config.json"),
        )
        self.server = create_server(
            self.config,
            demo=False,
            upstream_timeout=0.4,
        )
        self.addCleanup(self.server.stop)
        self.server.start()
        self.base = self.server.url.rstrip("/")

    # ---- 透传 -----------------------------------------------------
    def test_health_passthrough(self) -> None:
        status, headers, body = http_request(self.base + "/api/health")
        self.assertEqual(status, 200)
        self.assertIn("application/json", headers.get("content-type", ""))
        data = json.loads(body)
        self.assertEqual(data["name"], "fake-pi-A")
        # 本地重新生成 Content-Length，不透传上游逐跳头
        self.assertEqual(int(headers["content-length"]), len(body))
        self.assertNotIn("transfer-encoding", headers)

    def test_token_forwarded(self) -> None:
        http_request(self.base + "/api/state")
        recorded = self.upstream.find("/api/state")
        self.assertIsNotNone(recorded)
        self.assertEqual(recorded["headers"].get("x-console-token"), self.token)

    def test_binary_passthrough(self) -> None:
        status, headers, body = http_request(self.base + "/api/map.png")
        self.assertEqual(status, 200)
        self.assertEqual(headers.get("content-type"), "image/png")
        self.assertEqual(body, FAKE_PNG)

    def test_status_code_passthrough(self) -> None:
        status, _headers, body = http_request(self.base + "/api/conflict")
        self.assertEqual(status, 409)
        self.assertEqual(json.loads(body)["error"]["code"], "goal_active")

    def test_post_body_forwarded(self) -> None:
        payload = json.dumps({"vx": 0.15, "vy": 0.0, "wz": 0.3, "seq": 7}).encode()
        status, _headers, body = http_request(
            self.base + "/api/teleop", method="POST", body=payload
        )
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["ok"])
        recorded = self.upstream.find("/api/teleop", method="POST")
        self.assertIsNotNone(recorded)
        self.assertEqual(recorded["body"], payload)
        self.assertEqual(
            recorded["headers"].get("content-type"), "application/json"
        )

    # ---- SSE ------------------------------------------------------
    def test_sse_proxy_streaming(self) -> None:
        request = urllib.request.Request(self.base + "/api/events")
        response = urllib.request.urlopen(request, timeout=10)
        try:
            headers = {key.lower(): value for key, value in response.headers.items()}
            self.assertIn("text/event-stream", headers.get("content-type", ""))
            self.assertEqual(headers.get("transfer-encoding"), "chunked")
            self.assertEqual(headers.get("x-accel-buffering"), "no")
            self.assertNotIn("content-length", headers)
            lines: list[str] = []
            events: list[str] = []
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline:
                raw = response.readline()
                if not raw:
                    break
                line = raw.decode("utf-8").strip()
                if line:
                    lines.append(line)
                if line.startswith("event: "):
                    events.append(line[len("event: ") :])
                if len(events) >= 3:
                    break
            self.assertIn("scan", events)
            self.assertIn("retry: 2000", lines)
        finally:
            response.close()

    # ---- 上游故障 -------------------------------------------------
    def test_unreachable_502(self) -> None:
        closed_port = free_port()
        self.server.backend.set_target("127.0.0.1", closed_port, self.token)
        status, _headers, body = http_request(self.base + "/api/state")
        self.assertEqual(status, 502)
        data = json.loads(body)
        self.assertEqual(data["error"]["code"], "upstream_unreachable")
        self.assertIn("message", data["error"])

    def test_timeout_504(self) -> None:
        status, _headers, body = http_request(self.base + "/api/slow")
        self.assertEqual(status, 504)
        self.assertEqual(json.loads(body)["error"]["code"], "upstream_timeout")

    # ---- /local/config 即时切换 -----------------------------------
    def test_local_config_switch_target(self) -> None:
        second = FakeUpstream("fake-pi-B")
        self.addCleanup(second.stop)
        payload = json.dumps(
            {"pi_host": "127.0.0.1", "pi_port": second.port, "token": "new-token"}
        ).encode()
        status, _headers, body = http_request(
            self.base + "/local/config", method="POST", body=payload
        )
        self.assertEqual(status, 200)
        self.assertEqual(
            json.loads(body)["target"], {"host": "127.0.0.1", "port": second.port}
        )
        # 代理目标即时切换
        status, _headers, body = http_request(self.base + "/api/health")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["name"], "fake-pi-B")
        self.assertEqual(
            second.find("/api/health")["headers"].get("x-console-token"),
            "new-token",
        )
        # 配置已持久化
        saved = json.loads(Path(self.config.config_path).read_text(encoding="utf-8"))
        self.assertEqual(saved["pi_port"], second.port)
        self.assertEqual(saved["token"], "new-token")

    def test_local_ping_reachable(self) -> None:
        status, _headers, body = http_request(self.base + "/local/ping")
        self.assertEqual(status, 200)
        data = json.loads(body)
        self.assertTrue(data["reachable"])
        self.assertIsNone(data["error"])
        self.assertGreaterEqual(data["latency_ms"], 0.0)


if __name__ == "__main__":
    unittest.main()
