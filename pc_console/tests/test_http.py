"""本地 HTTP 服务测试：静态资源、防穿越、MIME、/local/*、请求体积与方法限制。

运行::

    cd pc_console && python -m unittest discover -s tests -v
"""

from __future__ import annotations

import json
import shutil
import socket
import unittest
import urllib.error
import urllib.request
import uuid
from http.client import HTTPConnection
from pathlib import Path

from console_server.config import Config
from console_server.http_server import create_server

_TMP_ROOT = Path(__file__).resolve().parent.parent / "_tmp"


def make_temp_dir(prefix: str = "http_") -> Path:
    path = _TMP_ROOT / (prefix + uuid.uuid4().hex[:10])
    path.mkdir(parents=True, exist_ok=True)
    return path


def free_port() -> int:
    """取一个当前空闲的本地端口（用于「不可达上游」测试）。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def http_request(
    url: str,
    method: str = "GET",
    body: bytes | None = None,
    timeout: float = 5.0,
    headers: dict | None = None,
) -> tuple[int, dict, bytes]:
    request = urllib.request.Request(url, data=body, method=method, headers=headers or {})
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


def json_request(
    base: str, path: str, method: str = "GET", body: dict | None = None
) -> tuple[int, dict | None]:
    raw = json.dumps(body).encode() if body is not None else None
    status, _headers, data = http_request(
        base + path,
        method=method,
        body=raw,
        headers={"Content-Type": "application/json"} if raw is not None else None,
    )
    return status, (json.loads(data) if data else None)


class _ServerCase(unittest.TestCase):
    demo = True

    def setUp(self) -> None:
        self.tmp = make_temp_dir()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        # 自建 web 目录（不依赖 WP-C3 的真实前端内容）
        self.web = self.tmp / "web"
        (self.web / "js").mkdir(parents=True)
        (self.web / "css").mkdir()
        (self.web / "index.html").write_text(
            "<!doctype html><html><body>test</body></html>", encoding="utf-8"
        )
        (self.web / "js" / "app.js").write_text("export const x = 1;\n", encoding="utf-8")
        (self.web / "css" / "style.css").write_text("body{color:#fff}\n", encoding="utf-8")
        (self.web / "icon.svg").write_text("<svg></svg>", encoding="utf-8")
        (self.web / "data.json").write_text("{}", encoding="utf-8")
        (self.web / "secret.txt").write_text("do not serve", encoding="utf-8")
        self.config = Config(
            host="127.0.0.1",
            port=0,
            pi_host="127.0.0.1",
            pi_port=1,
            token="",
            browser="none",
            config_path=str(self.tmp / "config.json"),
        )
        self.server = create_server(
            self.config,
            demo=self.demo,
            web_dir=self.web,
            demo_maps_dir=str(self.tmp / "demo_maps"),
            upstream_timeout=0.4,
        )
        self.addCleanup(self.server.stop)
        self.server.start()
        self.base = self.server.url.rstrip("/")

    def raw_request(self, path: str, method: str = "GET") -> tuple[int, dict, bytes]:
        conn = HTTPConnection("127.0.0.1", self.server.port, timeout=5.0)
        try:
            conn.request(method, path)
            response = conn.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            conn.close()


class StaticServingTest(_ServerCase):
    demo = True

    def test_index_served(self) -> None:
        status, headers, body = http_request(self.base + "/")
        self.assertEqual(status, 200)
        self.assertIn("text/html", headers["content-type"])
        self.assertIn(b"<!doctype html", body)
        # /index.html 同样可访问
        status, _headers, body2 = http_request(self.base + "/index.html")
        self.assertEqual(status, 200)
        self.assertEqual(body, body2)

    def test_mime_types_and_no_store(self) -> None:
        expected = {
            "/js/app.js": "text/javascript",
            "/css/style.css": "text/css",
            "/icon.svg": "image/svg+xml",
            "/data.json": "application/json",
        }
        for path, content_type in expected.items():
            status, headers, body = http_request(self.base + path)
            self.assertEqual(status, 200, path)
            self.assertIn(content_type, headers.get("content-type", ""), path)
            self.assertEqual(headers.get("cache-control"), "no-store", path)
            self.assertTrue(body, path)

    def test_suffix_whitelist(self) -> None:
        status, _headers, _body = http_request(self.base + "/secret.txt")
        self.assertEqual(status, 404)
        status, _headers, _body = http_request(self.base + "/noext")
        self.assertEqual(status, 404)

    def test_path_traversal_blocked(self) -> None:
        candidates = [
            "/../config.json",
            "/%2e%2e/config.json",
            "/js/../../config.json",
            "/js/..%2f..%2fconfig.json",
            "/..%5cconfig.json",
        ]
        for path in candidates:
            status, _headers, _body = self.raw_request(path)
            self.assertEqual(status, 404, f"路径穿越未被拦截: {path}")

    def test_missing_static_404(self) -> None:
        status, headers, body = http_request(self.base + "/missing.js")
        self.assertEqual(status, 404)
        self.assertIn("text/plain", headers.get("content-type", ""))
        self.assertIn(b"404", body)
        status, _headers, _body = http_request(self.base + "/js/")
        self.assertEqual(status, 404)

    def test_head_request(self) -> None:
        expected = len((self.web / "js" / "app.js").read_bytes())
        status, headers, body = http_request(self.base + "/js/app.js", method="HEAD")
        self.assertEqual(status, 200)
        self.assertEqual(body, b"")
        self.assertEqual(int(headers["content-length"]), expected)

    def test_options_cors(self) -> None:
        status, headers, body = http_request(self.base + "/api/state", method="OPTIONS")
        self.assertEqual(status, 204)
        self.assertEqual(headers.get("access-control-allow-origin"), "*")
        self.assertIn("X-Console-Token", headers.get("access-control-allow-headers", ""))
        self.assertEqual(body, b"")


class MissingWebDirTest(unittest.TestCase):
    def test_missing_web_dir_does_not_break_api(self) -> None:
        tmp = make_temp_dir("http_noweb_")
        self.addCleanup(shutil.rmtree, tmp, True)
        config = Config(host="127.0.0.1", port=0, config_path=str(tmp / "config.json"))
        server = create_server(
            config,
            demo=True,
            web_dir=tmp / "does_not_exist",
            demo_maps_dir=str(tmp / "demo_maps"),
        )
        self.addCleanup(server.stop)
        server.start()
        base = server.url.rstrip("/")
        status, _headers, _body = http_request(base + "/")
        self.assertEqual(status, 404)
        status, _headers, body = http_request(base + "/api/health")
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["ok"])


class LocalEndpointsDemoTest(_ServerCase):
    demo = True

    def test_config_get(self) -> None:
        status, headers, body = http_request(self.base + "/local/config")
        self.assertEqual(status, 200)
        self.assertIn("application/json", headers.get("content-type", ""))
        data = json.loads(body)
        self.assertTrue(data["demo"])
        self.assertEqual(data["version"], "1.0.0")
        self.assertEqual(data["web"]["host"], "127.0.0.1")
        self.assertEqual(data["web"]["port"], self.server.port)
        self.assertEqual(data["target"], {"host": "127.0.0.1", "port": 1})
        self.assertFalse(data["token_set"])
        self.assertTrue(data["config_path"])

    def test_config_post_rejected_in_demo(self) -> None:
        status, data = json_request(
            self.base,
            "/local/config",
            method="POST",
            body={"pi_host": "192.168.1.2", "pi_port": 8080},
        )
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "demo_mode")

    def test_ping_demo(self) -> None:
        status, data = json_request(self.base, "/local/ping")
        self.assertEqual(status, 200)
        self.assertTrue(data["reachable"])
        self.assertIsNone(data["error"])
        self.assertTrue(data["demo"])


class LocalEndpointsRealTest(_ServerCase):
    demo = False

    def setUp(self) -> None:
        super().setUp()
        self.closed_port = free_port()

    def test_config_get_real_mode(self) -> None:
        status, data = json_request(self.base, "/local/config")
        self.assertEqual(status, 200)
        self.assertFalse(data["demo"])
        self.assertEqual(data["config_path"], str(self.tmp / "config.json"))

    def test_ping_unreachable(self) -> None:
        self.config.pi_port = self.closed_port
        self.server.backend.set_target(self.config.pi_host, self.closed_port, self.config.token)
        status, data = json_request(self.base, "/local/ping")
        self.assertEqual(status, 200)
        self.assertFalse(data["reachable"])
        self.assertIsNone(data["latency_ms"])
        self.assertTrue(data["error"])

    def test_config_post_validation(self) -> None:
        # 缺 pi_port
        status, data = json_request(
            self.base, "/local/config", method="POST", body={"pi_host": "1.2.3.4"}
        )
        self.assertEqual(status, 400)
        self.assertEqual(data["error"]["code"], "bad_request")
        # 非法 host
        status, data = json_request(
            self.base,
            "/local/config",
            method="POST",
            body={"pi_host": "http://1.2.3.4", "pi_port": 8080},
        )
        self.assertEqual(status, 400)
        self.assertEqual(data["error"]["detail"]["field"], "pi_host")
        # 非法端口
        status, data = json_request(
            self.base,
            "/local/config",
            method="POST",
            body={"pi_host": "1.2.3.4", "pi_port": 70000},
        )
        self.assertEqual(status, 400)
        # 无字段
        status, data = json_request(self.base, "/local/config", method="POST", body={})
        self.assertEqual(status, 400)

    def test_config_post_persists(self) -> None:
        status, data = json_request(
            self.base,
            "/local/config",
            method="POST",
            body={"pi_host": "192.168.50.99", "pi_port": 8181, "token": "secret-token"},
        )
        self.assertEqual(status, 200)
        self.assertEqual(data["target"], {"host": "192.168.50.99", "port": 8181})
        self.assertTrue(data["token_set"])
        config_path = Path(self.config.config_path)
        self.assertTrue(config_path.is_file())
        saved = json.loads(config_path.read_text(encoding="utf-8"))
        self.assertEqual(saved["pi_host"], "192.168.50.99")
        self.assertEqual(saved["pi_port"], 8181)
        self.assertEqual(saved["token"], "secret-token")
        # GET 即时反映
        status, data = json_request(self.base, "/local/config")
        self.assertEqual(data["target"], {"host": "192.168.50.99", "port": 8181})
        self.assertTrue(data["token_set"])

    def test_api_unreachable_502(self) -> None:
        self.config.pi_port = self.closed_port
        self.server.backend.set_target(self.config.pi_host, self.closed_port, "")
        status, headers, body = http_request(self.base + "/api/state")
        self.assertEqual(status, 502)
        self.assertIn("application/json", headers.get("content-type", ""))
        data = json.loads(body)
        self.assertEqual(data["error"]["code"], "upstream_unreachable")
        self.assertIn("message", data["error"])

    def test_sse_unreachable_502(self) -> None:
        self.config.pi_port = self.closed_port
        self.server.backend.set_target(self.config.pi_host, self.closed_port, "")
        status, _headers, body = http_request(self.base + "/api/events")
        self.assertEqual(status, 502)
        self.assertEqual(json.loads(body)["error"]["code"], "upstream_unreachable")

    def test_unknown_local_404(self) -> None:
        status, data = json_request(self.base, "/local/unknown")
        self.assertEqual(status, 404)
        self.assertEqual(data["error"]["code"], "not_found")


class PayloadAndMethodTest(_ServerCase):
    demo = True

    def test_body_too_large(self) -> None:
        big = b'{"vx":0.1,"pad":"' + b"x" * (70 * 1024) + b'"}'
        status, _headers, body = http_request(
            self.base + "/api/teleop",
            method="POST",
            body=big,
            headers={"Content-Type": "application/json"},
        )
        self.assertEqual(status, 413)
        self.assertEqual(json.loads(body)["error"]["code"], "payload_too_large")

    def test_invalid_json_400(self) -> None:
        status, _headers, body = http_request(
            self.base + "/api/teleop",
            method="POST",
            body=b"{not-json",
            headers={"Content-Type": "application/json"},
        )
        self.assertEqual(status, 400)
        self.assertEqual(json.loads(body)["error"]["code"], "bad_request")

    def test_unknown_api_404(self) -> None:
        status, data = json_request(self.base, "/api/does_not_exist")
        self.assertEqual(status, 404)
        self.assertEqual(data["error"]["code"], "not_found")

    def test_wrong_method_405(self) -> None:
        status, data = json_request(self.base, "/api/teleop")
        self.assertEqual(status, 405)
        self.assertEqual(data["error"]["code"], "method_not_allowed")

    def test_post_events_405(self) -> None:
        status, data = json_request(self.base, "/api/events", method="POST", body={})
        self.assertEqual(status, 405)

    def test_head_api(self) -> None:
        status, headers, body = http_request(self.base + "/api/health", method="HEAD")
        self.assertEqual(status, 200)
        self.assertEqual(body, b"")
        self.assertGreater(int(headers["content-length"]), 0)

    def test_post_without_body(self) -> None:
        # curl -X POST 不带 body / 无 Content-Length 时按空 body 处理
        status, _headers, body = self.raw_request("/api/session/stop", method="POST")
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["ok"])

    def test_head_events(self) -> None:
        status, headers, body = http_request(self.base + "/api/events", method="HEAD")
        self.assertEqual(status, 200)
        self.assertEqual(body, b"")
        self.assertIn("text/event-stream", headers.get("content-type", ""))


if __name__ == "__main__":
    unittest.main()
