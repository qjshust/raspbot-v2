"""HTTP + SSE 层：``ThreadingHTTPServer``、路由、token/CORS、SSE（仅标准库）。

契约（设计文档 §3）：

- 全部接口在 ``/api/`` 下；JSON 用 compact separators；
- 可选 token：``X-Console-Token`` 头，GET 接口也允许 ``?token=``；
- CORS：``*`` / ``GET, POST, OPTIONS`` / ``Content-Type, X-Console-Token``；
- JSON 请求体 ≤ 64 KB；SSE 客户端 ≤ 4（超出 503）；
- SSE：HTTP/1.1 + ``Transfer-Encoding: chunked``，逐帧 flush，无 Content-Length，
  首帧 ``retry: 2000``，空闲 10 s 发 ``: ping``，并带 ``X-Accel-Buffering: no``
  （避免反向代理缓冲）；普通 JSON 路由用 Content-Length + keep-alive。

本模块不导入 rclpy，可离线用假 controller 启动真实 HTTP 服务测试。
"""

from __future__ import annotations

import queue
import re
import select
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

from . import protocol
from .protocol import ApiError, ErrorCode
from .state import CLOSE_SENTINEL

#: GET JSON 路由：path → (controller 方法名, 参数来源)
GET_ROUTES = {
    '/api/health': ('health', 'none'),
    '/api/state': ('state_snapshot', 'none'),
    '/api/map/meta': ('map_meta', 'none'),
    '/api/maps': ('maps', 'none'),
    '/api/logs': ('logs', 'query'),
    '/api/session/logs': ('session_logs', 'query'),
    '/api/ros': ('ros_info', 'none'),
}

#: GET 二进制路由
GET_BINARY_ROUTES = {
    '/api/map.png': ('map_png', 'version'),
}

#: POST JSON 路由
POST_ROUTES = {
    '/api/map/save': ('map_save', 'body'),
    '/api/session/start': ('session_start', 'body'),
    '/api/session/stop': ('session_stop', 'none'),
    '/api/teleop': ('teleop', 'body'),
    '/api/teleop/stop': ('teleop_stop', 'none'),
    '/api/estop': ('estop', 'body'),
    '/api/goal': ('goal', 'body'),
    '/api/goal/cancel': ('goal_cancel', 'none'),
    '/api/initialpose': ('initialpose', 'body'),
    '/api/nav/activate': ('nav_activate', 'none'),
    '/api/actuator/buzzer': ('actuator_buzzer', 'body'),
    '/api/actuator/led': ('actuator_led', 'body'),
    '/api/actuator/servo': ('actuator_servo', 'body'),
}

#: ``/api/maps/<name>/preview.png``
PREVIEW_RE = re.compile(r'^/api/maps/([^/]+)/preview\.png$')

#: SSE 路由
SSE_PATHS = ('/api/events',)

#: SSE 空闲 ping 间隔（§3.2：每 10 s 无数据发注释）
SSE_PING_INTERVAL_S = 10.0

#: SSE 客户端队列读取超时（用于检查关闭标志/ping）
SSE_QUEUE_TIMEOUT_S = 1.0

#: 构造路由条目的轻量结构
class _Route:
    __slots__ = ('kind', 'name', 'arg', 'extra')

    def __init__(self, kind, name, arg='none', extra=None):
        self.kind = kind      # 'json' | 'binary' | 'sse'
        self.name = name      # controller 方法名
        self.arg = arg        # 'none' | 'body' | 'query' | 'version' | 'name'
        self.extra = extra

    def __repr__(self):  # pragma: no cover - 调试
        return '_Route(%r, %r, %r)' % (self.kind, self.name, self.arg)


def _cors_headers():
    return {
        'Access-Control-Allow-Origin': '*',
        'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
        'Access-Control-Allow-Headers': 'Content-Type, X-Console-Token',
    }


class HttpApi:
    """把 :class:`raspbot_console.controller.Controller` 暴露成 HTTP/SSE 服务。"""

    def __init__(self, controller, config=None):
        self.controller = controller
        self.config = config if config is not None else getattr(
            controller, 'config', None)
        self._server = None
        self._thread = None
        self.stop_event = threading.Event()
        self.running = False

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def start(self, host=None, port=None):
        """启动监听线程；返回 self。端口 0 时由系统分配（测试用）。"""
        if self._server is not None:
            return self
        if self.config is None:
            raise RuntimeError('HttpApi 缺少 config')
        listen_host = self.config.host if host is None else host
        listen_port = self.config.port if port is None else port
        self._server = _ConsoleHTTPServer((listen_host, int(listen_port)),
                                          self)
        self._thread = threading.Thread(target=self._serve,
                                        name='console-http', daemon=True)
        self._thread.start()
        self.running = True
        return self

    def _serve(self):
        try:
            self._server.serve_forever(poll_interval=0.2)
        except Exception:  # pragma: no cover - 退出路径
            pass

    def stop(self):
        """停止 HTTP 服务（不负责关闭事件总线）。"""
        self.running = False
        self.stop_event.set()
        server = self._server
        self._server = None
        if server is not None:
            try:
                server.shutdown()
            except Exception:  # pragma: no cover
                pass
            try:
                server.server_close()
            except Exception:  # pragma: no cover
                pass
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=3.0)

    @property
    def port(self):
        server = self._server
        if server is None:
            return None
        return int(server.server_address[1])

    @property
    def address(self):
        server = self._server
        if server is None:
            return None
        host, port = server.server_address[:2]
        return str(host), int(port)

    # ------------------------------------------------------------------
    # 鉴权 / 路由
    # ------------------------------------------------------------------
    def authenticate(self, method, path, split, headers):
        """token 校验（``/api/health`` 免 token，§3.3）。"""
        token = getattr(self.config, 'token', '') if self.config else ''
        if not token:
            return
        if path == '/api/health':
            return
        header = headers.get('X-Console-Token') if headers is not None else None
        if header == token:
            return
        if method == 'GET':
            query = parse_qs(split.query, keep_blank_values=True)
            if token in query.get('token', []):
                return
        raise ApiError(ErrorCode.BAD_TOKEN,
                       'token 无效或缺失（请求头 X-Console-Token，'
                       'GET 可用 ?token=）', status=401)

    def route(self, method, path):
        """解析 (method, path) → :class:`_Route`；未知/方法不符抛 ApiError。"""
        if method == 'GET':
            if path in SSE_PATHS:
                return _Route('sse', 'events')
            if path in GET_ROUTES:
                name, arg = GET_ROUTES[path]
                return _Route('json', name, arg)
            if path in GET_BINARY_ROUTES:
                name, arg = GET_BINARY_ROUTES[path]
                return _Route('binary', name, arg)
            match = PREVIEW_RE.match(path)
            if match:
                return _Route('binary', 'map_preview', 'name',
                              unquote(match.group(1)))
            if path in POST_ROUTES:
                raise ApiError(ErrorCode.METHOD_NOT_ALLOWED,
                               '该接口只接受 POST：%s' % path, status=405)
            raise ApiError(ErrorCode.NOT_FOUND, '接口不存在：%s' % path,
                           status=404)
        if method == 'POST':
            if path in POST_ROUTES:
                name, arg = POST_ROUTES[path]
                return _Route('json', name, arg)
            if path in GET_ROUTES or path in GET_BINARY_ROUTES \
                    or path in SSE_PATHS or PREVIEW_RE.match(path):
                raise ApiError(ErrorCode.METHOD_NOT_ALLOWED,
                               '该接口只接受 GET：%s' % path, status=405)
            raise ApiError(ErrorCode.NOT_FOUND, '接口不存在：%s' % path,
                           status=404)
        raise ApiError(ErrorCode.METHOD_NOT_ALLOWED,
                       '不支持的 HTTP 方法：%s' % method, status=405)

    # ------------------------------------------------------------------
    # 执行
    # ------------------------------------------------------------------
    def execute(self, route, query, body):
        """调用 controller 方法，归一化为 ``(status, headers, payload)``。"""
        controller = self.controller
        method = getattr(controller, route.name, None)
        if method is None:
            raise ApiError(ErrorCode.INTERNAL,
                           '网关未实现接口：%s' % route.name, status=500)
        try:
            if route.arg == 'body':
                result = method(body if body is not None else {})
            elif route.arg == 'query':
                result = method(query)
            elif route.arg == 'version':
                version = None
                for key in ('v', 'version'):
                    values = query.get(key)
                    if values:
                        version = values[0]
                        break
                result = method(version)
            elif route.arg == 'name':
                result = method(route.extra)
            else:
                result = method()
        except ApiError:
            raise
        except ValueError as exc:
            raise ApiError(ErrorCode.BAD_REQUEST, str(exc))
        except OSError as exc:
            raise ApiError(ErrorCode.INTERNAL, 'IO 错误：%s' % exc, status=500)
        if isinstance(result, tuple):
            if len(result) == 3:
                status, headers, payload = result
                return int(status), dict(headers or {}), payload
            if len(result) == 2:
                status, payload = result
                return int(status), {}, payload
        return 200, {}, result

    def log(self, level, message):
        try:
            self.controller._log(level, message)
        except Exception:  # pragma: no cover
            pass


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

class _ConsoleHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, api):
        self.api = api
        super().__init__(address, _Handler)


class _Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'raspbot-console/1.0.0'
    sys_version = ''

    # -- 入口 -----------------------------------------------------------
    def log_message(self, fmt, *args):  # 静默：日志统一走网关 log 事件
        pass

    def do_GET(self):
        self._dispatch('GET')

    def do_POST(self):
        self._dispatch('POST')

    def do_OPTIONS(self):
        self._dispatch('OPTIONS')

    def do_HEAD(self):
        self._dispatch('HEAD')

    def _dispatch(self, method):
        api = self.server.api
        try:
            split = urlsplit(self.path)
            path = unquote(split.path or '/')
            query = parse_qs(split.query, keep_blank_values=True)
            if method == 'OPTIONS':
                self._send_options()
                return
            api.authenticate(method, path, split, self.headers)
            route = api.route(method, path)
            if route.kind == 'sse':
                self._handle_sse(route)
                return
            body = self._read_json_body() if method == 'POST' else None
            status, headers, payload = api.execute(route, query, body)
            if route.kind == 'binary':
                self._send_bytes(status, headers, payload)
            else:
                self._send_json(status, payload, headers)
        except ApiError as exc:
            self._send_api_error(exc)
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True
        except OSError:
            self.close_connection = True
        except Exception as exc:  # pragma: no cover - 兜底
            api.log('error', 'HTTP 处理异常: %r' % exc)
            try:
                self._send_api_error(ApiError(
                    ErrorCode.INTERNAL, '网关内部错误：%s' % exc, status=500))
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.close_connection = True

    # -- 请求体 ---------------------------------------------------------
    def _read_json_body(self):
        length_header = self.headers.get('Content-Length')
        if length_header is None:
            return {}
        try:
            length = int(length_header)
        except (TypeError, ValueError):
            raise ApiError(ErrorCode.BAD_REQUEST, 'Content-Length 非法')
        if length > protocol.MAX_JSON_BODY:
            self.close_connection = True
            raise ApiError(ErrorCode.PAYLOAD_TOO_LARGE,
                           'JSON 请求体不得超过 64 KB')
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        if len(raw) != length:
            self.close_connection = True
            raise ApiError(ErrorCode.BAD_REQUEST, '请求体不完整')
        return protocol.require_object(protocol.json_loads(raw))

    # -- 响应 -----------------------------------------------------------
    def _base_headers(self):
        headers = _cors_headers()
        headers['Cache-Control'] = 'no-store'
        return headers

    def _send_json(self, status, payload, extra_headers=None):
        body = protocol.json_dumps(payload).encode('utf-8')
        self.send_response(int(status))
        headers = self._base_headers()
        headers['Content-Type'] = 'application/json; charset=utf-8'
        headers['Content-Length'] = str(len(body))
        if extra_headers:
            headers.update(extra_headers)
        for key, value in headers.items():
            self.send_header(key, value)
        self.end_headers()
        if body:
            self.wfile.write(body)
        self.wfile.flush()

    def _send_bytes(self, status, headers, body):
        body = bytes(body)
        self.send_response(int(status))
        merged = self._base_headers()
        merged.update(dict(headers or {}))
        merged.setdefault('Content-Type', 'application/octet-stream')
        merged['Content-Length'] = str(len(body))
        for key, value in merged.items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)
        self.wfile.flush()

    def _send_api_error(self, exc):
        headers = {}
        if exc.status == 405:
            headers['Allow'] = 'GET, POST, OPTIONS'
        self._send_json(exc.status, exc.payload(), headers)

    def _send_options(self):
        self.send_response(204)
        headers = _cors_headers()
        headers['Access-Control-Max-Age'] = '600'
        headers['Allow'] = 'GET, POST, OPTIONS'
        headers['Content-Length'] = '0'
        for key, value in headers.items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.flush()

    # -- SSE（HTTP/1.1 chunked，逐帧 flush） ---------------------------
    def _handle_sse(self, route):
        api = self.server.api
        controller = api.controller
        try:
            client_id, client_queue = controller.event_bus.subscribe()
        except Exception:
            raise ApiError(ErrorCode.TOO_MANY_SSE,
                           'SSE 客户端已达上限（%d）'
                           % getattr(api.config, 'max_sse', 4), status=503)
        try:
            self.send_response(200)
            self.send_header('Content-Type',
                             'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('X-Accel-Buffering', 'no')
            self.send_header('Transfer-Encoding', 'chunked')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.flush()
            # 首帧：retry: 2000（§3.2）
            self._write_chunk(protocol.sse_retry_frame())
            last_send = time.monotonic()
            while not api.stop_event.is_set():
                try:
                    item = client_queue.get(timeout=SSE_QUEUE_TIMEOUT_S)
                except queue.Empty:
                    if self._client_gone():
                        break
                    if time.monotonic() - last_send >= SSE_PING_INTERVAL_S:
                        self._write_chunk(b': ping\n\n')
                        last_send = time.monotonic()
                    continue
                if not item:
                    break
                event, payload = item
                if event == CLOSE_SENTINEL[0]:
                    break
                self._write_chunk(protocol.sse_frame(event, payload))
                last_send = time.monotonic()
            self._write_final_chunk()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            try:
                controller.event_bus.unsubscribe(client_id)
            except Exception:  # pragma: no cover
                pass
            self.close_connection = True

    def _client_gone(self):
        """非阻塞探测 SSE 客户端是否已断开（避免占满 max_sse 名额）。"""
        try:
            readable, _writable, _errors = select.select(
                [self.connection], [], [], 0)
        except (OSError, ValueError):
            return True
        if not readable:
            return False
        try:
            data = self.connection.recv(1, socket.MSG_PEEK)
        except (BlockingIOError, InterruptedError):
            return False
        except OSError:
            return True
        return not data

    def _write_chunk(self, data):
        if not data:
            return
        head = ('%X\r\n' % len(data)).encode('ascii')
        self.wfile.write(head)
        self.wfile.write(data)
        self.wfile.write(b'\r\n')
        self.wfile.flush()

    def _write_final_chunk(self):
        try:
            self.wfile.write(b'0\r\n\r\n')
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
