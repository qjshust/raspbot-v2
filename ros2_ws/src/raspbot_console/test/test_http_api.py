"""HTTP/SSE 集成测试：真实 ThreadingHTTPServer + 假 runtime/会话（离线）。

覆盖：health/state 契约、SSE chunked 分帧、teleop 校验与看门狗状态、
estop 锁存、session 409、map 404/PNG/保存/列表/预览、导航 initialpose→
goal→cancel 流程、token/CORS、405/404、请求体上限。
"""


import http.client
import json
import os
import socket
import sys
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
import support  # noqa: E402

from raspbot_console import http_api, mapio, protocol  # noqa: E402
from raspbot_console import session as session_mod  # noqa: E402
from raspbot_console.config import Config  # noqa: E402
from raspbot_console.controller import Controller  # noqa: E402
from raspbot_console.http_api import HttpApi  # noqa: E402
from raspbot_console.session import SessionManager  # noqa: E402
from raspbot_console.state import StateStore  # noqa: E402


# ---------------------------------------------------------------------------
# 假对象
# ---------------------------------------------------------------------------
class FakeClock:
    def __init__(self, value=1000.0):
        self.value = float(value)

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += float(seconds)


class FakeProc:
    def __init__(self, command, **kwargs):
        self.command = list(command)
        self.kwargs = kwargs
        self.pid = 4321
        self.returncode = None
        self.stdout = __import__('io').StringIO(
            '[bringup] line one\n[bringup] line two\n')

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is None:
            self.returncode = 0
        return self.returncode


class ProcFactory:
    def __init__(self):
        self.procs = []

    def __call__(self, command, **kwargs):
        proc = FakeProc(command, **kwargs)
        self.procs.append(proc)
        return proc


class FakeRuntime:
    """controller 期望的 runtime 鸭子接口。"""

    def __init__(self):
        self.calls = []
        self.graph_data = {'nodes': [], 'topics': [],
                           'publishers': {}, 'subscribers': {}}
        self.has_map_transform = False
        self.reject_goal = False

    # -- graph ---------------------------------------------------------
    def graph(self):
        return self.graph_data

    def node_names(self):
        return list(self.graph_data.get('nodes') or [])

    # -- 发布 ---------------------------------------------------------
    def publish_teleop(self, vx, vy, wz):
        self.calls.append(('publish_teleop', vx, vy, wz))

    def publish_zero_twist(self):
        self.calls.append(('publish_zero_twist',))

    def publish_estop(self, active):
        self.calls.append(('publish_estop', active))

    def publish_buzzer(self, on):
        self.calls.append(('publish_buzzer', on))

    def publish_led(self, r, g, b):
        self.calls.append(('publish_led', r, g, b))

    def publish_servo(self, pan, tilt):
        self.calls.append(('publish_servo', pan, tilt))

    def begin_initialpose(self, x, y, yaw):
        self.calls.append(('begin_initialpose', x, y, yaw))

    def begin_activation(self):
        self.calls.append(('begin_activation',))
        return True

    def cancel_activation(self):
        self.calls.append(('cancel_activation',))

    def cancel_nav_goal(self):
        self.calls.append(('cancel_nav_goal',))

    def has_map_tf(self):
        return self.has_map_transform

    def send_nav_goal(self, x, y, yaw, goal_id=None):
        self.calls.append(('send_nav_goal', x, y, yaw))
        if self.reject_goal:
            return {'accepted': False, 'goal_id': goal_id,
                    'reason': 'test reject'}
        return {'accepted': True, 'goal_id': goal_id, 'reason': None}


# ---------------------------------------------------------------------------
# SSE 读取工具
# ---------------------------------------------------------------------------
class ChunkReader:
    def __init__(self, sock, initial=b''):
        self.sock = sock
        self.buffer = bytearray(initial)

    def _fill(self):
        data = self.sock.recv(4096)
        if not data:
            raise EOFError('SSE 连接已关闭')
        self.buffer.extend(data)

    def read_exact(self, count):
        while len(self.buffer) < count:
            self._fill()
        data = bytes(self.buffer[:count])
        del self.buffer[:count]
        return data

    def read_line(self):
        while b'\r\n' not in self.buffer:
            self._fill()
        index = self.buffer.index(b'\r\n')
        data = bytes(self.buffer[:index])
        del self.buffer[:index + 2]
        return data

    def read_chunk(self):
        size = int(self.read_line().strip(), 16)
        if size == 0:
            return b''
        data = self.read_exact(size)
        self.read_exact(2)  # 尾随 CRLF
        return data


# ---------------------------------------------------------------------------
# 基类
# ---------------------------------------------------------------------------
class HttpApiTestBase(unittest.TestCase):
    TOKEN = ''

    def setUp(self):
        self.tmpdir = support.temp_dir()
        self.clock = FakeClock()
        self.factory = ProcFactory()
        self._original_signal = session_mod.signal_process_group
        session_mod.signal_process_group = lambda proc, sig: None
        self.config = Config(
            host='127.0.0.1', port=0, token=self.TOKEN,
            maps_dir=os.path.join(self.tmpdir, 'maps'),
            maps_fallback=os.path.join(self.tmpdir, 'fallback'),
            session_timeout_s=10.0,
        )
        self.state = StateStore(self.config.limits,
                                max_sse=self.config.max_sse,
                                clock=self.clock)
        self.runtime = FakeRuntime()
        self.session = SessionManager(
            clock=self.clock, wall_clock=self.clock,
            popen_factory=self.factory)
        self.controller = Controller(
            self.config, self.state, self.session, runtime=self.runtime,
            clock=self.clock, wall_clock=self.clock)
        self.http = HttpApi(self.controller, self.config)
        self.http.start()
        self.port = self.http.port
        self._sockets = []

    def tearDown(self):
        for sock in self._sockets:
            try:
                sock.close()
            except OSError:
                pass
        try:
            self.state.event_bus.close()
        except Exception:
            pass
        try:
            self.http.stop()
        except Exception:
            pass
        session_mod.signal_process_group = self._original_signal

    # -- HTTP 工具 ------------------------------------------------------
    def request(self, method, path, body=None, headers=None, raw_body=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port,
                                                timeout=10)
        send_headers = {'Content-Type': 'application/json'}
        if headers:
            send_headers.update(headers)
        if raw_body is not None:
            data = raw_body
        elif body is not None:
            data = json.dumps(body).encode('utf-8')
        else:
            data = None
        try:
            connection.request(method, path, body=data,
                               headers=send_headers)
            response = connection.getresponse()
            payload = response.read()
            response_headers = {key.lower(): value
                                for key, value in response.getheaders()}
            return response.status, response_headers, payload
        finally:
            connection.close()

    def get_json(self, path):
        status, headers, payload = self.request('GET', path)
        return status, headers, json.loads(payload.decode('utf-8'))

    def post_json(self, path, body=None):
        status, headers, payload = self.request('POST', path, body=body)
        return status, headers, json.loads(payload.decode('utf-8'))

    # -- 状态推进 -------------------------------------------------------
    def advance(self, seconds, step=0.2):
        end = self.clock.value + float(seconds)
        while self.clock.value < end:
            self.clock.advance(step)
            self.controller.tick_session()

    def _base_graph(self):
        return {
            'nodes': ['raspbot_base', 'ekf_node', 'twist_mux'],
            'topics': [{'name': '/odom',
                        'types': ['nav_msgs/msg/Odometry']}],
            'publishers': {'/odom': 1, '/cmd_vel_safe': 1},
            'subscribers': {'/cmd_vel_teleop': 1},
        }

    def _nav_graph(self):
        graph = self._base_graph()
        graph['nodes'] = graph['nodes'] + ['amcl', 'map_server']
        graph['publishers']['/map'] = 1
        return graph

    def inject_map(self):
        grid = mapio.make_map_grid(
            3, 2, 0.05, (-2.4, -2.0, 0.0), [0, -1, 100, 50, 100, 0],
            frame_id='map', stamp=1764012345.2)
        changed, meta = self.state.set_map(grid)
        self.assertTrue(changed)
        return meta

    def start_base_session(self):
        status, _headers, payload = self.post_json('/api/session/start',
                                         {'mode': 'base'})
        self.assertEqual(status, 202, payload)
        self.runtime.graph_data = self._base_graph()
        self.advance(1.5)
        self.assertEqual(self.session.snapshot()['state'], 'running')
        return payload

    def start_nav_session(self):
        map_yaml = os.path.join(self.tmpdir, 'nav_map.yaml')
        with open(map_yaml, 'w', encoding='utf-8') as fh:
            fh.write(mapio.format_map_yaml('nav_map.pgm', 0.05,
                                           (0.0, 0.0, 0.0)))
        status, _headers, payload = self.post_json(
            '/api/session/start', {'mode': 'nav', 'map': map_yaml})
        self.assertEqual(status, 202, payload)
        self.runtime.graph_data = self._nav_graph()
        self.inject_map()
        self.advance(1.5)
        self.assertEqual(self.session.snapshot()['state'], 'running')
        return payload

    # -- SSE 工具 -------------------------------------------------------
    def open_sse(self, query=''):
        sock = socket.create_connection(('127.0.0.1', self.port), timeout=10)
        sock.settimeout(10)
        path = '/api/events' + query
        sock.sendall(('GET %s HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n'
                      % path).encode('utf-8'))
        self._sockets.append(sock)
        buffer = b''
        while b'\r\n\r\n' not in buffer:
            data = sock.recv(4096)
            if not data:
                raise EOFError('SSE 响应头截断')
            buffer += data
        head, rest = buffer.split(b'\r\n\r\n', 1)
        return sock, head.decode('latin-1'), rest


# ---------------------------------------------------------------------------
# 基础契约测试
# ---------------------------------------------------------------------------
class BasicApiTests(HttpApiTestBase):

    def test_health(self):
        status, headers, payload = self.request('GET', '/api/health')
        self.assertEqual(status, 200)
        data = json.loads(payload.decode('utf-8'))
        self.assertTrue(data['ok'])
        self.assertEqual(data['name'], 'raspbot-console')
        self.assertEqual(data['api'], 1)
        self.assertEqual(data['version'], '1.0.0')
        self.assertIsInstance(data['server_time'], float)
        self.assertIsInstance(data['uptime_s'], float)
        self.assertEqual(headers['content-type'],
                         'application/json; charset=utf-8')
        self.assertEqual(headers['access-control-allow-origin'], '*')

    def test_state_schema(self):
        status, headers, data = self.get_json('/api/state')
        self.assertEqual(status, 200)
        for key in ('api', 'server_time', 'uptime_s', 'session', 'ros',
                    'limits', 'robot', 'nav', 'map', 'estop', 'teleop',
                    'peripherals', 'diagnostics', 'logs'):
            self.assertIn(key, data)
        self.assertEqual(data['api'], 1)
        self.assertEqual(data['session']['state'], 'idle')
        self.assertEqual(data['session']['available_modes'],
                         ['base', 'slam', 'nav'])
        self.assertEqual(data['nav']['activation'],
                         {'done': 0, 'total': 6, 'phase': ''})
        self.assertFalse(data['map']['available'])
        self.assertEqual(data['diagnostics']['level'], 'unknown')
        self.assertIsInstance(data['server_time'], float)

    def test_cors_preflight(self):
        status, headers, _payload = self.request('OPTIONS', '/api/state')
        self.assertEqual(status, 204)
        self.assertEqual(headers['access-control-allow-origin'], '*')
        self.assertEqual(headers['access-control-allow-methods'],
                         'GET, POST, OPTIONS')
        self.assertIn('x-console-token',
                      headers['access-control-allow-headers'].lower())

    def test_unknown_path_and_wrong_method(self):
        status, headers, payload = self.request('GET', '/api/nope')
        self.assertEqual(status, 404)
        self.assertEqual(json.loads(payload)['error']['code'], 'not_found')
        status, headers, payload = self.request('POST', '/api/state')
        self.assertEqual(status, 405)
        self.assertIn('allow', headers)
        status, _, payload = self.request('GET', '/api/session/start')
        self.assertEqual(status, 405)

    def test_invalid_json_body(self):
        status, _, payload = self.request('POST', '/api/estop',
                                          raw_body=b'{bad json')
        self.assertEqual(status, 400)
        self.assertEqual(json.loads(payload)['error']['code'], 'bad_request')

    def test_payload_too_large(self):
        sock = socket.create_connection(('127.0.0.1', self.port), timeout=10)
        try:
            sock.sendall(b'POST /api/estop HTTP/1.1\r\nHost: 127.0.0.1\r\n'
                         b'Content-Type: application/json\r\n'
                         b'Content-Length: 70000\r\n\r\n')
            data = b''
            while b'\r\n\r\n' not in data:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                data += chunk
            head, rest = data.split(b'\r\n\r\n', 1)
            self.assertIn(b'400', head.split(b'\r\n')[0])
            length = 0
            for line in head.split(b'\r\n'):
                if line.lower().startswith(b'content-length:'):
                    length = int(line.split(b':')[1].strip())
            body = rest
            while len(body) < length:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                body += chunk
            self.assertIn(b'payload_too_large', body)
        finally:
            sock.close()

    def test_logs_and_ros(self):
        status, _headers, data = self.get_json('/api/logs?limit=10')
        self.assertEqual(status, 200)
        self.assertIn('logs', data)
        status, _headers, data = self.get_json('/api/logs?limit=0')
        self.assertEqual(status, 400)
        self.runtime.graph_data = {
            'nodes': ['b', 'a'],
            'topics': [{'name': '/odom', 'types': ['nav_msgs/msg/Odometry']}],
            'publishers': {}, 'subscribers': {},
        }
        status, _headers, data = self.get_json('/api/ros')
        self.assertEqual(status, 200)
        self.assertEqual(data['nodes'], ['a', 'b'])
        self.assertEqual(data['topics'][0]['name'], '/odom')

    def test_session_logs_after_start(self):
        self.post_json('/api/session/start', {'mode': 'base'})
        status, _headers, data = self.get_json('/api/session/logs?limit=50')
        self.assertEqual(status, 200)
        self.assertTrue(data['logs'])
        self.assertTrue(data['logs'][0]['line'].startswith('$ bash -lc'))


# ---------------------------------------------------------------------------
# 会话 / 遥控 / 急停
# ---------------------------------------------------------------------------
class SessionTeleopEstopTests(HttpApiTestBase):

    def test_start_invalid_payloads(self):
        status, _, data = self.post_json('/api/session/start', {'mode': 'x'})
        self.assertEqual(status, 400)
        status, _, data = self.post_json('/api/session/start', {'mode': 'nav'})
        self.assertEqual(status, 400)
        self.assertEqual(data['error']['code'], 'bad_request')
        status, _, data = self.post_json('/api/session/start',
                                         {'mode': 'nav',
                                          'map': '/no/such/map.yaml'})
        self.assertEqual(status, 400)

    def test_start_busy_when_starting(self):
        self.post_json('/api/session/start', {'mode': 'base'})
        status, _, data = self.post_json('/api/session/start',
                                         {'mode': 'slam'})
        self.assertEqual(status, 409)
        self.assertEqual(data['error']['code'], 'busy')

    def test_state_reports_session_after_start(self):
        payload = self.start_base_session()
        self.assertEqual(payload['session']['mode'], 'base')
        status, _headers, data = self.get_json('/api/state')
        self.assertEqual(data['session']['mode'], 'base')
        self.assertEqual(data['session']['state'], 'running')
        self.assertAlmostEqual(data['session']['progress'], 1.0)

    def test_teleop_validation_and_estop(self):
        status, _, data = self.post_json('/api/teleop',
                                         {'vx': 0.1, 'vy': 0, 'wz': 0})
        self.assertEqual(status, 409)
        self.assertEqual(data['error']['code'], 'session_not_running')

        self.start_base_session()
        status, _, data = self.post_json('/api/teleop', {'vx': 'x', 'vy': 0,
                                                         'wz': 0})
        self.assertEqual(status, 400)
        status, _, data = self.post_json('/api/teleop', {'vx': 0.1})
        self.assertEqual(status, 400)

        status, _, data = self.post_json('/api/teleop',
                                         {'vx': 9.0, 'vy': -9.0,
                                          'wz': 9.0, 'seq': 1})
        self.assertEqual(status, 200)
        self.assertTrue(data['ok'])
        teleop = self.state.teleop_snapshot()
        self.assertTrue(teleop['active'])
        self.assertAlmostEqual(teleop['vx'], 0.30)
        self.assertAlmostEqual(teleop['vy'], -0.30)
        self.assertAlmostEqual(teleop['wz'], 1.5)
        self.assertIn(('publish_teleop', 0.30, -0.30, 1.5),
                      self.runtime.calls)
        # 乱序 seq 被丢弃但返回 200
        status, _, _ = self.post_json('/api/teleop',
                                      {'vx': 0.01, 'vy': 0, 'wz': 0,
                                       'seq': 0})
        self.assertEqual(status, 200)
        self.assertAlmostEqual(self.state.teleop_snapshot()['vx'], 0.30)
        # 停遥控
        status, _, _ = self.post_json('/api/teleop/stop')
        self.assertEqual(status, 200)
        self.assertFalse(self.state.teleop_snapshot()['active'])
        self.assertIn(('publish_zero_twist',), self.runtime.calls)

    def test_estop_latches_and_blocks(self):
        self.start_base_session()
        status, _, data = self.post_json('/api/estop', {'active': True})
        self.assertEqual(status, 200)
        self.assertTrue(data['estop']['active'])
        self.assertIsInstance(data['estop']['since'], float)
        self.assertIn(('publish_estop', True), self.runtime.calls)
        self.assertTrue(self.state.estop_snapshot()['active'])
        for path, body in (('/api/teleop', {'vx': 0, 'vy': 0, 'wz': 0}),
                           ('/api/session/start', {'mode': 'slam'})):
            status, _, data = self.post_json(path, body)
            self.assertEqual(status, 409, path)
            self.assertEqual(data['error']['code'], 'estop_active')
        status, _, data = self.post_json('/api/estop', {'active': False})
        self.assertEqual(status, 200)
        self.assertFalse(data['estop']['active'])
        self.assertIn(('publish_estop', False), self.runtime.calls)
        status, _, _ = self.post_json('/api/teleop',
                                      {'vx': 0.1, 'vy': 0, 'wz': 0})
        self.assertEqual(status, 200)

    def test_estop_requires_bool(self):
        status, _, _ = self.post_json('/api/estop', {})
        self.assertEqual(status, 400)
        status, _, _ = self.post_json('/api/estop', {'active': 1})
        self.assertEqual(status, 400)

    def test_session_stop_flow(self):
        self.start_base_session()
        proc = self.factory.procs[0]
        status, _, data = self.post_json('/api/session/stop')
        self.assertEqual(status, 202)
        self.assertEqual(data['session']['state'], 'stopping')
        proc.returncode = 0
        self.advance(1.0)
        # 停止路径必须停遥控并下发零速（§5.2）
        self.assertIn(('publish_zero_twist',), self.runtime.calls)
        self.assertFalse(self.state.teleop_snapshot()['active'])
        status, _headers, data = self.get_json('/api/state')
        self.assertEqual(data['session']['state'], 'idle')
        status, _, data = self.post_json('/api/session/stop')
        self.assertEqual(status, 200)
        self.assertTrue(data['ok'])

    def test_actuators_require_running(self):
        status, _, data = self.post_json('/api/actuator/buzzer', {'on': True})
        self.assertEqual(status, 409)
        self.start_base_session()
        status, _, _ = self.post_json('/api/actuator/buzzer', {'on': True})
        self.assertEqual(status, 200)
        self.assertTrue(self.state.peripherals_snapshot()['buzzer'])
        status, _, _ = self.post_json('/api/actuator/led',
                                      {'r': 0.5, 'g': 2.0, 'b': -1.0})
        self.assertEqual(status, 200)
        led = self.state.peripherals_snapshot()['led']
        self.assertAlmostEqual(led['g'], 1.0)
        self.assertAlmostEqual(led['b'], 0.0)
        status, _, _ = self.post_json('/api/actuator/servo',
                                      {'pan': 200.0, 'tilt': 200.0})
        self.assertEqual(status, 200)
        servo = self.state.peripherals_snapshot()['servo']
        self.assertAlmostEqual(servo['pan'], 180.0)
        self.assertAlmostEqual(servo['tilt'], 110.0)
        self.assertIn(('publish_servo', 180.0, 110.0), self.runtime.calls)


# ---------------------------------------------------------------------------
# 地图接口
# ---------------------------------------------------------------------------
class MapApiTests(HttpApiTestBase):

    def test_map_meta_and_png_when_missing(self):
        status, _headers, data = self.get_json('/api/map/meta')
        self.assertEqual(status, 200)
        self.assertFalse(data['available'])
        status, _headers, payload = self.request('GET', '/api/map.png')
        self.assertEqual(status, 404)
        self.assertEqual(json.loads(payload)['error']['code'], 'no_map')

    def test_map_png_and_meta_after_map(self):
        meta = self.inject_map()
        status, _headers, data = self.get_json('/api/map/meta')
        self.assertEqual(status, 200)
        self.assertTrue(data['available'])
        self.assertEqual(data['version'], meta['version'])
        self.assertEqual(data['width'], 3)
        self.assertEqual(data['height'], 2)

        status, headers, payload = self.request(
            'GET', '/api/map.png?v=%d' % meta['version'])
        self.assertEqual(status, 200)
        self.assertEqual(headers['content-type'], 'image/png')
        self.assertIn('immutable', headers['cache-control'])
        width, height, rows = mapio.decode_png_gray8(payload)
        self.assertEqual((width, height), (3, 2))
        # 行 0 = 世界最大 y：[50, 100, 0] → [127, 0, 254]
        self.assertEqual(rows[0], bytes([127, 0, 254]))
        self.assertEqual(rows[1], bytes([254, 205, 0]))

        # 旧版本号仍返回当前地图，但不做长缓存
        status, headers, payload = self.request('GET', '/api/map.png?v=999')
        self.assertEqual(status, 200)
        self.assertEqual(headers['cache-control'], 'no-cache')

    def test_map_save_flow(self):
        # 会话未运行 → 409
        status, _, data = self.post_json('/api/map/save', {'name': 'room'})
        self.assertEqual(status, 409)
        self.assertEqual(data['error']['code'], 'session_idle')

        self.start_base_session()
        # 无地图 → 409
        status, _, data = self.post_json('/api/map/save', {'name': 'room'})
        self.assertEqual(status, 409)
        self.assertEqual(data['error']['code'], 'no_map')

        self.inject_map()
        status, _, data = self.post_json('/api/map/save', {'name': 'room'})
        self.assertEqual(status, 200, data)
        self.assertTrue(os.path.isfile(data['yaml']))
        self.assertTrue(os.path.isfile(data['pgm']))
        self.assertEqual(data['width'], 3)
        self.assertEqual(data['height'], 2)
        self.assertAlmostEqual(data['resolution'], 0.05)
        self.assertEqual(data['name'], 'room')

        # 同名冲突 → 409 map_exists；overwrite=true 覆盖
        status, _, data = self.post_json('/api/map/save', {'name': 'room'})
        self.assertEqual(status, 409)
        self.assertEqual(data['error']['code'], 'map_exists')
        status, _, data = self.post_json('/api/map/save',
                                         {'name': 'room',
                                          'overwrite': True})
        self.assertEqual(status, 200)

        # 非法名称 → 400
        status, _, _ = self.post_json('/api/map/save', {'name': '../x'})
        self.assertEqual(status, 400)

        # 列表 + 预览
        status, _headers, data = self.get_json('/api/maps')
        self.assertEqual(status, 200)
        names = [entry['name'] for entry in data['maps']]
        self.assertIn('room', names)
        entry = [item for item in data['maps'] if item['name'] == 'room'][0]
        self.assertTrue(entry['valid'])
        self.assertEqual(entry['width'], 3)
        self.assertEqual(entry['preview_url'], '/api/maps/room/preview.png')
        self.assertEqual(data['dirs'], [os.path.dirname(entry['path'])])

        status, headers, payload = self.request(
            'GET', '/api/maps/room/preview.png')
        self.assertEqual(status, 200)
        self.assertEqual(headers['content-type'], 'image/png')
        width, height, _rows = mapio.decode_png_gray8(payload)
        self.assertEqual((width, height), (3, 2))

    def test_preview_404(self):
        status, _headers, payload = self.request(
            'GET', '/api/maps/nope/preview.png')
        self.assertEqual(status, 404)
        self.assertEqual(json.loads(payload)['error']['code'], 'not_found')


# ---------------------------------------------------------------------------
# 导航流程
# ---------------------------------------------------------------------------
class NavigationApiTests(HttpApiTestBase):

    def test_initialpose_and_goal_flow(self):
        # 非导航模式 → 409
        self.start_base_session()
        status, _, data = self.post_json('/api/goal',
                                         {'x': 1.0, 'y': 0.0, 'yaw': None})
        self.assertEqual(status, 409)
        self.assertEqual(data['error']['code'], 'nav_not_ready')
        self.post_json('/api/session/stop')
        self.factory.procs[0].returncode = 0
        self.advance(1.0)

        # 导航模式：初始位姿前 nav.state=waiting_initialpose
        self.start_nav_session()
        status, _headers, data = self.get_json('/api/state')
        self.assertEqual(data['nav']['state'], 'waiting_initialpose')
        status, _, data = self.post_json('/api/goal',
                                         {'x': 1.0, 'y': 0.0, 'yaw': 0.0})
        self.assertEqual(status, 409)
        self.assertEqual(data['error']['code'], 'nav_not_ready')

        # 初始位姿
        status, _, data = self.post_json('/api/initialpose',
                                         {'x': 0.5, 'y': 0.2, 'yaw': 0.1})
        self.assertEqual(status, 200)
        self.assertEqual(data['nav']['state'], 'activating')
        self.assertIn(('begin_initialpose', 0.5, 0.2, 0.1),
                      self.runtime.calls)
        status, _headers, data = self.get_json('/api/state')
        self.assertEqual(data['nav']['state'], 'activating')
        self.assertAlmostEqual(data['nav']['initialpose']['x'], 0.5)

        # 激活完成后 ready
        self.controller.on_activation_progress(6, 6, '完成')
        self.controller.on_activation_done(True, '')
        status, _headers, data = self.get_json('/api/state')
        self.assertEqual(data['nav']['state'], 'ready')

        # 发送目标
        status, _, data = self.post_json('/api/goal',
                                         {'x': 1.5, 'y': 0.0, 'yaw': None})
        self.assertEqual(status, 202, data)
        self.assertEqual(data['goal']['x'], 1.5)
        self.assertTrue(data['goal_id'])
        self.assertAlmostEqual(data['goal']['yaw'], 0.0)
        status, _headers, data = self.get_json('/api/state')
        self.assertEqual(data['nav']['state'], 'goal_active')
        self.assertEqual(data['nav']['goal_state'], 'accepted')

        # 已有活动目标 → 409 goal_active
        status, _, data = self.post_json('/api/goal',
                                         {'x': 2.0, 'y': 0.0, 'yaw': 0.0})
        self.assertEqual(status, 409)
        self.assertEqual(data['error']['code'], 'goal_active')

        # 取消
        status, _, _ = self.post_json('/api/goal/cancel')
        self.assertEqual(status, 200)
        self.assertIn(('cancel_nav_goal',), self.runtime.calls)
        status, _headers, data = self.get_json('/api/state')
        self.assertEqual(data['nav']['state'], 'canceled')

        # 终态 3 s 后自动回到 ready
        self.advance(3.5)
        status, _headers, data = self.get_json('/api/state')
        self.assertEqual(data['nav']['state'], 'ready')

    def test_goal_rejected_returns_503(self):
        self.start_nav_session()
        self.post_json('/api/initialpose', {'x': 0, 'y': 0, 'yaw': 0})
        self.controller.on_activation_done(True, '')
        self.runtime.reject_goal = True
        status, _, data = self.post_json('/api/goal',
                                         {'x': 1.0, 'y': 0.0, 'yaw': 0.0})
        self.assertEqual(status, 503)
        self.assertEqual(data['error']['code'], 'nav_rejected')

    def test_nav_activate_retry(self):
        self.start_nav_session()
        status, _, data = self.post_json('/api/nav/activate')
        self.assertEqual(status, 202, data)
        self.assertIn(('begin_activation',), self.runtime.calls)
        # 已激活完成后再点重试 → 允许再次编排（202）；进行中则 409
        self.controller.on_activation_done(True, '')
        status, _, _ = self.post_json('/api/nav/activate')
        self.assertEqual(status, 202)

    def test_initialpose_requires_nav_mode(self):
        status, _, _ = self.post_json('/api/initialpose',
                                      {'x': 0, 'y': 0, 'yaw': 0})
        self.assertEqual(status, 409)
        self.start_base_session()
        status, _, data = self.post_json('/api/initialpose',
                                         {'x': 0, 'y': 0, 'yaw': 0})
        self.assertEqual(status, 409)
        self.assertEqual(data['error']['code'], 'nav_not_ready')
        status, _, _ = self.post_json('/api/initialpose', {'x': 0})
        self.assertEqual(status, 400)


# ---------------------------------------------------------------------------
# SSE
# ---------------------------------------------------------------------------
class SseTests(HttpApiTestBase):

    def test_chunked_headers_and_state_event(self):
        sock, head, rest = self.open_sse()
        lower = head.lower()
        self.assertIn('http/1.1 200', lower)
        self.assertIn('text/event-stream', lower)
        self.assertIn('transfer-encoding: chunked', lower)
        self.assertIn('x-accel-buffering: no', lower)
        self.assertIn('cache-control: no-cache', lower)
        self.assertNotIn('content-length', lower)

        reader = ChunkReader(sock, rest)
        first = reader.read_chunk()
        self.assertEqual(first, b'retry: 2000\n\n')

        self.controller.publish_state_snapshot()
        frame = b''
        for _attempt in range(20):
            chunk = reader.read_chunk()
            if not chunk:
                break
            frame += chunk
            if b'event: state' in frame and frame.endswith(b'\n\n'):
                break
        self.assertIn(b'event: state\n', frame)
        data_line = [line for line in frame.decode('utf-8').split('\n')
                     if line.startswith('data: ')][0]
        payload = json.loads(data_line[len('data: '):])
        self.assertEqual(payload['state']['api'], 1)
        self.assertIn('session', payload['state'])

    def test_sse_with_http_client(self):
        """用 http.client 增量读取 SSE 帧（PC 代理/EventSource 的读取方式）。"""
        connection = http.client.HTTPConnection('127.0.0.1', self.port,
                                                timeout=10)
        try:
            connection.request('GET', '/api/events')
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertIn('text/event-stream',
                          response.getheader('Content-Type') or '')
            self.assertEqual(response.getheader('X-Accel-Buffering'), 'no')
            self.assertEqual(response.getheader('Transfer-Encoding'),
                             'chunked')
            # 首帧 retry: 2000（两行）
            self.assertEqual(response.readline(), b'retry: 2000\n')
            self.assertEqual(response.readline(), b'\n')
            # 触发 state 事件后读取一帧
            self.controller.publish_state_snapshot()
            lines = []
            for _attempt in range(20):
                line = response.readline()
                if not line:
                    break
                lines.append(line)
                if len(lines) >= 3 and lines[-1] == b'\n' \
                        and any(item.startswith(b'data: ') for item in lines):
                    break
            self.assertTrue(any(item == b'event: state\n' for item in lines),
                            lines)
            data_line = [item for item in lines
                         if item.startswith(b'data: ')][0]
            payload = json.loads(data_line[len(b'data: '):].decode('utf-8'))
            self.assertEqual(payload['state']['api'], 1)
        finally:
            connection.close()

    def test_ping_when_idle(self):
        original_ping = http_api.SSE_PING_INTERVAL_S
        original_timeout = http_api.SSE_QUEUE_TIMEOUT_S
        http_api.SSE_PING_INTERVAL_S = 0.1
        http_api.SSE_QUEUE_TIMEOUT_S = 0.02
        try:
            sock, _head, rest = self.open_sse()
            reader = ChunkReader(sock, rest)
            self.assertEqual(reader.read_chunk(), b'retry: 2000\n\n')
            frame = b''
            for _attempt in range(50):
                frame += reader.read_chunk()
                if b': ping' in frame:
                    break
            self.assertIn(b': ping\n\n', frame)
        finally:
            http_api.SSE_PING_INTERVAL_S = original_ping
            http_api.SSE_QUEUE_TIMEOUT_S = original_timeout

    def test_max_four_clients(self):
        readers = []
        for _index in range(4):
            sock, _head, rest = self.open_sse()
            reader = ChunkReader(sock, rest)
            self.assertEqual(reader.read_chunk(), b'retry: 2000\n\n')
            readers.append(reader)
        status, _headers, payload = self.request('GET', '/api/events')
        self.assertEqual(status, 503)
        self.assertEqual(json.loads(payload)['error']['code'], 'too_many_sse')
        for reader in readers:
            reader.sock.close()
        # 服务端在下一轮 1 s 队列超时后检测断连并释放名额
        deadline = time.time() + 5.0
        while time.time() < deadline and self.state.event_bus.client_count > 0:
            time.sleep(0.05)
        self.assertEqual(self.state.event_bus.client_count, 0)
        sock, _head, rest = self.open_sse()
        ChunkReader(sock, rest).read_chunk()
        sock.close()


# ---------------------------------------------------------------------------
# token
# ---------------------------------------------------------------------------
class TokenTests(HttpApiTestBase):
    TOKEN = 'secret-token'

    def test_health_without_token(self):
        status, _headers, _payload = self.request('GET', '/api/health')
        self.assertEqual(status, 200)

    def test_state_requires_token(self):
        status, _headers, payload = self.request('GET', '/api/state')
        self.assertEqual(status, 401)
        self.assertEqual(json.loads(payload)['error']['code'], 'bad_token')
        status, _headers, _payload = self.request(
            'GET', '/api/state', headers={'X-Console-Token': 'secret-token'})
        self.assertEqual(status, 200)
        status, _headers, _payload = self.request(
            'GET', '/api/state?token=secret-token')
        self.assertEqual(status, 200)

    def test_post_requires_header_token(self):
        status, _headers, _payload = self.request(
            'POST', '/api/teleop', body={'vx': 0, 'vy': 0, 'wz': 0})
        self.assertEqual(status, 401)
        status, _headers, _payload = self.request(
            'POST', '/api/teleop', body={'vx': 0, 'vy': 0, 'wz': 0},
            headers={'X-Console-Token': 'secret-token'})
        # 会话未运行 → 409（说明已通过鉴权）
        self.assertEqual(status, 409)

    def test_sse_with_query_token(self):
        status, _headers, payload = self.request('GET', '/api/events')
        self.assertEqual(status, 401)
        sock, head, rest = self.open_sse('?token=secret-token')
        lower = head.lower()
        self.assertIn('text/event-stream', lower)
        reader = ChunkReader(sock, rest)
        self.assertEqual(reader.read_chunk(), b'retry: 2000\n\n')
        sock.close()


if __name__ == '__main__':
    unittest.main()
