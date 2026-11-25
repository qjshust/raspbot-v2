"""protocol 纯逻辑单元测试（离线，不依赖 rclpy）。"""


import json
import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
import support  # noqa: E402

from raspbot_console import protocol  # noqa: E402


class JsonTests(unittest.TestCase):

    def test_compact_separators_and_utf8(self):
        text = protocol.json_dumps({'msg': '地图已保存', 'n': 1, 'ok': True})
        self.assertNotIn(' ', text)
        self.assertIn('地图已保存', text)
        self.assertEqual(json.loads(text)['msg'], '地图已保存')
        self.assertEqual(text, '{"msg":"地图已保存","n":1,"ok":true}')

    def test_sanitize_replaces_non_finite(self):
        data = {'a': float('nan'), 'b': [float('inf'), 1.5], 'c': (1, 2)}
        clean = protocol.sanitize(data)
        self.assertIsNone(clean['a'])
        self.assertEqual(clean['b'], [None, 1.5])
        self.assertEqual(clean['c'], [1, 2])
        protocol.json_dumps(data)  # 不应抛

    def test_json_loads_empty_and_bad(self):
        self.assertEqual(protocol.json_loads(b''), {})
        self.assertEqual(protocol.json_loads('{}'), {})
        with self.assertRaises(protocol.ApiError) as ctx:
            protocol.json_loads(b'not-json')
        self.assertEqual(ctx.exception.code, protocol.ErrorCode.BAD_REQUEST)


class ApiErrorTests(unittest.TestCase):

    def test_payload_shape(self):
        err = protocol.ApiError(protocol.ErrorCode.MAP_EXISTS, '已存在',
                                detail={'name': 'x'})
        payload = err.payload()
        self.assertEqual(payload['error']['code'], 'map_exists')
        self.assertEqual(payload['error']['message'], '已存在')
        self.assertEqual(payload['error']['detail'], {'name': 'x'})
        self.assertEqual(err.status, 409)

    def test_default_status_mapping(self):
        cases = {
            protocol.ErrorCode.BAD_REQUEST: 400,
            protocol.ErrorCode.BAD_TOKEN: 401,
            protocol.ErrorCode.NOT_FOUND: 404,
            protocol.ErrorCode.METHOD_NOT_ALLOWED: 405,
            protocol.ErrorCode.BUSY: 409,
            protocol.ErrorCode.NAV_REJECTED: 503,
        }
        for code, status in cases.items():
            self.assertEqual(protocol.ApiError(code, 'x').status, status)

    def test_error_payload_helper(self):
        payload = protocol.error_payload('x', 'y', {'f': 1})
        self.assertEqual(payload['error']['detail'], {'f': 1})


class ValidationTests(unittest.TestCase):

    def test_require_number(self):
        body = {'x': 1.5, 'y': -2, 'bad': 'a', 'nan': float('nan'),
                'bool': True}
        self.assertAlmostEqual(protocol.require_number(body, 'x'), 1.5)
        self.assertAlmostEqual(protocol.require_number(body, 'y'), -2.0)
        self.assertEqual(protocol.require_number(body, 'missing', required=False,
                                                 default=3.0), 3.0)
        for field in ('bad', 'nan', 'bool'):
            with self.assertRaises(protocol.ApiError) as ctx:
                protocol.require_number(body, field)
            self.assertEqual(ctx.exception.code, 'bad_request')
        with self.assertRaises(protocol.ApiError):
            protocol.require_number(body, 'missing')

    def test_require_number_clamps(self):
        self.assertEqual(protocol.require_number({'v': 5.0}, 'v', low=0.0,
                                                 high=1.0), 1.0)
        self.assertEqual(protocol.require_number({'v': -5.0}, 'v', low=0.0,
                                                 high=1.0), 0.0)

    def test_require_bool(self):
        self.assertIs(protocol.require_bool({'on': True}, 'on'), True)
        self.assertIs(protocol.require_bool({'on': False}, 'on'), False)
        self.assertIs(protocol.require_bool({}, 'on', required=False,
                                          default=False), False)
        with self.assertRaises(protocol.ApiError):
            protocol.require_bool({'on': 1}, 'on')
        with self.assertRaises(protocol.ApiError):
            protocol.require_bool({}, 'on')

    def test_optional_int(self):
        self.assertEqual(protocol.optional_int({'seq': 3}, 'seq'), 3)
        self.assertEqual(protocol.optional_int({'seq': 3.0}, 'seq'), 3)
        self.assertIsNone(protocol.optional_int({}, 'seq'))
        for value in (True, 2.5, 'x', float('inf')):
            with self.assertRaises(protocol.ApiError):
                protocol.optional_int({'seq': value}, 'seq')

    def test_query_int(self):
        self.assertEqual(protocol.query_int({'limit': ['200']}, 'limit', 10,
                                            1, 300), 200)
        self.assertEqual(protocol.query_int({}, 'limit', 10), 10)
        with self.assertRaises(protocol.ApiError):
            protocol.query_int({'limit': ['x']}, 'limit', 10)
        with self.assertRaises(protocol.ApiError):
            protocol.query_int({'limit': ['0']}, 'limit', 10, 1, 300)
        with self.assertRaises(protocol.ApiError):
            protocol.query_int({'limit': ['999']}, 'limit', 10, 1, 300)

    def test_require_mode(self):
        for mode in ('base', 'slam', 'nav'):
            self.assertEqual(protocol.require_mode({'mode': mode}), mode)
        for bad in (None, '', 'other', 1):
            with self.assertRaises(protocol.ApiError):
                protocol.require_mode({'mode': bad})

    def test_validate_map_name(self):
        for name in ('room', 'room_20261124_2310', 'a.b-c_1', 'A' * 40):
            self.assertEqual(protocol.validate_map_name(name), name)
        for bad in ('', 'a/b', 'a b', '中文', 'a' * 41, '.', '..', None):
            with self.assertRaises(protocol.ApiError):
                protocol.validate_map_name(bad)

    def test_validate_map_path(self):
        tmp = support.temp_dir()
        yaml_path = os.path.join(tmp, 'room.yaml')
        with open(yaml_path, 'w', encoding='utf-8') as fh:
            fh.write('image: room.pgm\n')
        self.assertEqual(protocol.validate_map_path(yaml_path), yaml_path)
        for bad in ('relative/room.yaml', os.path.join(tmp, 'nope.yaml'),
                    os.path.join(tmp, 'room.txt'),
                    os.path.join(tmp, 'room.yaml.bak'), None):
            with self.assertRaises(protocol.ApiError):
                protocol.validate_map_path(bad)

    def test_clamp_velocity(self):
        limits = {'max_linear': 0.3, 'max_lateral': 0.3, 'max_angular': 1.5}
        self.assertEqual(protocol.clamp_velocity(1.0, -1.0, 9.0, limits),
                         (0.3, -0.3, 1.5))


class MathTests(unittest.TestCase):

    def test_yaw_quaternion_roundtrip(self):
        for yaw in (0.0, math.pi / 2, -math.pi / 2, 1.234, math.pi):
            q = protocol.yaw_to_quaternion(yaw)
            back = protocol.quaternion_to_yaw(*q)
            self.assertAlmostEqual(protocol.wrap_angle(back - yaw), 0.0,
                                   places=9)

    def test_wrap_angle(self):
        self.assertAlmostEqual(protocol.wrap_angle(3 * math.pi), math.pi)
        self.assertAlmostEqual(protocol.wrap_angle(-3 * math.pi), math.pi)
        self.assertAlmostEqual(protocol.wrap_angle(0.5), 0.5)

    def test_downsample_sequence(self):
        values = list(range(1000))
        out = protocol.downsample_sequence(values, 200)
        self.assertEqual(len(out), 200)
        self.assertEqual(out[-1], 999)
        self.assertEqual(protocol.downsample_sequence([1, 2], 200), [1, 2])
        self.assertEqual(protocol.downsample_sequence([], 10), [])


class SseFrameTests(unittest.TestCase):

    def test_retry_frame(self):
        self.assertEqual(protocol.sse_retry_frame(2000), b'retry: 2000\n\n')

    def test_event_frame_single_line_json(self):
        frame = protocol.sse_frame('state', {'state': {'api': 1, 'x': [1, 2]}})
        self.assertTrue(frame.startswith(b'event: state\ndata: '))
        self.assertTrue(frame.endswith(b'\n\n'))
        lines = frame.decode('utf-8').split('\n')
        self.assertEqual(lines[0], 'event: state')
        self.assertTrue(lines[1].startswith('data: '))
        payload = json.loads(lines[1][len('data: '):])
        self.assertEqual(payload['state']['api'], 1)
        self.assertNotIn('\n', lines[1])


if __name__ == '__main__':
    unittest.main()
