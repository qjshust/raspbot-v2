"""state 纯逻辑单元测试（离线，不依赖 rclpy）。"""


import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from raspbot_console import mapio, state  # noqa: E402


def _grid(width=2, height=2, data=(0, -1, 100, 50), stamp=1.0):
    return mapio.make_map_grid(width, height, 0.05, (0.0, 0.0, 0.0), list(data),
                               frame_id='map', stamp=stamp)


class RateMeterTests(unittest.TestCase):

    def test_window_rate(self):
        clock = {'now': 0.0}
        meter = state.RateMeter(2.0, clock=lambda: clock['now'])
        for index in range(21):
            clock['now'] = index * 0.1
            meter.tick()
        self.assertAlmostEqual(meter.rate(), 10.0, delta=0.2)
        clock['now'] = 10.0
        self.assertEqual(meter.rate(), 0.0)

    def test_reset(self):
        meter = state.RateMeter(1.0)
        meter.tick(0.0)
        meter.tick(0.5)
        self.assertGreater(meter.rate(0.5), 0.0)
        meter.reset()
        self.assertEqual(meter.rate(0.5), 0.0)


class LogBufferTests(unittest.TestCase):

    def test_ring_and_limit(self):
        clock = {'now': 100.0}
        buffer = state.LogBuffer(maxlen=3, clock=lambda: clock['now'])
        for index in range(5):
            buffer.add('info', 'msg%d' % index)
        entries = buffer.recent()
        self.assertEqual(len(entries), 3)
        self.assertEqual(entries[-1]['msg'], 'msg4')
        self.assertEqual(len(buffer.recent(2)), 2)
        self.assertEqual(buffer.recent(1)[0]['msg'], 'msg4')
        self.assertEqual(buffer.recent(0), [])
        self.assertEqual(buffer.recent(None)[0]['level'], 'info')


class DiagnosticAggregationTests(unittest.TestCase):

    def test_empty(self):
        result = state.aggregate_diagnostics([])
        self.assertEqual(result['level'], 'unknown')
        self.assertEqual(result['counts'], {'ok': 0, 'warn': 0, 'error': 0})
        self.assertEqual(result['items'], [])

    def test_levels_and_ordering(self):
        items = [
            {'name': 'a', 'level': 0, 'message': 'ok'},
            {'name': 'b', 'level': 1, 'message': 'warn'},
            {'name': 'c', 'level': 2, 'message': 'err'},
            {'name': 'd', 'level': 3, 'message': 'stale'},
            {'name': 'e', 'level': 0, 'message': 'ok'},
            {'name': 'f', 'level': 9, 'message': 'unknown'},
        ]
        result = state.aggregate_diagnostics(items)
        self.assertEqual(result['level'], 'error')
        self.assertEqual(result['counts'], {'ok': 2, 'warn': 3, 'error': 1})
        names = [entry['name'] for entry in result['items']]
        self.assertEqual(names[:2], ['c', 'b'])

    def test_max_items(self):
        items = [{'name': 'n%d' % index, 'level': 0, 'message': 'ok'}
                 for index in range(30)]
        result = state.aggregate_diagnostics(items)
        self.assertEqual(len(result['items']), state.DIAG_MAX_ITEMS)


class EventBusTests(unittest.TestCase):

    def test_fanout_and_order(self):
        bus = state.EventBus(max_clients=2, queue_size=10)
        cid1, queue1 = bus.subscribe()
        cid2, queue2 = bus.subscribe()
        self.assertEqual(bus.client_count, 2)
        bus.publish('state', {'n': 1})
        bus.publish('log', {'n': 2})
        self.assertEqual(queue1.get_nowait(), ('state', {'n': 1}))
        self.assertEqual(queue1.get_nowait(), ('log', {'n': 2}))
        self.assertEqual(queue2.get_nowait(), ('state', {'n': 1}))
        bus.unsubscribe(cid1)
        bus.unsubscribe(cid2)
        self.assertEqual(bus.client_count, 0)

    def test_drop_oldest_when_full(self):
        bus = state.EventBus(max_clients=1, queue_size=2)
        _cid, client_queue = bus.subscribe()
        for index in range(5):
            bus.publish('state', {'n': index})
        self.assertEqual(client_queue.get_nowait(), ('state', {'n': 3}))
        self.assertEqual(client_queue.get_nowait(), ('state', {'n': 4}))

    def test_max_clients_raises(self):
        bus = state.EventBus(max_clients=1)
        bus.subscribe()
        with self.assertRaises(state.EventBusFull):
            bus.subscribe()

    def test_close_wakes_clients(self):
        bus = state.EventBus(max_clients=2)
        client_id, client_queue = bus.subscribe()
        bus.close()
        self.assertTrue(bus.closed)
        self.assertEqual(client_queue.get_nowait(), state.CLOSE_SENTINEL)
        self.assertEqual(bus.publish('state', {}), 0)
        with self.assertRaises(state.EventBusClosed):
            bus.subscribe()
        bus.unsubscribe(client_id)


class StateStoreTests(unittest.TestCase):

    def setUp(self):
        clock = {'now': 1000.0}
        self.clock = clock
        self.store = state.StateStore(
            {'max_linear': 0.3, 'max_lateral': 0.2, 'max_angular': 1.0},
            clock=lambda: clock['now'], max_logs=5, max_sse=2)

    def test_default_snapshot_schema(self):
        snapshot = self.store.snapshot()
        required = {
            'api', 'server_time', 'uptime_s', 'session', 'ros', 'limits',
            'robot', 'nav', 'map', 'estop', 'teleop', 'peripherals',
            'diagnostics', 'logs',
        }
        self.assertEqual(required - set(snapshot), set())
        self.assertEqual(snapshot['api'], 1)
        self.assertIsInstance(snapshot['server_time'], float)
        self.assertIsInstance(snapshot['uptime_s'], float)
        self.assertIsNone(snapshot['robot'])
        self.assertAlmostEqual(snapshot['limits']['max_lateral'], 0.2)
        session = snapshot['session']
        self.assertEqual((session['mode'], session['state']), ('idle', 'idle'))
        self.assertEqual(session['available_modes'], ['base', 'slam', 'nav'])
        self.assertIsInstance(session['since'], float)
        self.assertEqual(snapshot['nav']['state'], 'unavailable')
        self.assertEqual(snapshot['nav']['activation'],
                         {'done': 0, 'total': 6, 'phase': ''})
        self.assertEqual(snapshot['map']['available'], False)
        self.assertIsNone(snapshot['map']['width'])
        self.assertEqual(snapshot['estop'], {'active': False, 'since': None})
        self.assertEqual(snapshot['diagnostics']['level'], 'unknown')
        self.assertEqual(snapshot['peripherals']['servo']['pan'], 90.0)
        self.assertEqual(snapshot['ros']['ready']['base'], False)

    def test_uptime_and_server_time(self):
        self.clock['now'] = 1010.0
        snapshot = self.store.snapshot()
        self.assertAlmostEqual(snapshot['server_time'], 1010.0)
        self.assertAlmostEqual(snapshot['uptime_s'], 10.0)

    def test_session_update_deepcopy(self):
        self.store.set_session({'mode': 'slam', 'state': 'starting'})
        snapshot = self.store.session_snapshot()
        self.assertEqual(snapshot['state'], 'starting')
        self.assertEqual(snapshot['managed'], True)  # 默认值补齐
        snapshot['mode'] = 'changed'
        self.assertEqual(self.store.session_snapshot()['mode'], 'slam')

    def test_map_versioning(self):
        self.store.set_map(_grid(stamp=1.0))
        snapshot = self.store.snapshot()
        self.assertEqual(snapshot['map']['available'], True)
        self.assertEqual(snapshot['map']['version'], 1)
        changed, meta = self.store.set_map(_grid(stamp=2.0))
        self.assertFalse(changed)
        self.assertEqual(meta['version'], 1)
        changed, meta = self.store.set_map(_grid(data=(0, 0, 0, 0)))
        self.assertTrue(changed)
        self.assertEqual(meta['version'], 2)
        self.assertIsNotNone(snapshot['ros']['rates']['map_age_s'])

    def test_robot(self):
        robot = {'frame': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.1,
                 'stamp': 1.0, 'speed': {'vx': 0, 'vy': 0, 'wz': 0},
                 'imu': {'yaw_deg': 5.0, 'age_s': 0.1},
                 'scan': {'front_m': 1.0, 'min_m': 0.5, 'age_s': 0.1}}
        self.store.set_robot(robot)
        snapshot = self.store.snapshot()
        self.assertEqual(snapshot['robot']['frame'], 'map')
        robot['x'] = 99
        self.assertEqual(self.store.snapshot()['robot']['x'], 1.0)
        self.store.set_robot(None)
        self.assertIsNone(self.store.snapshot()['robot'])

    def test_estop_and_commanded(self):
        self.assertFalse(self.store.estop_commanded)
        self.store.set_estop(True)
        estop = self.store.estop_snapshot()
        self.assertTrue(estop['active'])
        self.assertAlmostEqual(estop['since'], 1000.0)
        self.clock['now'] = 1005.0
        self.store.set_estop(True)  # 重复置 true 不刷新 since
        self.assertAlmostEqual(self.store.estop_snapshot()['since'], 1000.0)
        self.store.mark_estop_commanded()
        self.assertTrue(self.store.estop_commanded)
        self.store.set_estop(False)
        self.assertEqual(self.store.estop_snapshot(),
                         {'active': False, 'since': None})

    def test_teleop_watchdog(self):
        self.assertEqual(self.store.consume_teleop(1000.0, 0.5),
                         ('idle', None))
        self.store.set_teleop(True, 0.2, 0.0, 0.5, last_cmd=1000.0, seq=7)
        self.assertEqual(self.store.consume_teleop(1000.2, 0.5),
                         ('active', (0.2, 0.0, 0.5)))
        self.assertEqual(self.store.get_teleop_seq(), 7)
        self.assertEqual(self.store.consume_teleop(1001.0, 0.5),
                         ('expired', None))
        self.assertFalse(self.store.teleop_snapshot()['active'])
        self.store.stop_teleop()
        self.assertEqual(self.store.teleop_snapshot()['vx'], 0.0)

    def test_nav_updates(self):
        self.store.set_nav(state='activating', message='x',
                           activation={'done': 1, 'total': 6, 'phase': 'a'})
        nav = self.store.nav_snapshot()
        self.assertEqual(nav['state'], 'activating')
        self.assertEqual(nav['activation']['done'], 1)
        self.store.reset_nav()
        self.assertEqual(self.store.nav_snapshot()['state'], 'unavailable')

    def test_peripherals(self):
        self.store.set_peripherals(buzzer=True, led={'r': 0.1, 'g': 0.2,
                                                     'b': 0.3},
                                   servo={'pan': 10.0, 'tilt': 100.0})
        peripherals = self.store.peripherals_snapshot()
        self.assertTrue(peripherals['buzzer'])
        self.assertAlmostEqual(peripherals['servo']['tilt'], 100.0)

    def test_diagnostics(self):
        self.clock['now'] = 1000.5
        self.store.set_diagnostics([
            {'name': 'Raspbot Safety: E-Stop', 'level': 1, 'message': 'x',
             'hardware_id': 'estop'},
        ])
        snapshot = self.store.snapshot()
        self.assertEqual(snapshot['diagnostics']['level'], 'warn')
        self.assertEqual(snapshot['diagnostics']['age_s'], 0.0)
        self.assertEqual(snapshot['ros']['ready']['diagnostics'], False)

    def test_logs_and_events(self):
        for index in range(10):
            self.store.add_log('info', 'm%d' % index)
        snapshot = self.store.snapshot()
        self.assertEqual(len(snapshot['logs']), 5)  # max_logs=5
        self.assertEqual(snapshot['logs'][-1]['msg'], 'm9')
        self.assertEqual(self.store.log_entries(2)[-1]['msg'], 'm9')
        self.assertIsInstance(snapshot['logs'][0]['ts'], float)

    def test_ros_rates(self):
        for index in range(11):
            self.clock['now'] = 1000.0 + index * 0.1
            self.store.tick_scan()
            self.store.tick_odom()
        snapshot = self.store.snapshot()
        self.assertAlmostEqual(snapshot['ros']['rates']['scan_hz'], 10.0,
                               delta=0.5)
        self.assertAlmostEqual(snapshot['ros']['rates']['odom_hz'], 10.0,
                               delta=0.5)

    def test_snapshot_json_serializable(self):
        import json
        self.store.set_robot({'frame': 'map', 'x': float('nan'),
                              'y': 0.0, 'yaw': 0.0, 'stamp': 1.0,
                              'speed': {'vx': 0, 'vy': 0, 'wz': 0},
                              'imu': {'yaw_deg': None, 'age_s': None},
                              'scan': {'front_m': None, 'min_m': None,
                                       'age_s': None}})
        snapshot = self.store.snapshot()
        text = json.dumps(snapshot, allow_nan=False)
        self.assertIn('"x": null', text)


if __name__ == '__main__':
    unittest.main()
