"""session 会话状态机单元测试（离线，用假进程/假时钟）。"""


import itertools
import os
import sys
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from raspbot_console import session  # noqa: E402
from raspbot_console.session import LaunchSpec, SessionError  # noqa: E402


class FakeClock:
    def __init__(self, value=1000.0):
        self.value = float(value)

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += float(seconds)


class FakeProc:
    _pids = itertools.count(1000)

    def __init__(self, command, **kwargs):
        self.command = list(command)
        self.kwargs = kwargs
        self.pid = next(self._pids)
        self.returncode = None
        self.stdout = __import__('io').StringIO(
            'bringup line 1\nbringup line 2\n')
        self.terminated = False

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


class CommandBuilderTests(unittest.TestCase):

    def test_base_command_matches_contract(self):
        command = session.build_launch_command(LaunchSpec('base'))
        self.assertEqual(command[:2], ['bash', '-lc'])
        self.assertEqual(
            command[2],
            'source /opt/ros/lyrical/setup.bash && '
            'source "$HOME/ros2_ws/install/setup.bash" && '
            'exec ros2 launch raspbot_bringup bringup.launch.py mode:=none')

    def test_slam_with_mock_and_camera(self):
        command = session.build_launch_command(
            LaunchSpec('slam', use_mock=True, use_camera=True,
                       use_perception=True))
        script = command[2]
        self.assertIn('mode:=slam', script)
        self.assertIn('use_mock:=true', script)
        self.assertIn('use_camera:=true', script)
        self.assertIn('use_perception:=true', script)
        self.assertNotIn('use_mock:=false', script)

    def test_nav_requires_and_passes_map(self):
        command = session.build_launch_command(
            LaunchSpec('nav', map_path='/mnt/ssd/maps/room.yaml'))
        self.assertIn('mode:=nav map:=/mnt/ssd/maps/room.yaml', command[2])
        with self.assertRaises(ValueError):
            session.build_launch_command(LaunchSpec('nav'))

    def test_custom_workspace(self):
        command = session.build_launch_command(
            LaunchSpec('base'), workspace='/opt/custom_ws')
        self.assertIn('install/setup.bash', command[2].replace('\\', '/'))
        self.assertNotIn('$HOME/ros2_ws', command[2])

    def test_unknown_mode(self):
        with self.assertRaises(ValueError):
            session.build_launch_command(LaunchSpec('other'))


class SessionManagerTests(unittest.TestCase):

    def setUp(self):
        self.clock = FakeClock()
        self.factory = ProcFactory()
        self.signals = []
        self._original_signal = session.signal_process_group
        session.signal_process_group = (
            lambda proc, sig: self.signals.append((proc.pid, sig)))
        self.ready = {'value': True}
        self.external = {'value': False}
        self.infer = {'value': None}
        self.manager = session.SessionManager(
            ready_check=lambda mode: self.ready['value'],
            external_check=lambda: self.external['value'],
            infer_mode=lambda: self.infer['value'],
            clock=self.clock,
            wall_clock=self.clock,
            popen_factory=self.factory,
        )

    def tearDown(self):
        session.signal_process_group = self._original_signal

    # -- 工具 -----------------------------------------------------------
    def _start(self, mode='slam', **kwargs):
        snapshot = self.manager.start(LaunchSpec(mode, **kwargs))
        time.sleep(0.02)  # 等 stdout reader 线程读完假进程输出
        return snapshot

    def _make_running(self):
        self._start('slam')
        self.clock.advance(0.6)
        snapshot = self.manager.tick()
        self.assertEqual(snapshot['state'], 'running')
        return snapshot

    # -- 启动 -----------------------------------------------------------
    def test_start_enters_starting_then_running(self):
        snapshot = self._start('slam')
        self.assertEqual(snapshot['mode'], 'slam')
        self.assertEqual(snapshot['state'], 'starting')
        self.assertFalse(snapshot['mock'])
        self.assertTrue(snapshot['managed'])
        self.assertEqual(snapshot['available_modes'], ['base', 'slam', 'nav'])
        self.assertAlmostEqual(snapshot['progress'], 0.0)
        self.assertEqual(len(self.factory.procs), 1)
        proc = self.factory.procs[0]
        self.assertTrue(proc.kwargs.get('start_new_session'))
        self.assertIn('mode:=slam', proc.command[2])

        self.clock.advance(0.6)
        snapshot = self.manager.tick()
        self.assertEqual(snapshot['state'], 'running')
        self.assertAlmostEqual(snapshot['progress'], 1.0)
        self.assertIn('建图', snapshot['message'])
        self.assertEqual(proc.poll(), None)

    def test_mode_mapping_mock_and_map(self):
        snapshot = self._start('nav', use_mock=True, map_path='/tmp/room.yaml')
        self.assertEqual(snapshot['mode'], 'nav')
        self.assertEqual(snapshot['map'], '/tmp/room.yaml')
        self.assertTrue(snapshot['mock'])
        script = self.factory.procs[0].command[2]
        self.assertIn('mode:=nav map:=/tmp/room.yaml', script)
        self.assertIn('use_mock:=true', script)

    def test_start_while_starting_is_busy(self):
        self._start('base')
        with self.assertRaises(SessionError) as ctx:
            self.manager.start(LaunchSpec('slam'))
        self.assertEqual(ctx.exception.code, 'busy')

    def test_external_stack_refuses_start(self):
        self.external['value'] = True
        with self.assertRaises(SessionError) as ctx:
            self.manager.start(LaunchSpec('base'))
        self.assertEqual(ctx.exception.code, 'external_stack')
        self.assertEqual(self.manager.state, session.STATE_IDLE)

    def test_start_timeout_enters_error(self):
        self.ready['value'] = False
        self._start('base')
        self.clock.advance(60.0)
        snapshot = self.manager.tick()
        self.assertEqual(snapshot['state'], 'starting')
        self.assertAlmostEqual(snapshot['progress'], 0.5, delta=0.02)
        self.clock.advance(61.0)
        snapshot = self.manager.tick()
        self.assertEqual(snapshot['state'], 'error')
        self.assertIn('启动超时', snapshot['error'])
        self.assertIn('bringup line 2', snapshot['error'])
        proc = self.factory.procs[0]
        self.assertIn((proc.pid, session.SIGINT), self.signals)
        # error → stop 复位到 idle
        accepted, snapshot = self.manager.stop()
        self.assertFalse(accepted)
        self.assertEqual(snapshot['state'], 'idle')

    def test_process_early_exit_enters_error(self):
        self.ready['value'] = False
        self._start('slam')
        self.factory.procs[0].returncode = 1
        snapshot = self.manager.tick()
        self.assertEqual(snapshot['state'], 'error')
        self.assertIn('提前退出', snapshot['error'])
        self.assertIn('exit code 1', snapshot['error'])
        self.assertEqual(self.manager.mode, 'slam')

    # -- 停止 -----------------------------------------------------------
    def test_stop_running_sends_sigint_then_idle(self):
        self._make_running()
        proc = self.factory.procs[0]
        accepted, snapshot = self.manager.stop()
        self.assertTrue(accepted)
        self.assertEqual(snapshot['state'], 'stopping')
        self.assertIn((proc.pid, session.SIGINT), self.signals)
        self.clock.advance(0.5)
        self.manager.tick()
        self.assertEqual(self.manager.state, 'stopping')
        proc.returncode = 0
        self.clock.advance(0.5)
        snapshot = self.manager.tick()
        self.assertEqual(snapshot['state'], 'idle')
        self.assertEqual(snapshot['mode'], 'idle')
        self.assertIsNone(snapshot['map'])

    def test_stop_escalates_sigterm_then_sigkill(self):
        self._make_running()
        proc = self.factory.procs[0]
        self.manager.stop()
        self.clock.advance(21.0)
        self.manager.tick()
        self.assertIn((proc.pid, session.SIGTERM), self.signals)
        self.clock.advance(6.0)
        self.manager.tick()
        self.assertIn((proc.pid, session.SIGKILL), self.signals)
        proc.returncode = 0
        self.clock.advance(0.5)
        snapshot = self.manager.tick()
        self.assertEqual(snapshot['state'], 'idle')

    def test_stop_when_idle(self):
        accepted, snapshot = self.manager.stop()
        self.assertFalse(accepted)
        self.assertEqual(snapshot['state'], 'idle')

    def test_stop_when_stopping_is_idempotent(self):
        self._make_running()
        accepted1, _ = self.manager.stop()
        accepted2, snapshot = self.manager.stop()
        self.assertTrue(accepted1)
        self.assertTrue(accepted2)
        self.assertEqual(snapshot['state'], 'stopping')

    def test_mode_switch_stops_then_starts(self):
        self._make_running()
        old_proc = self.factory.procs[0]
        snapshot = self.manager.start(
            LaunchSpec('nav', map_path='/tmp/room.yaml'))
        self.assertEqual(snapshot['state'], 'stopping')
        old_proc.returncode = 0
        self.clock.advance(0.6)
        snapshot = self.manager.tick()
        self.assertEqual(snapshot['state'], 'starting')
        self.assertEqual(snapshot['mode'], 'nav')
        self.assertEqual(len(self.factory.procs), 2)
        self.assertIn('mode:=nav', self.factory.procs[1].command[2])

    # -- 附加模式 -------------------------------------------------------
    def test_attach_mode_refuses_start_stop(self):
        manager = session.SessionManager(attach=True, clock=self.clock,
                                        wall_clock=self.clock)
        with self.assertRaises(SessionError) as ctx:
            manager.start(LaunchSpec('base'))
        self.assertEqual(ctx.exception.code, 'attach_mode')
        with self.assertRaises(SessionError):
            manager.stop()
        self.assertFalse(manager.snapshot()['managed'])

    def test_no_session_manage_refuses_start(self):
        manager = session.SessionManager(manage=False, clock=self.clock,
                                        wall_clock=self.clock)
        with self.assertRaises(SessionError) as ctx:
            manager.start(LaunchSpec('base'))
        self.assertEqual(ctx.exception.code, 'attach_mode')

    def test_attach_infers_mode(self):
        manager = session.SessionManager(
            attach=True, infer_mode=lambda: self.infer['value'],
            clock=self.clock, wall_clock=self.clock)
        self.infer['value'] = 'slam'
        self.clock.advance(2.1)
        snapshot = manager.tick()
        self.assertEqual(snapshot['state'], 'running')
        self.assertEqual(snapshot['mode'], 'slam')
        self.assertTrue(snapshot['external'])
        self.assertFalse(snapshot['managed'])
        self.infer['value'] = None
        self.clock.advance(2.1)
        snapshot = manager.tick()
        self.assertEqual(snapshot['state'], 'idle')

    # -- 日志 -----------------------------------------------------------
    def test_session_logs_and_tail(self):
        self.ready['value'] = False
        self._start('base')
        self.factory.procs[0].returncode = 2
        self.manager.tick()
        logs = self.manager.session_logs(500)
        self.assertTrue(logs)
        self.assertIn('line', logs[-1])
        self.assertTrue(logs[0]['line'].startswith('$ bash -lc'))
        self.assertIsInstance(logs[0]['ts'], float)
        tail = self.manager.tail_summary(5)
        self.assertIn('bringup line', tail)
        self.assertEqual(len(self.manager.session_logs(1)), 1)


if __name__ == '__main__':
    unittest.main()
