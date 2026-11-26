"""会话管理：状态机 + ``ros2 launch`` 子进程（纯标准库，可离线测试）。

设计文档 §5.2：

- 启动命令统一 source ROS/工作区后 ``exec ros2 launch raspbot_bringup
  bringup.launch.py mode:=...``，``start_new_session=True`` 便于整组停止；
- 模式映射：``base→mode:=none``、``slam→mode:=slam``、``nav→mode:=nav map:=...``；
- 就绪判据每 0.5 s 轮询（超时 120 s），由 :class:`SessionManager` 注入的
  ``ready_check(mode)`` 提供（由 controller 用 ROS 图判定）；
- 启动失败/进程异常退出 → ``session.state=error``，``error`` 带最后 30 行日志摘要；
- 停止：SIGINT 进程组 → 20 s 超时 SIGTERM → 5 s → SIGKILL；
- 外部栈检测与 ``--attach`` 附加模式。

本模块不做任何 ROS 调用；``os``/``subprocess`` 之外只用标准库，测试用
``popen_factory`` 注入假进程。
"""

from __future__ import annotations

import os
import shlex
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass

#: 运行模式
MODES = ('base', 'slam', 'nav')

#: 网关模式 → bringup.launch.py 的 mode 参数（§5.2）
MODE_LAUNCH = {'base': 'none', 'slam': 'slam', 'nav': 'nav'}

#: 会话状态机状态（§3.1 session.state）
STATE_IDLE = 'idle'
STATE_STARTING = 'starting'
STATE_RUNNING = 'running'
STATE_STOPPING = 'stopping'
STATE_ERROR = 'error'

MODE_IDLE = 'idle'

#: 默认 ROS 发行版 setup 脚本
DEFAULT_ROS_SETUP = '/opt/ros/lyrical/setup.bash'

#: 默认工作区
DEFAULT_WORKSPACE = '~/ros2_ws'

#: 中文状态文案（§6.4 文案表的网关侧对应）
MODE_TEXT = {'base': '手动遥控', 'slam': '建图', 'nav': '导航'}

IDLE_MESSAGE = '空闲：可选择运行模式'
ERROR_MESSAGE = '启动失败或进程异常退出'

#: session.state=running 的文案
def running_message(mode):
    return '运行中：%s' % MODE_TEXT.get(mode, mode)


#: 信号常量（Windows 无 SIGKILL，测试/离线环境退化用数值）
SIGINT = getattr(signal, 'SIGINT', 2)
SIGTERM = getattr(signal, 'SIGTERM', 15)
SIGKILL = getattr(signal, 'SIGKILL', 9)


@dataclass(frozen=True)
class LaunchSpec:
    """一次 ``ros2 launch`` 请求的完整参数。"""

    mode: str
    map_path: str = None
    use_mock: bool = False
    use_camera: bool = False
    use_perception: bool = False


class SessionError(Exception):
    """会话操作冲突；``code`` 会被 controller 映射成 ApiError 错误码。"""

    def __init__(self, code, message):
        self.code = str(code)
        self.message = str(message)
        super().__init__(self.message)


# ---------------------------------------------------------------------------
# 启动命令构造（纯函数，可在任意机器上单测）
# ---------------------------------------------------------------------------

_SIMPLE_SHELL = set('abcdefghijklmnopqrstuvwxyz'
                    'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
                    '0123456789_./-')


def _shell_arg(text):
    """路径参数：安全字符原样输出，否则 bash 单引号转义（保持文档字面形式）。"""
    text = str(text)
    if text and all(ch in _SIMPLE_SHELL for ch in text):
        return text
    return shlex.quote(text)


def workspace_setup_expr(workspace):
    """生成 ``source`` 用的工作区 setup 表达式。

    默认 ``~/ros2_ws`` → 文档字面形式 ``"$HOME/ros2_ws/install/setup.bash"``；
    其它绝对路径按需转义。
    """
    raw = str(workspace or DEFAULT_WORKSPACE)
    expanded = os.path.normpath(os.path.expanduser(raw))
    home = os.path.normpath(os.path.expanduser('~'))
    if expanded == home:
        return '"$HOME/install/setup.bash"'
    prefix = home + os.sep
    if expanded.startswith(prefix):
        rel = expanded[len(prefix):].replace(os.sep, '/').strip('/')
        if rel:
            return '"$HOME/%s/install/setup.bash"' % rel
    return _shell_arg(os.path.join(expanded, 'install', 'setup.bash'))


def build_launch_command(spec, workspace=DEFAULT_WORKSPACE,
                         ros_setup=DEFAULT_ROS_SETUP, ros_log_dir=''):
    """构造 ``bash -lc`` 启动命令（§5.2），返回 list[str] 供 Popen 使用。

    ``ros_log_dir`` 非空时在 source 环境后显式 ``export ROS_LOG_DIR=<dir>``，
    避免节点把日志写到慢速/不稳定的 SSD 上（Pi 上 /mnt/ssd UAS 掉盘会拖住
    节点线程，进而导致 DDS 发现/lifecycle 激活超时）。
    """
    if spec.mode not in MODE_LAUNCH:
        raise ValueError('未知会话模式: %r' % (spec.mode,))
    if spec.mode == 'nav' and not spec.map_path:
        raise ValueError('nav 模式必须提供 map 路径')
    launch = 'ros2 launch raspbot_bringup bringup.launch.py mode:=%s' % (
        MODE_LAUNCH[spec.mode],)
    if spec.mode == 'nav':
        launch += ' map:=%s' % _shell_arg(spec.map_path)
    if spec.use_mock:
        launch += ' use_mock:=true'
    if spec.use_camera:
        launch += ' use_camera:=true'
    if spec.use_perception:
        launch += ' use_perception:=true'
    env_prefix = ''
    if ros_log_dir:
        env_prefix = 'export ROS_LOG_DIR=%s && ' % _shell_arg(str(ros_log_dir))
    script = 'source %s && source %s && %sexec %s' % (
        _shell_arg(ros_setup or DEFAULT_ROS_SETUP),
        workspace_setup_expr(workspace),
        env_prefix,
        launch,
    )
    return ['bash', '-lc', script]


def signal_process_group(proc, sig):
    """向子进程 **进程组** 发信号；失败时退化到单进程（便于测试/兼容）。

    本函数是模块级函数，测试可 monkeypatch 记录信号序列。
    """
    if proc is None:
        return
    pid = getattr(proc, 'pid', None)
    try:
        if pid is not None and hasattr(os, 'killpg'):
            os.killpg(os.getpgid(pid), sig)
            return
    except (ProcessLookupError, PermissionError, OSError, AttributeError):
        pass
    try:
        if sig == SIGKILL and hasattr(proc, 'kill'):
            proc.kill()
        elif sig == SIGTERM and hasattr(proc, 'terminate'):
            proc.terminate()
        elif hasattr(proc, 'send_signal'):
            proc.send_signal(sig)
    except (ProcessLookupError, OSError, AttributeError):
        pass


class SessionManager:
    """托管 bringup 会话（状态机 + 子进程 + stdout 环形缓冲）。

    线程模型：HTTP 线程调用 :meth:`start`/:meth:`stop`，会话监控线程周期性调用
    :meth:`tick`；所有状态读写都在内部 ``RLock`` 下。
    """

    def __init__(self, workspace=DEFAULT_WORKSPACE,
                 ros_setup=DEFAULT_ROS_SETUP, *,
                 ros_log_dir='',
                 timeout_s=120.0, stop_sigint_timeout_s=20.0,
                 stop_sigterm_timeout_s=5.0, session_log_maxlen=500,
                 attach=False, manage=True,
                 ready_check=None, external_check=None, infer_mode=None,
                 clock=time.monotonic, wall_clock=time.time,
                 popen_factory=subprocess.Popen):
        self.workspace = workspace
        self.ros_setup = ros_setup or DEFAULT_ROS_SETUP
        self.ros_log_dir = str(ros_log_dir or '')
        self.timeout_s = float(timeout_s)
        self.stop_sigint_timeout_s = float(stop_sigint_timeout_s)
        self.stop_sigterm_timeout_s = float(stop_sigterm_timeout_s)
        self.session_log_maxlen = int(session_log_maxlen)
        self.attach = bool(attach)
        self.manage = bool(manage)
        self.ready_check = ready_check
        self.external_check = external_check
        self.infer_mode = infer_mode
        self._clock = clock
        self._wall_clock = wall_clock
        self._popen_factory = popen_factory

        self._lock = threading.RLock()
        self._proc = None
        self._reader_thread = None
        self._mode = MODE_IDLE
        self._state = STATE_IDLE
        self._since = float(self._wall_clock())
        self._progress = 0.0
        self._message = IDLE_MESSAGE
        self._error = None
        self._map = None
        self._mock = False
        self._managed = bool(self.manage and not self.attach)
        self._external = bool(self.attach) or (not self._managed)
        self._pending = None
        self._start_mono = None
        self._stop_mono = None
        self._error_mono = None
        self._sigterm_sent = False
        self._sigkill_sent = False
        self._session_logs = deque(maxlen=self.session_log_maxlen)
        self._last_ext_poll = 0.0
        self._last_attach_poll = 0.0
        self._last_ready_poll = 0.0

    # ------------------------------------------------------------------
    # 注入
    # ------------------------------------------------------------------
    def set_ready_check(self, callback):
        self.ready_check = callback

    def set_external_check(self, callback):
        self.external_check = callback

    def set_infer_mode(self, callback):
        self.infer_mode = callback

    # ------------------------------------------------------------------
    # 只读属性
    # ------------------------------------------------------------------
    @property
    def managed(self):
        with self._lock:
            return self._managed

    @property
    def external(self):
        with self._lock:
            return self._external

    @property
    def mode(self):
        with self._lock:
            return self._mode

    @property
    def state(self):
        with self._lock:
            return self._state

    @property
    def process(self):
        with self._lock:
            return self._proc

    def set_external(self, external):
        with self._lock:
            if self._managed and self._state == STATE_IDLE:
                self._external = bool(external)

    def is_idle(self):
        with self._lock:
            return self._state == STATE_IDLE and self._mode == MODE_IDLE

    def _set_state(self, state, message=None, progress=None):
        """状态切换（调用方需持有锁）；同步刷新 since。"""
        if self._state != state:
            self._since = float(self._wall_clock())
        self._state = state
        if message is not None:
            self._message = message
        if progress is not None:
            self._progress = float(progress)

    # ------------------------------------------------------------------
    # 启动
    # ------------------------------------------------------------------
    def start(self, spec):
        """请求启动/切换模式；返回启动中/停止中的 session 快照。

        冲突（忙/外部栈/附加模式）→ :class:`SessionError`（controller 转 409）。
        """
        if not isinstance(spec, LaunchSpec):
            spec = LaunchSpec(**spec)
        with self._lock:
            if not self._managed:
                raise SessionError(
                    'attach_mode',
                    '网关未托管会话（--attach/--no-session-manage），'
                    '请由外部 systemd 管理 bringup')
            if self._state in (STATE_STARTING, STATE_STOPPING):
                raise SessionError('busy', '会话正在启动/停止，请稍后重试')
            if self._state == STATE_IDLE:
                # 启动前重新探测外部栈（缓存值可能是刚停止会话瞬间的误判）
                external = self._external
                if self.external_check is not None:
                    try:
                        external = bool(self.external_check())
                    except Exception:
                        external = self._external
                self._external = external
                if external:
                    raise SessionError(
                        'external_stack',
                        '检测到外部 ROS 栈在运行；请先停止 systemd 服务或使用 '
                        '--attach 附加模式（避免双 owner）')
                self._begin_start(spec)
                return self.snapshot()
            # 运行中（或 error 残留）：先停止，再在同一请求内启动新会话
            if self._state == STATE_ERROR:
                self._reset_to_idle_locked()
                self._begin_start(spec)
                return self.snapshot()
            self._pending = spec
            self._begin_stop_locked()
            return self.snapshot()

    def _begin_start(self, spec):
        """（调用方持有锁）拉起子进程并进入 starting。"""
        self._pending = None
        self._mode = spec.mode
        self._mock = bool(spec.use_mock)
        self._map = spec.map_path if spec.mode == 'nav' else None
        self._error = None
        self._sigterm_sent = False
        self._sigkill_sent = False
        try:
            command = build_launch_command(spec, self.workspace, self.ros_setup,
                                           self.ros_log_dir)
        except ValueError as exc:
            self._error = str(exc)
            self._set_state(STATE_ERROR, ERROR_MESSAGE, 0.0)
            raise SessionError('bad_request', str(exc))
        self._session_logs.append({
            'ts': float(self._wall_clock()),
            'line': '$ ' + ' '.join(command),
        })
        try:
            proc = self._popen_factory(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except (OSError, ValueError) as exc:
            self._error = '无法启动 bringup 进程: %s' % exc
            self._set_state(STATE_ERROR, ERROR_MESSAGE, 0.0)
            self._error_mono = self._clock()
            raise SessionError('ros_not_ready', self._error)
        self._proc = proc
        self._start_mono = self._clock()
        self._last_ready_poll = 0.0
        self._set_state(STATE_STARTING,
                        '正在启动（%s）…' % MODE_TEXT.get(spec.mode, spec.mode),
                        0.0)
        self._reader_thread = threading.Thread(
            target=self._reader_loop, args=(proc,),
            name='session-stdout', daemon=True)
        self._reader_thread.start()
        return proc

    def _reader_loop(self, proc):
        """读取 bringup stdout（stderr 已并入），写入 500 行环形缓冲。"""
        stream = getattr(proc, 'stdout', None)
        if stream is None:
            return
        try:
            for raw_line in iter(stream.readline, ''):
                line = str(raw_line).rstrip('\r\n')
                if not line:
                    continue
                with self._lock:
                    self._session_logs.append({
                        'ts': float(self._wall_clock()),
                        'line': line,
                    })
        except (OSError, ValueError):
            pass
        finally:
            try:
                stream.close()
            except (OSError, AttributeError):
                pass

    # ------------------------------------------------------------------
    # 停止
    # ------------------------------------------------------------------
    def stop(self):
        """请求停止；返回 ``(accepted, snapshot)``。

        ``accepted=True`` → 202（异步停止中）；``False`` → 200（本来就空闲）。
        附加模式抛 ``attach_mode``，忙时抛 ``busy``。
        """
        with self._lock:
            if not self._managed:
                raise SessionError('attach_mode',
                                   '网关未托管会话（--attach），不执行停止')
            if self._state == STATE_STOPPING:
                return True, self.snapshot()
            if self._state in (STATE_STARTING, STATE_RUNNING):
                self._pending = None
                self._begin_stop_locked()
                return True, self.snapshot()
            if self._state == STATE_ERROR:
                self._reset_to_idle_locked()
                return False, self.snapshot()
            self._reset_to_idle_locked()
            return False, self.snapshot()

    def _begin_stop_locked(self):
        proc = self._proc
        self._set_state(STATE_STOPPING, '正在停止…', 0.0)
        self._stop_mono = self._clock()
        self._sigterm_sent = False
        self._sigkill_sent = False
        if proc is None:
            self._finalize_stop_locked()
            return
        if proc.poll() is not None:
            self._finalize_stop_locked()
            return
        signal_process_group(proc, SIGINT)

    def _finalize_stop_locked(self):
        """（调用方持有锁）停止结束 → idle，若存在待启动请求则继续启动。"""
        if self._reader_thread is not None and not self._reader_thread.is_alive():
            self._reader_thread = None
        self._proc = None
        self._mode = MODE_IDLE
        self._map = None
        self._mock = False
        self._error = None
        pending = self._pending
        self._pending = None
        self._set_state(STATE_IDLE, IDLE_MESSAGE, 0.0)
        if pending is not None:
            try:
                self._begin_start(pending)
            except SessionError:
                # _begin_start 已把状态置为 error
                pass

    def _reset_to_idle_locked(self):
        proc = self._proc
        self._proc = None
        if proc is not None and proc.poll() is None:
            signal_process_group(proc, SIGKILL)
        self._mode = MODE_IDLE
        self._map = None
        self._mock = False
        self._error = None
        self._pending = None
        self._set_state(STATE_IDLE, IDLE_MESSAGE, 0.0)

    # ------------------------------------------------------------------
    # 周期 tick
    # ------------------------------------------------------------------
    def tick(self, now=None):
        """周期推进状态机（建议 0.2 s 调用；内部按 0.5 s 做就绪轮询）。"""
        now = self._clock() if now is None else float(now)
        with self._lock:
            state = self._state
            if state == STATE_STARTING:
                self._tick_starting_locked(now)
            elif state == STATE_RUNNING:
                self._tick_running_locked(now)
            elif state == STATE_STOPPING:
                self._tick_stopping_locked(now)
            elif state == STATE_ERROR:
                self._tick_error_locked(now)
            if not self._managed:
                self._tick_attach_locked(now)
            elif self._state == STATE_IDLE:
                self._tick_external_locked(now)
            return self.snapshot()

    def _tick_starting_locked(self, now):
        proc = self._proc
        if proc is not None and proc.poll() is not None:
            self._finalize_error_locked(
                'bringup 进程提前退出（exit code %r）' % (proc.poll(),))
            return
        elapsed = max(0.0, now - (self._start_mono or now))
        ready = False
        if (self.ready_check is not None
                and now - self._last_ready_poll >= 0.5):
            self._last_ready_poll = now
            try:
                ready = bool(self.ready_check(self._mode))
            except Exception:
                ready = False
        if ready:
            self._progress = 1.0
            self._set_state(STATE_RUNNING, running_message(self._mode), 1.0)
            return
        if elapsed > self.timeout_s:
            self._finalize_error_locked(
                '启动超时（%.0f s 内未就绪：/odom、/cmd_vel_safe、地图等关键链路）'
                % self.timeout_s, kill=True)
            return
        self._progress = min(0.95, elapsed / self.timeout_s)

    def _tick_running_locked(self, now):
        proc = self._proc
        if proc is not None and proc.poll() is not None:
            self._finalize_error_locked(
                'bringup 进程异常退出（exit code %r）' % (proc.poll(),))
            return
        if self._external:
            self._external = False

    def _tick_stopping_locked(self, now):
        proc = self._proc
        if proc is None or proc.poll() is not None:
            self._finalize_stop_locked()
            return
        elapsed = max(0.0, now - (self._stop_mono or now))
        if (elapsed > self.stop_sigint_timeout_s) and not self._sigterm_sent:
            self._sigterm_sent = True
            signal_process_group(proc, SIGTERM)
        if (elapsed > self.stop_sigint_timeout_s + self.stop_sigterm_timeout_s
                and not self._sigkill_sent):
            self._sigkill_sent = True
            signal_process_group(proc, SIGKILL)
        self._progress = min(0.95, elapsed / max(
            1.0, self.stop_sigint_timeout_s + self.stop_sigterm_timeout_s))
        if elapsed > (self.stop_sigint_timeout_s
                      + self.stop_sigterm_timeout_s + 5.0):
            # SIGKILL 后进程仍未回收：兜底直接进入 idle（不阻塞接口）
            self._finalize_stop_locked()

    def _tick_error_locked(self, now):
        """error 状态下确保子进程被回收（超时启动失败时可能还活着）。"""
        proc = self._proc
        if proc is None:
            return
        if proc.poll() is not None:
            self._proc = None
            return
        if self._error_mono is None:
            self._error_mono = now
        if not self._sigterm_sent:
            self._sigterm_sent = True
            signal_process_group(proc, SIGINT)
        elif now - self._error_mono > 5.0 and not self._sigkill_sent:
            self._sigkill_sent = True
            signal_process_group(proc, SIGKILL)

    def _tick_attach_locked(self, now):
        """附加模式：每 2 s 根据 ROS 图推断当前模式。"""
        if now - self._last_attach_poll < 2.0:
            return
        self._last_attach_poll = now
        inferred = None
        if self.infer_mode is not None:
            try:
                inferred = self.infer_mode()
            except Exception:
                inferred = None
        if inferred not in MODES:
            inferred = None
        if inferred is None:
            if self._mode != MODE_IDLE or self._state != STATE_IDLE:
                self._mode = MODE_IDLE
                self._error = None
                self._set_state(STATE_IDLE, IDLE_MESSAGE, 0.0)
            self._external = True
            return
        self._external = True
        if self._mode != inferred:
            self._mode = inferred
            self._set_state(STATE_RUNNING,
                            '已附加外部 ROS 栈：%s' % MODE_TEXT.get(inferred,
                                                                    inferred),
                            1.0)
        elif self._state != STATE_RUNNING:
            self._set_state(STATE_RUNNING,
                            '已附加外部 ROS 栈：%s' % MODE_TEXT.get(inferred,
                                                                    inferred),
                            1.0)

    def _tick_external_locked(self, now):
        """托管模式下检测外部栈（每 2 s）。"""
        if now - self._last_ext_poll < 2.0:
            return
        self._last_ext_poll = now
        if self.external_check is None:
            return
        try:
            self._external = bool(self.external_check())
        except Exception:
            self._external = False

    def _finalize_error_locked(self, message, kill=False):
        """（调用方持有锁）进入 error 状态；可选杀掉残留进程组。"""
        proc = self._proc
        if kill and proc is not None and proc.poll() is None:
            signal_process_group(proc, SIGINT)
            self._error_mono = None  # 交给 _tick_error_locked 升级 SIGKILL
        tail = self.tail_summary_locked(30)
        error = message
        if tail:
            error = '%s\n--- bringup 日志尾部 ---\n%s' % (message, tail)
        self._error = error[:8000]
        self._set_state(STATE_ERROR, ERROR_MESSAGE, 0.0)
        if self._error_mono is None:
            self._error_mono = self._clock()
        # error 状态保留 mode（UI 显示失败的是哪个模式），stop 后回 idle
        proc_now = self._proc
        if proc_now is not None and proc_now.poll() is not None:
            self._proc = None

    def tail_summary(self, lines=30):
        with self._lock:
            return self.tail_summary_locked(lines)

    def tail_summary_locked(self, lines=30):
        entries = list(self._session_logs)[-int(lines):]
        return '\n'.join('%s' % entry['line'] for entry in entries)

    # ------------------------------------------------------------------
    # 对外快照 / 日志
    # ------------------------------------------------------------------
    def snapshot(self):
        """返回 §3.1 的 ``session`` 对象。"""
        with self._lock:
            return {
                'mode': self._mode,
                'state': self._state,
                'mock': bool(self._mock),
                'managed': bool(self._managed),
                'external': bool(self._external),
                'available_modes': list(MODES),
                'since': float(self._since),
                'progress': float(self._progress),
                'message': self._message,
                'error': self._error,
                'map': self._map,
            }

    def session_logs(self, limit=200):
        """最近 ``limit`` 行 bringup stdout（``{"ts","line"}``，最多 500 行）。"""
        with self._lock:
            limit = int(limit)
            entries = list(self._session_logs)
            if limit > 0:
                entries = entries[-limit:]
            return [dict(entry) for entry in entries]

    # ------------------------------------------------------------------
    # 退出
    # ------------------------------------------------------------------
    def shutdown(self, timeout_s=30.0):
        """阻塞式停止（进程退出路径用）：SIGINT→…→SIGKILL，超时兜底。"""
        with self._lock:
            managed = self._managed
            state = self._state
        if not managed:
            self.force_kill()
            return
        if state in (STATE_RUNNING, STATE_STARTING, STATE_STOPPING):
            try:
                self.stop()
            except SessionError:
                pass
        deadline = self._clock() + float(timeout_s)
        while self._clock() < deadline:
            with self._lock:
                if self._state in (STATE_IDLE, STATE_ERROR):
                    break
            self.tick()
            time.sleep(0.05)
        self.force_kill()

    def force_kill(self):
        """无条件 SIGKILL 进程组并回收（退出路径兜底）。"""
        with self._lock:
            proc = self._proc
            self._proc = None
            self._mode = MODE_IDLE
            self._map = None
            self._mock = False
            self._pending = None
            self._set_state(STATE_IDLE, IDLE_MESSAGE, 0.0)
        if proc is not None:
            if proc.poll() is None:
                signal_process_group(proc, SIGKILL)
            try:
                proc.wait(timeout=2.0)
            except (OSError, ValueError, subprocess.TimeoutExpired,
                    AttributeError):
                pass
        thread = self._reader_thread
        if thread is not None:
            thread.join(timeout=1.0)
            self._reader_thread = None
