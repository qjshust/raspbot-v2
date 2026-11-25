"""编排层：HTTP 命令 → 状态仓 / 会话状态机 / ROS 运行时。

本模块不导入 rclpy；``runtime`` 是鸭子类型对象（仅 ``bridge`` 注入
:class:`ros_runtime.RosRuntime`，测试注入 FakeRuntime）。HTTP 层只调用这里
暴露的显式方法，每个方法返回 ``(http_status, payload)``；二进制响应的
``map_png``/``map_preview`` 返回 ``(status, headers, body_bytes)``。

Runtime 接口（全部可选，缺失时视为未就绪/空实现）：

.. code-block:: text

    publish_teleop(vx, vy, wz)            publish_zero_twist()
    publish_estop(active)                 publish_buzzer(on)
    publish_led(r, g, b)                  publish_servo(pan, tilt)
    begin_initialpose(x, y, yaw)          begin_activation() -> bool
    cancel_nav_goal()                     send_nav_goal(x, y, yaw, goal_id) -> dict
    has_map_tf() -> bool                  graph() -> dict
    cancel_activation()                   shutdown()

``graph()`` 至少返回 ``nodes``(list)、``topics``(list)、``publishers``(dict
topic→count)、``subscribers``(dict topic→count)。

运行时 → controller 的回调（跨线程、内部加锁）：

``on_nav_goal_feedback`` / ``on_nav_goal_result`` / ``on_nav_goal_rejected`` /
``on_activation_progress`` / ``on_activation_done`` / ``on_initialpose_tf_ready`` /
``sync_estop_from_diagnostics`` / ``on_lifecycle_states``。
"""

from __future__ import annotations

import os
import threading
import time
import uuid

from . import API_VERSION, SERVER_NAME, __version__
from . import mapio, protocol
from . import session as session_mod
from .protocol import ApiError, ErrorCode
from .session import LaunchSpec, SessionError

#: 导航终态（显示用）；进入终态后允许直接发新目标，并在 3 s 后自动回到 ready
NAV_TERMINAL_STATES = ('succeeded', 'failed', 'canceled')

#: 导航状态中文文案（§6.4 网关侧对应）
NAV_MESSAGES = {
    'unavailable': '未进入导航模式',
    'waiting_initialpose': '请在地图上设置初始位姿',
    'activating': '正在激活导航…',
    'ready': '导航就绪，点击地图设置目的地',
    'goal_active': '前往目的地',
    'succeeded': '已到达目的地',
    'failed': '导航失败',
    'canceled': '已取消导航目标',
    'error': '导航激活失败',
}

#: 最终导航状态 → goal_state（设计文档 §3.1 goal_state 取值）
_GOAL_STATE_RESULT = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}


class Controller:
    """有状态编排器；HTTP 线程、ROS 执行器线程、会话监控线程共享。"""

    def __init__(self, config, state, session, runtime=None, *,
                 clock=time.monotonic, wall_clock=time.time):
        self.config = config
        self.state = state
        self.session = session
        self.runtime = runtime
        self._clock = clock
        self._wall_clock = wall_clock
        self._lock = threading.RLock()

        self._png_cache = {}
        self._preview_cache = {}
        self._nav_goal_active = False
        self._nav_goal_id = None
        self._activation_running = False
        self._nav_terminal_since = None
        self._session_grace_until = 0.0
        self._last_session_snapshot = session.snapshot()
        state.set_session(self._last_session_snapshot)

        # 会话管理器回调
        session.set_ready_check(self.check_session_ready)
        session.set_external_check(self.detect_external_stack)
        session.set_infer_mode(self.infer_attach_mode)

    # ------------------------------------------------------------------
    # 基础工具
    # ------------------------------------------------------------------
    @property
    def event_bus(self):
        return self.state.event_bus

    def _now(self):
        return float(self._clock())

    def _now_wall(self):
        return float(self._wall_clock())

    def _runtime_call(self, name, *args, default=None):
        """调用 runtime 的可选方法；异常只记录日志并返回 default。"""
        runtime = self.runtime
        if runtime is None:
            return default
        method = getattr(runtime, name, None)
        if method is None:
            return default
        try:
            return method(*args)
        except Exception as exc:  # pragma: no cover - 运行时防御
            self._log('warn', 'runtime.%s 调用失败: %s' % (name, exc))
            return default

    def _log(self, level, message):
        entry = self.state.add_log(level, message)
        self.event_bus.publish('log', entry)
        return entry

    def log(self, level, message):
        """供 runtime/HTTP 层写网关日志（并发安全）。"""
        return self._log(level, message)

    def _publish_session(self, snapshot):
        self.state.set_session(snapshot)
        self.event_bus.publish('session', {'session': snapshot})

    def _graph(self):
        graph = self._runtime_call('graph', default=None)
        if not isinstance(graph, dict):
            graph = {}
        graph.setdefault('nodes', [])
        graph.setdefault('topics', [])
        graph.setdefault('publishers', {})
        graph.setdefault('subscribers', {})
        return graph

    # ------------------------------------------------------------------
    # 状态快照 / 会话监控
    # ------------------------------------------------------------------
    def state_snapshot(self):
        return self.state.snapshot()

    def health(self):
        now = self._now_wall()
        snapshot = self.state.snapshot(now)
        return 200, {
            'ok': True,
            'name': SERVER_NAME,
            'version': __version__,
            'api': API_VERSION,
            'server_time': now,
            'uptime_s': snapshot['uptime_s'],
        }

    def publish_state_snapshot(self):
        """构建 §3.1 快照并广播 SSE ``state``（由监控线程 10 Hz 调用）。"""
        snapshot = self.state.snapshot()
        self.event_bus.publish('state', {'state': snapshot})
        return snapshot

    def tick_session(self):
        """会话监控（建议 0.5 s 调用）：推进状态机、推送变化、维护 nav 状态。"""
        snapshot = self.session.tick()
        with self._lock:
            previous = self._last_session_snapshot
            if snapshot != previous:
                self._last_session_snapshot = snapshot
                self._publish_session(snapshot)
                self._on_session_changed(previous, snapshot)
            self._maintain_nav(previous, snapshot)
        return snapshot

    def _on_session_changed(self, previous, snapshot):
        old_state = previous.get('state')
        new_state = snapshot.get('state')
        mode = snapshot.get('mode')
        if old_state == new_state and previous.get('mode') == mode:
            return
        if new_state == session_mod.STATE_RUNNING:
            if mode == 'nav':
                self.state.set_nav(
                    state='waiting_initialpose',
                    message=NAV_MESSAGES['waiting_initialpose'],
                    initialpose=None, goal=None, goal_state='idle',
                    distance_remaining=None, navigation_time_s=None,
                    recoveries=None, lifecycle={},
                    activation={'done': 0, 'total': 6, 'phase': ''})
                self._activation_running = False
            else:
                self.state.reset_nav()
            self._log('info', '会话已启动：%s（%s）'
                      % (mode, session_mod.MODE_TEXT.get(mode, mode)))
        elif new_state == session_mod.STATE_STOPPING:
            self._cancel_nav_goal_internal('会话正在停止')
            self.state.reset_nav()
            self._activation_running = False
            self._runtime_call('cancel_activation')
            self._runtime_call('cancel_nav_goal')
            self._safe_stop_motion()
            self._log('info', '会话正在停止…')
        elif new_state == session_mod.STATE_IDLE:
            self._cancel_nav_goal_internal('会话已停止')
            self.state.reset_nav()
            self._activation_running = False
            self._safe_stop_motion()
            self._session_grace_until = self._now() + 3.0
            self._log('info', '会话已停止，回到空闲')
        elif new_state == session_mod.STATE_ERROR:
            self._cancel_nav_goal_internal('会话异常')
            self.state.reset_nav()
            self._activation_running = False
            self._safe_stop_motion()
            error = snapshot.get('error') or '未知原因'
            first_line = str(error).splitlines()[0] if str(error) else '未知原因'
            self._log('error', '会话异常退出：%s' % first_line)
        elif new_state == session_mod.STATE_STARTING:
            self._log('info', '正在启动会话：%s（%s）'
                      % (mode, session_mod.MODE_TEXT.get(mode, mode)))

    def _safe_stop_motion(self):
        """安全默认值：会话停止/异常/退出路径统一停遥控并下发零速（§1.2/§5.2）。"""
        try:
            self.state.stop_teleop()
            self._runtime_call('publish_zero_twist')
        except Exception:  # pragma: no cover - 退出路径防御
            pass

    def _maintain_nav(self, previous, snapshot):
        """导航终态 3 s 后自动回到 ready（便于连续下发目标）。"""
        nav = self.state.nav_snapshot()
        running = (snapshot.get('state') == session_mod.STATE_RUNNING
                   and snapshot.get('mode') == 'nav')
        if not running:
            return
        if nav['state'] in NAV_TERMINAL_STATES:
            if self._nav_terminal_since is None:
                self._nav_terminal_since = self._now()
            elif (self._now() - self._nav_terminal_since >= 3.0
                  and not self._nav_goal_active):
                self._nav_terminal_since = None
                self.state.set_nav(state='ready',
                                   message=NAV_MESSAGES['ready'])
        else:
            self._nav_terminal_since = None

    # ------------------------------------------------------------------
    # 会话就绪 / 外部栈推断（供 SessionManager 回调）
    # ------------------------------------------------------------------
    def check_session_ready(self, mode):
        """§5.2 就绪判据：核心链路 + 模式相关节点/话题。"""
        graph = self._graph()
        nodes = set(graph.get('nodes') or [])
        publishers = graph.get('publishers') or {}
        base_ok = (int(publishers.get('/odom', 0)) > 0
                   and int(publishers.get('/cmd_vel_safe', 0)) > 0)
        if not base_ok:
            return False
        if mode == 'base':
            return True
        map_ok = (int(publishers.get('/map', 0)) > 0
                  and self.state.map_grid() is not None)
        if mode == 'slam':
            return map_ok
        if mode == 'nav':
            return map_ok and ('amcl' in nodes)
        return False

    def detect_external_stack(self):
        """检测外部（systemd）已启动的 bringup 节点；停会话后有 3 s 宽限。"""
        if self._now() < self._session_grace_until:
            return False
        graph = self._graph()
        nodes = set(graph.get('nodes') or [])
        publishers = graph.get('publishers') or {}
        known = {'raspbot_base', 'amcl', 'slam_toolbox', 'bt_navigator',
                 'controller_server', 'ekf_node', 'twist_mux'}
        if nodes & known:
            return True
        return int(publishers.get('/cmd_vel_safe', 0)) > 0

    def infer_attach_mode(self):
        """--attach / --no-session-manage：从 ROS 图推断当前模式（§5.2）。"""
        graph = self._graph()
        nodes = set(graph.get('nodes') or [])
        publishers = graph.get('publishers') or {}
        if 'amcl' in nodes and ('bt_navigator' in nodes
                                or 'controller_server' in nodes):
            return 'nav'
        if 'slam_toolbox' in nodes:
            return 'slam'
        if ('raspbot_base' in nodes
                or int(publishers.get('/cmd_vel_safe', 0)) > 0):
            return 'base'
        return None

    # ------------------------------------------------------------------
    # 会话接口
    # ------------------------------------------------------------------
    def session_snapshot(self):
        return self.session.snapshot()

    def _require_session_running(self, code=ErrorCode.SESSION_NOT_RUNNING):
        snapshot = self.session.snapshot()
        if (snapshot.get('state') != session_mod.STATE_RUNNING
                or snapshot.get('mode') not in session_mod.MODES):
            raise ApiError(
                code,
                '会话未运行（当前 %s/%s），请先启动模式'
                % (snapshot.get('mode'), snapshot.get('state')),
                detail={'session': snapshot})
        return snapshot

    def _require_nav_running(self):
        snapshot = self._require_session_running(ErrorCode.NAV_NOT_READY)
        if snapshot.get('mode') != 'nav':
            raise ApiError(
                ErrorCode.NAV_NOT_READY,
                '当前会话不是导航模式（%s），请先启动导航'
                % snapshot.get('mode'),
                detail={'session': snapshot})
        return snapshot

    def session_start(self, payload):
        body = protocol.require_object(payload)
        mode = protocol.require_mode(body)
        map_path = None
        if mode == 'nav':
            map_path = protocol.validate_map_path(body.get('map'))
        use_mock = protocol.require_bool(body, 'use_mock', required=False,
                                        default=False)
        use_camera = protocol.require_bool(body, 'use_camera', required=False,
                                          default=False)
        use_perception = protocol.require_bool(
            body, 'use_perception', required=False, default=False)
        if self.state.estop_active():
            raise ApiError(ErrorCode.ESTOP_ACTIVE,
                           '急停已锁存，不能启动/切换会话；请先解除急停')
        current = self.session.snapshot()
        if current.get('state') in (session_mod.STATE_STARTING,
                                    session_mod.STATE_STOPPING):
            raise ApiError(ErrorCode.BUSY, '会话正在启动/停止，请稍后重试')
        spec = LaunchSpec(mode=mode, map_path=map_path, use_mock=use_mock,
                          use_camera=use_camera,
                          use_perception=use_perception)
        try:
            snapshot = self.session.start(spec)
        except SessionError as exc:
            raise ApiError(exc.code, exc.message)
        self._last_session_snapshot = snapshot
        self._publish_session(snapshot)
        self._log('info', '请求启动会话：%s（%s）'
                  % (mode, session_mod.MODE_TEXT.get(mode, mode)))
        return 202, {'ok': True, 'session': snapshot}

    def session_stop(self):
        try:
            accepted, snapshot = self.session.stop()
        except SessionError as exc:
            raise ApiError(exc.code, exc.message)
        self._last_session_snapshot = snapshot
        self._publish_session(snapshot)
        if accepted:
            self._log('info', '请求停止会话')
            return 202, {'ok': True, 'session': snapshot}
        return 200, {'ok': True}

    def session_logs(self, query=None):
        limit = protocol.query_int(query or {}, 'limit', 200, 1,
                                   protocol.MAX_SESSION_LOGS)
        return 200, {'logs': self.session.session_logs(limit)}

    # ------------------------------------------------------------------
    # 遥控
    # ------------------------------------------------------------------
    def teleop(self, payload):
        body = protocol.require_object(payload)
        vx = protocol.require_number(body, 'vx')
        vy = protocol.require_number(body, 'vy')
        wz = protocol.require_number(body, 'wz')
        seq = protocol.optional_int(body, 'seq')
        if self.state.estop_active():
            raise ApiError(ErrorCode.ESTOP_ACTIVE,
                           '急停已锁存，遥控被拒绝；请先解除急停')
        self._require_session_running()
        last_seq = self.state.get_teleop_seq()
        if seq is not None and last_seq is not None and seq <= last_seq:
            # 乱序/过期帧：丢弃但不报错（§3.3.4）
            return 200, {'ok': True}
        vx, vy, wz = protocol.clamp_velocity(vx, vy, wz, self.state.limits)
        if self._nav_goal_active:
            self._cancel_nav_goal_internal('遥控接管，先取消导航目标')
        now = self._now_wall()
        self.state.set_teleop(True, vx, vy, wz, last_cmd=now, seq=seq)
        self._runtime_call('publish_teleop', vx, vy, wz)
        return 200, {'ok': True}

    def teleop_stop(self):
        self.state.stop_teleop()
        self._runtime_call('publish_zero_twist')
        return 200, {'ok': True}

    # ------------------------------------------------------------------
    # 急停
    # ------------------------------------------------------------------
    def estop(self, payload):
        body = protocol.require_object(payload)
        active = protocol.require_bool(body, 'active')
        self.state.mark_estop_commanded()
        changed = self.state.set_estop(bool(active))
        if active:
            self.state.stop_teleop()
            self._runtime_call('publish_zero_twist')
            self._runtime_call('publish_estop', True)
            self._cancel_nav_goal_internal('急停')
            if changed:
                self._log('warn', '急停已锁存（/estop=true），'
                                  '会话启动/导航目标/遥控全部禁用')
            snapshot = {'active': True,
                        'since': self.state.estop_snapshot()['since']}
        else:
            self._runtime_call('publish_estop', False)
            if changed:
                self._log('info', '急停已解除（/estop=false）')
            snapshot = {'active': False, 'since': None}
        return 200, {'ok': True, 'estop': snapshot}

    def sync_estop_from_diagnostics(self, active):
        """启动期从 ``/diagnostics`` 的 ``Raspbot Safety: E-Stop`` 同步真实状态。

        只在本网关尚未主动发过 /estop 命令前生效（§3.1 补充规则 / §5.1）。
        """
        if self.state.estop_commanded:
            return False
        changed = self.state.set_estop(bool(active))
        if changed:
            self._log('warn' if active else 'info',
                      '从 /diagnostics 同步急停状态：%s'
                      % ('已急停' if active else '未急停'))
        return changed

    # ------------------------------------------------------------------
    # 导航
    # ------------------------------------------------------------------
    def _goal_is_active(self):
        nav = self.state.nav_snapshot()
        return (self._nav_goal_active
                or nav.get('goal_state') in ('accepted', 'active')
                or nav.get('state') == 'goal_active')

    def _cancel_nav_goal_internal(self, reason=''):
        with self._lock:
            if not self._goal_is_active():
                self._nav_goal_active = False
                return False
            self._runtime_call('cancel_nav_goal')
            self._nav_goal_active = False
            self._nav_goal_id = None
            message = '已取消导航目标'
            if reason:
                message += '（%s）' % reason
            self.state.set_nav(state='canceled', goal_state='canceled',
                               message=message)
            self._nav_terminal_since = self._now()
            self._log('info', '取消导航目标：%s' % (reason or '用户请求'))
            return True

    def initialpose(self, payload):
        body = protocol.require_object(payload)
        x = protocol.require_number(body, 'x')
        y = protocol.require_number(body, 'y')
        yaw = protocol.require_number(body, 'yaw')
        self._require_nav_running()
        if self._goal_is_active():
            self._cancel_nav_goal_internal('重新设置初始位姿')
        now = self._now_wall()
        self.state.set_nav(
            state='activating',
            message='已发布初始位姿，等待 map→base_link 可用…',
            initialpose={'x': x, 'y': y, 'yaw': yaw, 'stamp': now},
            goal=None,
            goal_state='idle',
            distance_remaining=None,
            navigation_time_s=None,
            recoveries=None,
            lifecycle={},
            activation={'done': 0, 'total': 6, 'phase': ''})
        self._activation_running = False
        self._nav_goal_active = False
        # 若上一次激活编排仍在进行，取消它（新初始位姿会重新触发）
        self._runtime_call('cancel_activation')
        self._log('info', '发布初始位姿 (%.2f, %.2f, %.2f rad)'
                  % (x, y, protocol.wrap_angle(yaw)))
        self._runtime_call('begin_initialpose', x, y, yaw)
        if self._runtime_call('has_map_tf', default=False):
            self.on_initialpose_tf_ready()
        return 200, {'ok': True, 'nav': {'state': 'activating'}}

    def nav_activate(self):
        self._require_nav_running()
        if self._activation_running:
            raise ApiError(ErrorCode.BUSY, '导航激活已在进行中')
        self._begin_activation()
        return 202, {'ok': True}

    def _begin_activation(self):
        snapshot = self._require_nav_running()
        if self._activation_running:
            return False
        if snapshot.get('state') != session_mod.STATE_RUNNING:
            raise ApiError(ErrorCode.SESSION_NOT_RUNNING, '会话未运行')
        self._activation_running = True
        self.state.set_nav(
            state='activating',
            message='正在激活导航（0/6）…',
            activation={'done': 0, 'total': 6, 'phase': '准备激活'})
        self._log('info', '开始 Nav2 生命周期激活编排')
        started = self._runtime_call('begin_activation', default=False)
        if started is False:
            self._activation_running = False
            raise ApiError(ErrorCode.ROS_NOT_READY,
                           'Nav2 激活编排未能启动（运行时不可用）')
        return True

    def goal(self, payload):
        body = protocol.require_object(payload)
        x = protocol.require_number(body, 'x')
        y = protocol.require_number(body, 'y')
        yaw_value = body.get('yaw')
        if yaw_value is None:
            robot = self.state.robot_snapshot()
            if robot is not None and robot.get('yaw') is not None:
                yaw = float(robot['yaw'])
            else:
                yaw = 0.0
                self._log('warn', '目标 yaw=null 且无机器人位姿，'
                                  '回退为 0.0 rad')
        else:
            yaw = protocol.require_number(body, 'yaw')
        if self.state.estop_active():
            raise ApiError(ErrorCode.ESTOP_ACTIVE,
                           '急停已锁存，导航目标被拒绝；请先解除急停')
        self._require_nav_running()
        if self._goal_is_active():
            raise ApiError(ErrorCode.GOAL_ACTIVE,
                           '已有活动导航目标，请先取消（POST /api/goal/cancel）')
        nav = self.state.nav_snapshot()
        if nav.get('state') not in ('ready',) + NAV_TERMINAL_STATES:
            raise ApiError(ErrorCode.NAV_NOT_READY,
                           '导航尚未就绪（nav.state=%s）：%s'
                           % (nav.get('state'), nav.get('message', '')))
        goal_id = uuid.uuid4().hex
        result = self._runtime_call('send_nav_goal', x, y, yaw, goal_id,
                                    default=None)
        if not isinstance(result, dict):
            raise ApiError(ErrorCode.ROS_NOT_READY,
                           'ROS 运行时未就绪，无法发送导航目标')
        if not result.get('accepted'):
            reason = result.get('reason') or 'Nav2 拒绝了目标'
            raise ApiError(ErrorCode.NAV_REJECTED,
                           '导航目标被拒绝：%s' % reason)
        now = self._now_wall()
        goal = {'x': x, 'y': y, 'yaw': yaw, 'stamp': now, 'goal_id': goal_id}
        self._nav_goal_active = True
        self._nav_goal_id = goal_id
        self._nav_terminal_since = None
        self.state.set_nav(
            state='goal_active',
            message='前往目的地：剩余 -- m',
            goal=goal,
            goal_state='accepted',
            distance_remaining=None,
            navigation_time_s=0.0,
            recoveries=0)
        self._log('info', '发送导航目标 (%.2f, %.2f, %.2f rad) goal_id=%s'
                  % (x, y, protocol.wrap_angle(yaw), goal_id))
        return 202, {'ok': True, 'goal': goal, 'goal_id': goal_id}

    def goal_cancel(self):
        self._cancel_nav_goal_internal('用户取消')
        return 200, {'ok': True}

    # -- runtime 回调（跨线程） --------------------------------------
    def on_nav_goal_accepted(self, goal_id):
        self._set_goal_state(goal_id, 'accepted')

    def on_nav_goal_active(self, goal_id):
        if goal_id and self._nav_goal_id and goal_id != self._nav_goal_id:
            return
        self._nav_goal_active = True
        self.state.set_nav(state='goal_active', goal_state='active')

    def on_nav_goal_feedback(self, goal_id=None, distance_remaining=None,
                             navigation_time_s=None, recoveries=None,
                             **kwargs):
        if goal_id and self._nav_goal_id and goal_id != self._nav_goal_id:
            return
        updates = {}
        if distance_remaining is not None:
            updates['distance_remaining'] = float(distance_remaining)
        if navigation_time_s is not None:
            updates['navigation_time_s'] = float(navigation_time_s)
        if recoveries is not None:
            updates['recoveries'] = int(recoveries)
        if updates:
            nav = self.state.nav_snapshot()
            distance = updates.get('distance_remaining',
                                   nav.get('distance_remaining'))
            elapsed = updates.get('navigation_time_s',
                                  nav.get('navigation_time_s'))
            message = '前往目的地'
            if distance is not None:
                message += '：剩余 %.2f m' % distance
            if elapsed is not None:
                message += ' / 已用 %.1f s' % elapsed
            updates['message'] = message
            self.state.set_nav(**updates)

    def on_nav_goal_result(self, goal_id=None, status_code=0):
        if goal_id and self._nav_goal_id and goal_id != self._nav_goal_id:
            return
        code = int(status_code or 0)
        goal_state = _GOAL_STATE_RESULT.get(code, 'aborted')
        if code == 4:
            nav_state, message, level = 'succeeded', '已到达目的地', 'info'
            distance = 0.0
        elif code == 5:
            nav_state, message, level = 'canceled', '导航已取消', 'info'
            distance = None
        else:
            nav_state, message, level = 'failed', '导航失败（status=%d）' % code, \
                'error'
            distance = None
        self._nav_goal_active = False
        self._nav_goal_id = None
        self._nav_terminal_since = self._now()
        self.state.set_nav(state=nav_state, goal_state=goal_state,
                           message=message,
                           distance_remaining=distance)
        self._log(level, '导航目标结束：%s（status=%d）' % (message, code))

    def on_nav_goal_rejected(self, reason='', goal_id=None):
        self._nav_goal_active = False
        self._nav_goal_id = None
        self.state.set_nav(state='failed', goal_state='aborted',
                           message='导航目标被拒绝：%s' % (reason or '未知原因'))
        self._log('error', '导航目标被拒绝：%s' % (reason or '未知原因'))

    def _set_goal_state(self, goal_id, goal_state):
        if goal_id and self._nav_goal_id and goal_id != self._nav_goal_id:
            return
        self.state.set_nav(goal_state=goal_state, state='goal_active')

    def on_initialpose_tf_ready(self):
        """TF ``map→base_link`` 已可用：自动进入 Nav2 激活编排（§5.3）。"""
        with self._lock:
            snapshot = self.session.snapshot()
            if (snapshot.get('state') != session_mod.STATE_RUNNING
                    or snapshot.get('mode') != 'nav'):
                return
            nav = self.state.nav_snapshot()
            if nav.get('state') not in ('activating', 'waiting_initialpose'):
                return
            self._log('info', 'map→base_link 已可用，开始激活 Nav2')
            try:
                self._begin_activation()
            except ApiError as exc:
                self.state.set_nav(state='error', message=exc.message,
                                   goal_state='idle')
                self._log('error', 'Nav2 激活失败：%s' % exc.message)

    def on_activation_progress(self, done=0, total=6, phase='',
                               lifecycle=None):
        with self._lock:
            self._activation_running = True
            updates = {
                'state': 'activating',
                'message': '正在激活导航（%d/%d）：%s'
                           % (int(done), int(total), phase or ''),
                'activation': {'done': int(done), 'total': int(total),
                               'phase': str(phase or '')},
            }
            if lifecycle:
                updates['lifecycle'] = dict(lifecycle)
            self.state.set_nav(**updates)

    def on_activation_done(self, success, message='', lifecycle=None):
        with self._lock:
            self._activation_running = False
            self._nav_terminal_since = None
            updates = {'lifecycle': dict(lifecycle)} if lifecycle else {}
            if success:
                updates.update(state='ready', message=NAV_MESSAGES['ready'])
                self._log('info', 'Nav2 激活完成，导航就绪')
            else:
                reason = message or '未知原因'
                updates.update(state='error',
                               message='导航激活失败：%s' % reason)
                self._log('error', 'Nav2 激活失败：%s' % reason)
            self.state.set_nav(**updates)

    def on_lifecycle_states(self, states):
        if isinstance(states, dict) and states:
            self.state.set_nav(lifecycle=dict(states))

    # ------------------------------------------------------------------
    # 地图
    # ------------------------------------------------------------------
    def map_meta(self):
        return 200, self.state.map_snapshot()

    def map_png(self, version=None):
        grid = self.state.map_grid()
        if grid is None:
            raise ApiError(ErrorCode.NO_MAP, '尚无 /map 地图', status=404)
        current = self.state.map_version()
        requested = None
        try:
            if version not in (None, ''):
                requested = int(str(version))
        except (TypeError, ValueError):
            requested = None
        with self._lock:
            png = self._png_cache.get(current)
            if png is None:
                png = grid.to_png()
                self._png_cache[current] = png
                while len(self._png_cache) > int(self.config.map_png_cache_size):
                    oldest = next(iter(self._png_cache))
                    self._png_cache.pop(oldest, None)
        cache_control = ('public, max-age=31536000, immutable'
                         if requested == current else 'no-cache')
        headers = {
            'Content-Type': 'image/png',
            'Cache-Control': cache_control,
            'ETag': '"map-v%d"' % current,
        }
        return 200, headers, png

    def maps(self):
        entries = mapio.list_maps(self.config.maps_dirs)
        dirs = mapio.existing_map_dirs(self.config.maps_dirs)
        return 200, {'maps': entries, 'dirs': dirs}

    def map_preview(self, name):
        if not mapio.map_name_valid(name):
            raise ApiError(ErrorCode.NOT_FOUND, '地图不存在：%r' % (name,),
                           status=404)
        entry = None
        for candidate in mapio.list_maps(self.config.maps_dirs):
            if candidate.get('name') == name:
                entry = candidate
                break
        if entry is None or not entry.get('valid') or not entry.get('pgm'):
            raise ApiError(ErrorCode.NOT_FOUND, '地图不存在或无有效预览：%s'
                           % name, status=404)
        pgm_path = entry['pgm']
        try:
            mtime = float(os.path.getmtime(pgm_path))
        except OSError:
            raise ApiError(ErrorCode.NOT_FOUND, '地图预览不可用：%s' % name,
                           status=404)
        with self._lock:
            cached = self._preview_cache.get(name)
            if cached is not None and cached[0] == mtime:
                png = cached[1]
            else:
                try:
                    png = mapio.pgm_file_to_png(pgm_path)
                except mapio.MapIOError as exc:
                    raise ApiError(ErrorCode.NOT_FOUND,
                                   '地图预览解析失败：%s' % exc, status=404)
                self._preview_cache[name] = (mtime, png)
        return 200, {'Content-Type': 'image/png',
                     'Cache-Control': 'public, max-age=60'}, png

    def map_save(self, payload):
        body = protocol.require_object(payload)
        name = protocol.validate_map_name(body.get('name'))
        overwrite = protocol.require_bool(body, 'overwrite', required=False,
                                          default=False)
        snapshot = self.session.snapshot()
        if (snapshot.get('state') != session_mod.STATE_RUNNING
                or snapshot.get('mode') not in session_mod.MODES):
            raise ApiError(ErrorCode.SESSION_IDLE,
                           '会话未运行，无法保存地图（先启动 base/slam/nav 之一）')
        grid = self.state.map_grid()
        if grid is None:
            raise ApiError(ErrorCode.NO_MAP, '尚未收到 /map，无地图可保存')
        directory = mapio.choose_save_dir(self.config.maps_dir,
                                          self.config.maps_fallback)
        try:
            result = mapio.save_map(directory, name, grid, overwrite=overwrite)
        except mapio.MapExistsError:
            raise ApiError(ErrorCode.MAP_EXISTS,
                           '同名地图已存在：%s（overwrite=true 可覆盖）' % name)
        except mapio.MapIOError as exc:
            raise ApiError(ErrorCode.INTERNAL, '保存地图失败：%s' % exc,
                           status=500)
        self._log('info', '地图已保存：%s（%dx%d @ %.3f m）'
                  % (name, grid.width, grid.height, grid.resolution))
        return 200, result

    # ------------------------------------------------------------------
    # 外设
    # ------------------------------------------------------------------
    def actuator_buzzer(self, payload):
        body = protocol.require_object(payload)
        on = protocol.require_bool(body, 'on')
        self._require_session_running()
        self.state.set_peripherals(buzzer=on)
        self._runtime_call('publish_buzzer', on)
        return 200, {'ok': True}

    def actuator_led(self, payload):
        body = protocol.require_object(payload)
        r = protocol.require_number(body, 'r', low=0.0, high=1.0)
        g = protocol.require_number(body, 'g', low=0.0, high=1.0)
        b = protocol.require_number(body, 'b', low=0.0, high=1.0)
        self._require_session_running()
        self.state.set_peripherals(led={'r': r, 'g': g, 'b': b})
        self._runtime_call('publish_led', r, g, b)
        return 200, {'ok': True}

    def actuator_servo(self, payload):
        body = protocol.require_object(payload)
        pan = protocol.require_number(body, 'pan', low=0.0, high=180.0)
        tilt = protocol.require_number(body, 'tilt', low=0.0, high=110.0)
        self._require_session_running()
        self.state.set_peripherals(servo={'pan': pan, 'tilt': tilt})
        self._runtime_call('publish_servo', pan, tilt)
        return 200, {'ok': True}

    # ------------------------------------------------------------------
    # 日志 / ROS 调试 / 退出
    # ------------------------------------------------------------------
    def logs(self, query=None):
        limit = protocol.query_int(query or {}, 'limit', 200, 1,
                                   protocol.MAX_GATEWAY_LOGS)
        return 200, {'logs': self.state.log_entries(limit)}

    def ros_info(self):
        graph = self._graph()
        nodes = sorted(str(name) for name in (graph.get('nodes') or []))
        topics = []
        for item in graph.get('topics') or []:
            if isinstance(item, dict):
                topics.append({'name': str(item.get('name', '')),
                               'types': list(item.get('types') or [])})
            else:
                topics.append({'name': str(item), 'types': []})
        topics.sort(key=lambda item: item['name'])
        return 200, {'nodes': nodes, 'topics': topics}

    def node_names(self):
        return list(self._graph().get('nodes') or [])

    def shutdown(self):
        """退出前安全收尾：停遥控并置零速（会话/HTTP 由 bridge 关闭）。"""
        self.state.stop_teleop()
        self._runtime_call('publish_zero_twist')
        self._runtime_call('cancel_nav_goal')
        self._runtime_call('cancel_activation')
        self._log('info', '网关正在退出：遥控已停止、零速已下发')
