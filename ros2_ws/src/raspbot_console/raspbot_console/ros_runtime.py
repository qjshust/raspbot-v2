"""ROS 2 适配层：订阅、TF、Nav2 action、lifecycle 激活、发布（唯一 rclpy 层）。

职责（设计文档 §5.1/§5.3/§5.4）：

- 订阅 ``/map``(transient_local) ``/scan``(sensor_data) ``/odom`` ``/plan``
  ``/diagnostics``；TF 用 ``tf2_ros.Buffer + TransformListener`` 查
  ``map→base_link`` / ``odom→base_link``（10 Hz），**绝不发布 TF**；
- 只发布：``/cmd_vel_teleop``（20 Hz + 500 ms 看门狗）、``/estop``、
  ``/initialpose``、``/buzzer``、``/led``、``/servo_pan``、``/servo_tilt``；
- Nav2 ``NavigateToPose`` action 客户端（feedback/result 字段运行时自省，
  兼容版本差异）+ lifecycle 激活编排（等价 ``.dsh/selftest/activate_nav2.sh``）；
- 事件推送：``scan``(5 Hz)、``path``(2 Hz 节流)、``map``(version 变化)；
- 状态仓由 :class:`raspbot_console.state.StateStore` 承载，本层只写数据。

线程模型：``MultiThreadedExecutor`` 独立线程执行所有 ROS 回调；HTTP 线程可
直接调用发布方法（rclpy 允许跨线程 publish）。lifecycle 编排使用 **非阻塞
状态机**（timer 轮询 + ``call_async``），不会阻塞执行器。
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State as LifecycleStateMsg
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA, Float32

from . import mapio, protocol

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

#: lifecycle 状态/迁移 ID；用 getattr 兼容不同发行版常量名
TRANSITION_CONFIGURE = getattr(Transition, 'TRANSITION_CONFIGURE', 1)
TRANSITION_ACTIVATE = getattr(Transition, 'TRANSITION_ACTIVATE', 3)
TRANSITION_SHUTDOWN = getattr(Transition, 'TRANSITION_SHUTDOWN', 5)

STATE_UNCONFIGURED = getattr(LifecycleStateMsg, 'PRIMARY_STATE_UNCONFIGURED',
                             1)
STATE_INACTIVE = getattr(LifecycleStateMsg, 'PRIMARY_STATE_INACTIVE', 2)
STATE_ACTIVE = getattr(LifecycleStateMsg, 'PRIMARY_STATE_ACTIVE', 3)
STATE_FINALIZED = getattr(LifecycleStateMsg, 'PRIMARY_STATE_FINALIZED', 4)

LIFECYCLE_STATE_NAMES = {0: 'unknown', 1: 'unconfigured', 2: 'inactive',
                         3: 'active', 4: 'finalized'}

#: lifecycle manager 节点名（nav2.launch.py 中固定）
MANAGER_NODE = 'lifecycle_manager_nav2'

#: Nav2 需要按顺序激活的节点（与 nav2.launch.py 的 node_names 一致）
NAV2_NODES = ('controller_server', 'planner_server', 'behavior_server',
              'bt_navigator', 'collision_monitor')

#: §3.3.7 规定的 /initialpose 协方差
INITIALPOSE_COVARIANCE = [
    0.25, 0, 0, 0, 0, 0,
    0, 0.25, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0.0685,
]

#: 诊断里急停条目的名字（safety_node.cpp）
ESTOP_DIAG_NAME = 'Raspbot Safety: E-Stop'

#: 导航目标状态码（action_msgs/GoalStatus 语义，§5.3）
GOAL_STATUS_SUCCEEDED = 4
GOAL_STATUS_CANCELED = 5
GOAL_STATUS_ABORTED = 6


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------

def _stamp_to_float(stamp):
    """builtin_interfaces/Time → float unix 秒。"""
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return time.time()


def _yaw_from_quaternion(quaternion):
    return protocol.quaternion_to_yaw(float(quaternion.x), float(quaternion.y),
                                      float(quaternion.z), float(quaternion.w))


def _action_ready(client):
    """非阻塞判断 action server 是否就绪（兼容 rclpy 版本差异）。"""
    if client is None:
        return False
    try:
        return bool(client.server_is_ready())
    except AttributeError:
        pass
    except Exception:
        return False
    try:
        return bool(client.wait_for_server(timeout_sec=0.0))
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Nav2 lifecycle 激活编排（非阻塞状态机）
# ---------------------------------------------------------------------------

class _NavActivation:
    """§5.3 的等价实现：manager shutdown → 5 个节点 configure/activate。

    顺序与 ``.dsh/selftest/activate_nav2.sh`` 一致：controller_server →
    planner_server → 等 /compute_path_to_pose → behavior_server → bt_navigator
    → 等 /navigate_to_pose → collision_monitor。激活用 lifecycle 服务，
    每步 configure 15 s / activate 30 s 超时，失败最多重试 3 次。
    """

    STEPS = (
        ('manager_shutdown', '关闭 lifecycle_manager_nav2（best-effort）'),
        ('controller_server', '激活 controller_server'),
        ('planner_server', '激活 planner_server'),
        ('wait_planner', '等待 /compute_path_to_pose action'),
        ('behavior_server', '激活 behavior_server'),
        ('bt_navigator', '激活 bt_navigator'),
        ('wait_navigator', '等待 /navigate_to_pose action'),
        ('collision_monitor', '激活 collision_monitor'),
    )
    TOTAL = 6  # 进度条总步数：lifecycle_manager + 5 个 Nav2 节点

    CONFIGURE_TIMEOUT = 15.0
    ACTIVATE_TIMEOUT = 30.0
    QUERY_TIMEOUT = 10.0
    WAIT_PLANNER_TIMEOUT = 10.0
    WAIT_NAVIGATOR_TIMEOUT = 15.0
    MANAGER_TIMEOUT = 5.0
    RETRY_LIMIT = 3
    RETRY_DELAY = 8.0

    def __init__(self, runtime):
        self.runtime = runtime
        self.index = 0
        self.done = 0
        self.finished = False
        self.success = False
        self.message = ''
        self.phase = self.STEPS[0][1]
        self._lifecycle = {}
        self._stage = 'enter'
        self._deadline = 0.0
        self._retry_at = 0.0
        self._attempt = 0
        self._future = None
        self._future_kind = None
        self._clients = {}
        self._started_at = runtime.now_mono()

    # -- 公共 -----------------------------------------------------------
    def start(self):
        self._report()

    def tick(self, now):
        if self.finished or now < self._retry_at:
            return
        try:
            self._advance(now)
        except Exception as exc:  # pragma: no cover - 运行时防御
            self._fail('激活编排异常: %r' % exc)

    # -- 步骤分发 -------------------------------------------------------
    def _advance(self, now):
        step = self.STEPS[self.index][0]
        if step == 'manager_shutdown':
            self._step_manager(now)
        elif step.startswith('wait_'):
            self._step_wait(now, step)
        else:
            self._step_node(now, step)

    def _next_step(self):
        self.index += 1
        self._stage = 'enter'
        self._attempt = 0
        self._future = None
        self._future_kind = None
        self._retry_at = 0.0
        if self.index >= len(self.STEPS):
            self._complete()
            return
        self.phase = self.STEPS[self.index][1]
        self._report()

    def _report(self, phase=None):
        if phase is not None:
            self.phase = phase
        self.runtime.controller.on_activation_progress(
            done=self.done, total=self.TOTAL, phase=self.phase,
            lifecycle=dict(self._lifecycle))

    def _log(self, level, message):
        self.runtime.controller.log(level, message)

    def _complete(self):
        if self.finished:
            return
        if not self.runtime.navigator_action_ready():
            self._fail('/navigate_to_pose action server 未就绪')
            return
        self.finished = True
        self.success = True
        self.phase = '激活完成'
        self.runtime.controller.on_activation_done(
            True, '', dict(self._lifecycle))

    def _fail(self, message):
        if self.finished:
            return
        self.finished = True
        self.success = False
        self.message = str(message)
        self._log('error', 'Nav2 激活失败：%s' % message)
        self.runtime.controller.on_activation_done(
            False, str(message), dict(self._lifecycle))

    # -- 步骤实现 -------------------------------------------------------
    def _step_manager(self, now):
        if self._stage == 'enter':
            nodes = set(self.runtime.node_names())
            if MANAGER_NODE not in nodes:
                self._lifecycle[MANAGER_NODE] = 'absent'
                self.done += 1
                self._next_step()
                return
            client = self.runtime.change_state_client(MANAGER_NODE)
            if not client.service_is_ready():
                if now - self._started_at > self.MANAGER_TIMEOUT:
                    # best-effort：manager 可能刚退出，直接继续
                    self._lifecycle[MANAGER_NODE] = 'unknown'
                    self.done += 1
                    self._next_step()
                return
            self._future = client.call_async(
                self.runtime.change_state_request(TRANSITION_SHUTDOWN))
            self._future_kind = 'manager'
            self._stage = 'wait'
            self._deadline = now + self.MANAGER_TIMEOUT
            return
        if self._future is None:
            self._stage = 'enter'
            return
        if not self._future.done():
            if now > self._deadline:
                self._future = None
                self._lifecycle[MANAGER_NODE] = 'shutdown-timeout'
                self.done += 1
                self._next_step()
            return
        self._future = None
        self._lifecycle[MANAGER_NODE] = 'shutdown'
        self.done += 1
        self._log('info', '已 best-effort 关闭 lifecycle_manager_nav2')
        self._next_step()

    def _step_wait(self, now, step):
        if self._stage == 'enter':
            self._stage = 'wait'
            self._deadline = now + (
                self.WAIT_PLANNER_TIMEOUT if step == 'wait_planner'
                else self.WAIT_NAVIGATOR_TIMEOUT)
        if step == 'wait_planner':
            if self.runtime.planner_action_ready():
                self._next_step()
                return
            if now > self._deadline:
                self._fail('%.0f s 内未发现 /compute_path_to_pose action server'
                           % self.WAIT_PLANNER_TIMEOUT)
            return
        if self.runtime.navigator_action_ready():
            self._next_step()
            return
        if now > self._deadline:
            self._fail('%.0f s 内未发现 /navigate_to_pose action server'
                       % self.WAIT_NAVIGATOR_TIMEOUT)

    def _step_node(self, now, name):
        clients = self._clients.get(name)
        if clients is None:
            clients = {
                'get': self.runtime.get_state_client(name),
                'change': self.runtime.change_state_client(name),
            }
            self._clients[name] = clients
        if self._stage == 'enter':
            self._stage = 'query'
            self._deadline = now + self.QUERY_TIMEOUT
            self._report(phase='查询 %s 状态' % name)

        if self._future is not None:
            if not self._future.done():
                if now > self._deadline:
                    self._future = None
                    self._op_failed(name, 'lifecycle 服务调用超时')
                return
            future = self._future
            kind = self._future_kind
            self._future = None
            self._future_kind = None
            try:
                response = future.result()
            except Exception as exc:
                self._op_failed(name, 'lifecycle 服务调用失败: %r' % exc)
                return
            if kind == 'query':
                self._on_query_result(name, response, now)
            else:
                self._on_change_result(name, kind, response, now)
            return

        if self._stage == 'query':
            client = clients['get']
            if not client.service_is_ready():
                if now > self._deadline:
                    self._op_failed(name, 'lifecycle 服务未就绪（/%s/get_state）'
                                    % name)
                return
            self._future = client.call_async(GetState.Request())
            self._future_kind = 'query'
            self._deadline = now + self.QUERY_TIMEOUT
            return

        if self._stage in ('configure', 'activate'):
            client = clients['change']
            if not client.service_is_ready():
                if now > self._deadline:
                    self._op_failed(name, 'change_state 服务未就绪')
                return
            transition = (TRANSITION_CONFIGURE if self._stage == 'configure'
                          else TRANSITION_ACTIVATE)
            self._future = client.call_async(
                self.runtime.change_state_request(transition))
            self._future_kind = self._stage
            self._deadline = now + (
                self.CONFIGURE_TIMEOUT if self._stage == 'configure'
                else self.ACTIVATE_TIMEOUT)
            return

    def _on_query_result(self, name, response, now):
        state_id = 0
        try:
            state_id = int(response.current_state.id)
        except (AttributeError, TypeError, ValueError):
            state_id = 0
        self._lifecycle[name] = LIFECYCLE_STATE_NAMES.get(state_id, 'unknown')
        if state_id == STATE_ACTIVE:
            self._node_done(name)
            return
        if state_id == STATE_UNCONFIGURED:
            self._stage = 'configure'
            self._deadline = now + self.CONFIGURE_TIMEOUT
            self._report(phase='配置 %s' % name)
            return
        if state_id == STATE_INACTIVE:
            self._stage = 'activate'
            self._deadline = now + self.ACTIVATE_TIMEOUT
            self._report(phase='激活 %s' % name)
            return
        if state_id == STATE_FINALIZED:
            self._fail('/%s 处于 finalized 状态，无法激活（需重启会话）' % name)
            return
        # unknown / 迁移中：1 s 后重新查询
        self._retry_at = now + 1.0
        self._stage = 'query'
        self._deadline = now + self.QUERY_TIMEOUT

    def _on_change_result(self, name, kind, response, now):
        success = bool(getattr(response, 'success', False))
        if success:
            self._lifecycle[name] = ('inactive' if kind == 'configure'
                                     else 'active')
            # 复核状态，最终以 get_state 为准
            self._stage = 'query'
            self._deadline = now + self.QUERY_TIMEOUT
            return
        self._op_failed(name, '%s 返回 success=false' % kind)

    def _node_done(self, name):
        self.done = min(self.TOTAL, self.done + 1)
        self._log('info', '/%s 已激活（%d/%d）' % (name, self.done, self.TOTAL))
        self._next_step()

    def _op_failed(self, name, reason):
        self._attempt += 1
        if self._attempt >= self.RETRY_LIMIT:
            self._fail('/%s 激活失败（%s），已重试 %d 次'
                       % (name, reason, self._attempt))
            return
        self._retry_at = self.runtime.now_mono() + self.RETRY_DELAY
        self._stage = 'query'
        self._log('warn', '/%s 激活未成功（%s），%.0f s 后重试（%d/%d）'
                  % (name, reason, self.RETRY_DELAY, self._attempt,
                     self.RETRY_LIMIT))


# ---------------------------------------------------------------------------
# ROS 运行时
# ---------------------------------------------------------------------------

class RosRuntime:
    """rclpy 节点与全部 ROS 交互；由 ``bridge`` 创建并 ``start()``。"""

    def __init__(self, config, state, controller):
        self.config = config
        self.state = state
        self.controller = controller
        self._lock = threading.RLock()
        self._started_mono = time.monotonic()
        self._shutdown = threading.Event()
        self._warn_times = {}
        self.node = Node('raspbot_console')

        # -- TF：只读，绝不发布 -------------------------------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        try:
            # 兼容不同 tf2_ros 版本：显式关闭 spin_thread（节点由本模块的
            # MultiThreadedExecutor 统一 spin，避免重复 spin 同一 node）
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self.node, spin_thread=False)
        except TypeError:  # pragma: no cover - 旧版 tf2_ros 无该关键字
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self.node)

        # -- 发布器（可靠 QoS，设计文档 §3.3.4/§5.1） ----------------
        reliable_10 = QoSProfile(
            depth=10, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE)
        reliable_1 = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE)
        self.pub_cmd_vel_teleop = self.node.create_publisher(
            Twist, '/cmd_vel_teleop', reliable_10)
        self.pub_estop = self.node.create_publisher(Bool, '/estop', reliable_1)
        self.pub_initialpose = self.node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', reliable_10)
        self.pub_buzzer = self.node.create_publisher(Bool, '/buzzer',
                                                     reliable_10)
        self.pub_led = self.node.create_publisher(ColorRGBA, '/led',
                                                  reliable_10)
        self.pub_servo_pan = self.node.create_publisher(Float32, '/servo_pan',
                                                        reliable_10)
        self.pub_servo_tilt = self.node.create_publisher(
            Float32, '/servo_tilt', reliable_10)

        # -- 订阅器 ------------------------------------------------
        map_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, '/map', self._on_map,
                                      map_qos)
        self.node.create_subscription(LaserScan, '/scan', self._on_scan,
                                      qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.node.create_subscription(Path, '/plan', self._on_plan, 10)
        self.node.create_subscription(DiagnosticArray, '/diagnostics',
                                      self._on_diagnostics, 10)

        # -- action 客户端 ------------------------------------------
        self._nav_client = ActionClient(self.node, NavigateToPose,
                                        '/navigate_to_pose')
        self._planner_client = ActionClient(self.node, ComputePathToPose,
                                            '/compute_path_to_pose')

        # -- 服务客户端（lifecycle） --------------------------------
        self._service_clients = {}
        self._goal_handle = None
        self._goal_id = None

        # -- 数据缓存 ----------------------------------------------
        self._last_odom = None
        self._last_scan = None
        self._last_plan = None
        self._plan_dirty = False
        self._initialpose = None
        self._activation = None
        self._map_version_logged = 0
        self._teleop_expired_logged = False

        # -- 定时器 ------------------------------------------------
        self._timer_robot = self._timer(1.0 / 10.0, self._tick_robot)
        self._timer_scan = self._timer(
            1.0 / max(0.1, float(config.scan_rate_hz)), self._tick_scan)
        self._timer_path = self._timer(
            1.0 / max(0.1, float(config.path_rate_hz)), self._tick_path)
        self._timer_teleop = self._timer(
            1.0 / max(1.0, float(config.teleop_rate_hz)), self._tick_teleop)
        self._timer_estop = self._timer(
            max(0.1, float(config.estop_resend_s)), self._tick_estop)
        self._timer_graph = self._timer(1.0, self._tick_graph)
        self._timer_initialpose = self._timer(
            1.0 / max(0.1, float(config.initialpose_rate_hz)),
            self._tick_initialpose)
        self._timer_activation = self._timer(0.1, self._tick_activation)

        self._executor = None
        self._executor_thread = None

        self.controller.log('info', 'ROS 运行时已创建：节点 raspbot_console')

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def start(self):
        """启动 MultiThreadedExecutor 线程（rclpy.init 由 bridge 负责）。"""
        if self._executor is not None:
            return self
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self.node)
        self._executor_thread = threading.Thread(
            target=self._spin_executor, name='ros-executor', daemon=True)
        self._executor_thread.start()
        return self

    def _spin_executor(self):
        try:
            self._executor.spin()
        except Exception:  # pragma: no cover - 退出路径
            pass

    def stop(self):
        """停止执行器并销毁节点（rclpy.shutdown 由 bridge 负责）。"""
        if self._shutdown.is_set():
            return
        self._shutdown.set()
        executor = self._executor
        self._executor = None
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=5.0)
            except TypeError:  # pragma: no cover - 旧版本 rclpy
                try:
                    executor.shutdown()
                except Exception:
                    pass
            except Exception:  # pragma: no cover
                pass
        thread = self._executor_thread
        self._executor_thread = None
        if thread is not None:
            thread.join(timeout=5.0)
        try:
            self.node.destroy_node()
        except Exception:  # pragma: no cover
            pass

    # ------------------------------------------------------------------
    # 帮助方法
    # ------------------------------------------------------------------
    def _timer(self, period_s, callback):
        return self.node.create_timer(max(0.001, float(period_s)), callback)

    def now_mono(self):
        return time.monotonic()

    def _log_warn_once(self, key, message, interval_s=10.0):
        now = time.monotonic()
        if now - self._warn_times.get(key, 0.0) < interval_s:
            return
        self._warn_times[key] = now
        self.controller.log('warn', message)

    def _lookup(self, parent, child):
        try:
            return self.tf_buffer.lookup_transform(parent, child, Time())
        except Exception:
            return None

    def has_map_tf(self):
        return self._lookup('map', 'base_link') is not None

    def node_names(self):
        try:
            return list(self.node.get_node_names())
        except Exception:  # pragma: no cover
            return []

    def _action_ready(self, client):
        return _action_ready(client)

    def navigator_action_ready(self):
        return _action_ready(self._nav_client)

    def planner_action_ready(self):
        return _action_ready(self._planner_client)

    def _service_client(self, kind, node_name):
        key = (kind, node_name)
        with self._lock:
            client = self._service_clients.get(key)
            if client is None:
                if kind == 'get':
                    client = self.node.create_client(
                        GetState, '/%s/get_state' % node_name)
                else:
                    client = self.node.create_client(
                        ChangeState, '/%s/change_state' % node_name)
                self._service_clients[key] = client
            return client

    def get_state_client(self, node_name):
        return self._service_client('get', node_name)

    def change_state_client(self, node_name):
        return self._service_client('change', node_name)

    def change_state_request(self, transition_id):
        request = ChangeState.Request()
        try:
            request.transition.id = int(transition_id)
        except (AttributeError, TypeError):  # pragma: no cover - 消息差异
            pass
        return request

    def graph(self):
        """节点/话题/发布者/订阅者快照（``/api/ros`` 与就绪判据用）。"""
        node = self.node
        try:
            nodes = list(node.get_node_names())
        except Exception:  # pragma: no cover
            nodes = []
        topics = []
        try:
            for name, types in node.get_topic_names_and_types():
                topics.append({'name': str(name),
                               'types': [str(t) for t in types]})
        except Exception:  # pragma: no cover
            topics = []
        publishers = {}
        for topic in ('/odom', '/cmd_vel_safe', '/map', '/scan', '/plan',
                      '/diagnostics', '/cmd_vel', '/cmd_vel_nav',
                      '/cmd_vel_teleop'):
            try:
                publishers[topic] = int(node.count_publishers(topic))
            except Exception:  # pragma: no cover
                publishers[topic] = 0
        subscribers = {}
        for topic in ('/cmd_vel_teleop', '/cmd_vel_nav', '/cmd_vel_safe',
                      '/estop', '/buzzer', '/led', '/servo_pan',
                      '/servo_tilt', '/initialpose'):
            try:
                subscribers[topic] = int(node.count_subscribers(topic))
            except Exception:  # pragma: no cover
                subscribers[topic] = 0
        return {'nodes': nodes, 'topics': topics,
                'publishers': publishers, 'subscribers': subscribers}

    # ------------------------------------------------------------------
    # 发布接口（controller 调用，任意线程）
    # ------------------------------------------------------------------
    def _publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.pub_cmd_vel_teleop.publish(msg)

    def publish_teleop(self, vx, vy, wz):
        self._publish_twist(vx, vy, wz)

    def publish_zero_twist(self):
        self._publish_twist(0.0, 0.0, 0.0)

    def publish_estop(self, active):
        msg = Bool()
        msg.data = bool(active)
        self.pub_estop.publish(msg)

    def publish_buzzer(self, on):
        msg = Bool()
        msg.data = bool(on)
        self.pub_buzzer.publish(msg)

    def publish_led(self, r, g, b):
        msg = ColorRGBA()
        msg.r = float(r)
        msg.g = float(g)
        msg.b = float(b)
        msg.a = 1.0
        self.pub_led.publish(msg)

    def publish_servo(self, pan, tilt):
        pan_msg = Float32()
        pan_msg.data = float(pan)
        self.pub_servo_pan.publish(pan_msg)
        tilt_msg = Float32()
        tilt_msg.data = float(tilt)
        self.pub_servo_tilt.publish(tilt_msg)

    def _initialpose_msg(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        q = protocol.yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance = list(INITIALPOSE_COVARIANCE)
        return msg

    def publish_initialpose(self, x, y, yaw):
        self.pub_initialpose.publish(self._initialpose_msg(x, y, yaw))

    def begin_initialpose(self, x, y, yaw):
        """§3.3.7：2 Hz 重复发布最多 20 s，直到 TF map→base_link 可用。"""
        until = self.now_mono() + float(self.config.initialpose_timeout_s)
        with self._lock:
            self._initialpose = {'x': float(x), 'y': float(y),
                                 'yaw': float(yaw), 'until': until,
                                 'notified': False}
        self.publish_initialpose(x, y, yaw)
        self.controller.log('info', '开始 2 Hz 重发 /initialpose（最多 %.0f s）'
                            % self.config.initialpose_timeout_s)

    def robot_yaw(self):
        robot = self.state.robot_snapshot()
        if robot is None:
            return None
        return robot.get('yaw')

    # ------------------------------------------------------------------
    # Nav2 action
    # ------------------------------------------------------------------
    def send_nav_goal(self, x, y, yaw, goal_id=None):
        """构造 NavigateToPose 并异步发送；同步等待「是否被接受」。

        返回 ``{'accepted': bool, 'goal_id': str, 'reason': str|None}``。
        """
        if not self.navigator_action_ready():
            return {'accepted': False, 'goal_id': goal_id,
                    'reason': '/navigate_to_pose action server 未就绪'}
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        q = protocol.yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]
        # 兼容不同 Nav2 版本的 behavior_tree 字段
        try:
            fields = goal.get_fields_and_field_types()
        except Exception:  # pragma: no cover
            fields = {}
        if 'behavior_tree' in fields and not getattr(goal, 'behavior_tree', ''):
            try:
                goal.behavior_tree = ''
            except Exception:  # pragma: no cover
                pass

        try:
            future = self._nav_client.send_goal_async(
                goal, feedback_callback=self._on_goal_feedback)
        except Exception as exc:
            return {'accepted': False, 'goal_id': goal_id,
                    'reason': 'send_goal_async 失败: %r' % exc}
        event = threading.Event()
        try:
            future.add_done_callback(lambda _future: event.set())
        except Exception:  # pragma: no cover
            pass
        if not event.wait(timeout=5.0):
            return {'accepted': False, 'goal_id': goal_id,
                    'reason': '等待 Nav2 接受目标超时（5 s）'}
        try:
            goal_handle = future.result()
        except Exception as exc:
            return {'accepted': False, 'goal_id': goal_id,
                    'reason': '目标响应异常: %r' % exc}
        if goal_handle is None or not getattr(goal_handle, 'accepted', False):
            return {'accepted': False, 'goal_id': goal_id,
                    'reason': 'Nav2 拒绝了目标'}
        with self._lock:
            self._goal_handle = goal_handle
            self._goal_id = goal_id
        self.controller.on_nav_goal_accepted(goal_id)
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_goal_result)
        except Exception as exc:  # pragma: no cover
            self.controller.log('warn', '获取导航结果失败: %r' % exc)
        return {'accepted': True, 'goal_id': goal_id, 'reason': None}

    def cancel_nav_goal(self):
        with self._lock:
            goal_handle = self._goal_handle
            self._goal_handle = None
        if goal_handle is None:
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('cancel_goal', '取消导航目标失败: %r' % exc)

    def _on_goal_feedback(self, feedback_msg):
        try:
            feedback = getattr(feedback_msg, 'feedback', None)
            data = self._extract_feedback(feedback)
            with self._lock:
                goal_id = self._goal_id
            self.controller.on_nav_goal_feedback(goal_id, **data)
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('goal_feedback', '反馈解析失败: %r' % exc)

    def _on_goal_result(self, future):
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            with self._lock:
                goal_id = self._goal_id
                self._goal_id = None
                self._goal_handle = None
            self.controller.on_nav_goal_result(goal_id, GOAL_STATUS_ABORTED)
            self._log_warn_once('goal_result', '导航结果读取失败: %r' % exc)
            return
        with self._lock:
            goal_id = self._goal_id
            self._goal_id = None
            self._goal_handle = None
        status = int(getattr(response, 'status', 0) or 0)
        inner = self._extract_result_status(getattr(response, 'result', None))
        if status not in (GOAL_STATUS_SUCCEEDED, GOAL_STATUS_CANCELED,
                          GOAL_STATUS_ABORTED) \
                and inner in (GOAL_STATUS_SUCCEEDED, GOAL_STATUS_CANCELED,
                              GOAL_STATUS_ABORTED):
            status = inner
        self.controller.on_nav_goal_result(goal_id, status)

    @staticmethod
    def _extract_feedback(feedback):
        """运行时自省读取 feedback 字段（兼容 Nav2 版本差异，§5.3）。"""
        data = {}
        if feedback is None:
            return data
        for name in ('distance_remaining',):
            value = getattr(feedback, name, None)
            if protocol.is_number(value):
                data['distance_remaining'] = float(value)
        for name in ('navigation_time', 'estimated_time_remaining'):
            value = getattr(feedback, name, None)
            sec = getattr(value, 'sec', None)
            nanosec = getattr(value, 'nanosec', 0)
            if isinstance(sec, int):
                total = float(sec) + float(nanosec or 0) * 1e-9
                if name == 'navigation_time':
                    data['navigation_time_s'] = total
        for name in ('number_of_recoveries', 'recoveries'):
            value = getattr(feedback, name, None)
            if isinstance(value, int) and not isinstance(value, bool):
                data['recoveries'] = int(value)
        return data

    @staticmethod
    def _extract_result_status(result_msg):
        """运行时自省读取 NavigateToPose.Result 内的状态码。"""
        if result_msg is None:
            return None
        for name in ('status', 'result', 'error_code'):
            value = getattr(result_msg, name, None)
            if isinstance(value, int) and not isinstance(value, bool):
                return int(value)
        return None

    # ------------------------------------------------------------------
    # lifecycle 激活
    # ------------------------------------------------------------------
    def begin_activation(self):
        with self._lock:
            if self._activation is not None and not self._activation.finished:
                return True
            activation = _NavActivation(self)
            self._activation = activation
        activation.start()
        self.controller.log('info', '开始 Nav2 生命周期激活（6 步，失败重试 3 次）')
        return True

    def cancel_activation(self):
        with self._lock:
            activation = self._activation
            self._activation = None
        if activation is not None and not activation.finished:
            self.controller.log('info', '会话结束，取消进行中的 Nav2 激活编排')

    def _tick_activation(self):
        with self._lock:
            activation = self._activation
        if activation is None:
            return
        activation.tick(self.now_mono())
        if activation.finished:
            with self._lock:
                if self._activation is activation:
                    self._activation = None

    # ------------------------------------------------------------------
    # 周期任务
    # ------------------------------------------------------------------
    def _tick_robot(self):
        """10 Hz：TF → robot 位姿（map 优先，退化 odom，再退化 /odom 消息）。"""
        try:
            now_wall = time.time()
            pose = None
            frame = 'odom'
            stamp = None
            transform = self._lookup('map', 'base_link')
            if transform is not None:
                frame = 'map'
                pose = transform.transform.translation
                stamp = _stamp_to_float(transform.header.stamp)
            else:
                transform = self._lookup('odom', 'base_link')
                if transform is not None:
                    frame = 'odom'
                    pose = transform.transform.translation
                    stamp = _stamp_to_float(transform.header.stamp)
            with self._lock:
                odom = self._last_odom
                scan = self._last_scan
            if pose is not None:
                rotation = transform.transform.rotation
                yaw = _yaw_from_quaternion(rotation)
            elif odom is not None:
                frame = 'odom'
                pose = None
                yaw = odom['yaw']
                stamp = odom['stamp']
            else:
                self.state.set_robot(None)
                return
            x = float(pose.x) if pose is not None else float(odom['x'])
            y = float(pose.y) if pose is not None else float(odom['y'])
            speed = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
            imu = {'yaw_deg': None, 'age_s': None}
            if odom is not None:
                speed = {'vx': float(odom['vx']), 'vy': float(odom['vy']),
                         'wz': float(odom['wz'])}
                imu = {'yaw_deg': math.degrees(protocol.wrap_angle(odom['yaw'])),
                       'age_s': max(0.0, now_wall - odom['arrived'])}
            scan_stats = {'front_m': None, 'min_m': None, 'age_s': None}
            if scan is not None:
                scan_stats = self._scan_stats(scan, now_wall)
            if stamp is None:
                stamp = now_wall
            self.state.set_robot({
                'frame': frame,
                'x': x,
                'y': y,
                'yaw': protocol.wrap_angle(float(yaw)),
                'stamp': float(stamp),
                'speed': speed,
                'imu': imu,
                'scan': scan_stats,
            })
        except Exception as exc:  # pragma: no cover - 运行时防御
            self._log_warn_once('robot_tick', 'robot 位姿更新失败: %r' % exc)

    @staticmethod
    def _scan_stats(scan, now_wall):
        min_range = None
        front_min = None
        ranges = scan['ranges']
        angle = scan['angle_min']
        increment = scan['angle_increment']
        for index, value in enumerate(ranges):
            if not protocol.is_number(value):
                continue
            distance = float(value)
            if distance < scan['range_min'] or distance > scan['range_max']:
                continue
            if min_range is None or distance < min_range:
                min_range = distance
            beam_angle = angle + index * increment
            if abs(beam_angle) <= math.pi / 6.0:
                if front_min is None or distance < front_min:
                    front_min = distance
        return {
            'front_m': front_min,
            'min_m': min_range,
            'age_s': max(0.0, now_wall - scan['arrived']),
        }

    def _tick_scan(self):
        """5 Hz：/scan → map/odom 坐标点，降采样 ≤360（§5.4）。"""
        try:
            with self._lock:
                scan = self._last_scan
            if scan is None:
                return
            if time.time() - scan['arrived'] > 2.0:
                return
            frame = scan['frame_id'] or 'laser'
            target, transform = None, None
            for candidate in ('map', 'odom'):
                transform = self._lookup(candidate, frame)
                if transform is not None:
                    target = candidate
                    break
            points = []
            ranges = scan['ranges']
            angle = scan['angle_min']
            increment = scan['angle_increment']
            for index, value in enumerate(ranges):
                if not protocol.is_number(value):
                    continue
                distance = float(value)
                if distance < scan['range_min'] or distance > scan['range_max']:
                    continue
                beam_angle = angle + index * increment
                px = distance * math.cos(beam_angle)
                py = distance * math.sin(beam_angle)
                if transform is not None:
                    translation = transform.transform.translation
                    yaw = _yaw_from_quaternion(transform.transform.rotation)
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    px, py = (float(translation.x) + px * cos_yaw
                              - py * sin_yaw,
                              float(translation.y) + px * sin_yaw
                              + py * cos_yaw)
                points.append((px, py))
            points = protocol.downsample_sequence(
                points, int(self.config.scan_max_points))
            flat = []
            for px, py in points:
                flat.append(round(px, 3))
                flat.append(round(py, 3))
            event = {'stamp': float(scan['stamp']),
                     'frame': target or frame,
                     'points': flat}
            self.controller.event_bus.publish('scan', event)
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('scan_tick', 'scan 事件生成失败: %r' % exc)

    def _tick_path(self):
        """2 Hz：/plan 折线降采样 ≤200（§5.4）。"""
        try:
            with self._lock:
                plan = self._last_plan
                dirty = self._plan_dirty
                self._plan_dirty = False
            if plan is None or not dirty:
                return
            points = protocol.downsample_sequence(
                plan['points'], int(self.config.path_max_points))
            flat = []
            for px, py in points:
                flat.append(round(float(px), 3))
                flat.append(round(float(py), 3))
            event = {'stamp': float(plan['stamp']),
                     'frame': plan['frame'] or 'map', 'points': flat}
            self.controller.event_bus.publish('path', event)
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('path_tick', 'path 事件生成失败: %r' % exc)

    def _tick_teleop(self):
        """20 Hz：/cmd_vel_teleop + 500 ms 看门狗（§5.1）。"""
        try:
            if self.state.estop_active():
                return
            now_wall = time.time()
            status, command = self.state.consume_teleop(
                now_wall, float(self.config.teleop_timeout_s))
            if status == 'active':
                self._teleop_expired_logged = False
                self._publish_twist(*command)
            elif status == 'expired':
                # 看门狗：连发两次零速后停止发布
                self._publish_twist(0.0, 0.0, 0.0)
                self._publish_twist(0.0, 0.0, 0.0)
                if not self._teleop_expired_logged:
                    self._teleop_expired_logged = True
                    self.controller.log(
                        'info', '遥控看门狗超时（%.1f s 无新指令），已发零速'
                        % float(self.config.teleop_timeout_s))
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('teleop_tick', '遥控发布失败: %r' % exc)

    def _tick_estop(self):
        """急停期间每秒重发 /estop=true（§3.3.5）。"""
        try:
            if self.state.estop_active():
                self.publish_estop(True)
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('estop_tick', '急停重发失败: %r' % exc)

    def _tick_graph(self):
        """1 Hz：节点/话题健康 + ros.ready + 速率（§3.1/§5.4）。"""
        try:
            graph = self.graph()
            nodes = set(graph['nodes'])
            publishers = graph['publishers']
            subscribers = graph['subscribers']
            diagnostics_age = self.state.diagnostics_age()
            ready = {
                'base': (publishers.get('/odom', 0) > 0
                         and publishers.get('/cmd_vel_safe', 0) > 0),
                'teleop': subscribers.get('/cmd_vel_teleop', 0) > 0,
                'map': (publishers.get('/map', 0) > 0
                        and self.state.map_grid() is not None),
                'localization': 'amcl' in nodes,
                'nav2': self.navigator_action_ready(),
                'diagnostics': (diagnostics_age is not None
                                and diagnostics_age <= 5.0),
            }
            self.state.set_ros(nodes=len(nodes), ready=ready,
                               ok=ready['base'] and ready['teleop'])
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('graph_tick', 'ROS 图查询失败: %r' % exc)

    def _tick_initialpose(self):
        """2 Hz 重发 /initialpose，直到 TF map→base_link 可用（§3.3.7）。"""
        try:
            with self._lock:
                initialpose = dict(self._initialpose) \
                    if self._initialpose is not None else None
            if initialpose is None:
                return
            self.publish_initialpose(initialpose['x'], initialpose['y'],
                                     initialpose['yaw'])
            if self.has_map_tf():
                with self._lock:
                    self._initialpose = None
                self.controller.log('info',
                                    'map→base_link 已出现，初始位姿生效')
                self.controller.on_initialpose_tf_ready()
                return
            if self.now_mono() > initialpose['until']:
                with self._lock:
                    self._initialpose = None
                self.controller.log(
                    'warn', '初始位姿已重发 %.0f s，map→base_link 仍不可用'
                    % float(self.config.initialpose_timeout_s))
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('initialpose_tick', '初始位姿重发失败: %r' % exc)

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _on_odom(self, msg):
        try:
            pose = msg.pose.pose
            stamp = _stamp_to_float(msg.header.stamp)
            self._last_odom = {
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'yaw': _yaw_from_quaternion(pose.orientation),
                'vx': float(msg.twist.twist.linear.x),
                'vy': float(msg.twist.twist.linear.y),
                'wz': float(msg.twist.twist.angular.z),
                'stamp': stamp,
                'arrived': time.time(),
            }
            self.state.tick_odom()
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('odom_cb', 'odom 解析失败: %r' % exc)

    def _on_scan(self, msg):
        try:
            self._last_scan = {
                'ranges': tuple(float(v) for v in msg.ranges),
                'angle_min': float(msg.angle_min),
                'angle_increment': float(msg.angle_increment),
                'range_min': float(msg.range_min),
                'range_max': float(msg.range_max),
                'frame_id': str(msg.header.frame_id or 'laser'),
                'stamp': _stamp_to_float(msg.header.stamp),
                'arrived': time.time(),
            }
            self.state.tick_scan()
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('scan_cb', 'scan 解析失败: %r' % exc)

    def _on_plan(self, msg):
        try:
            points = [(float(pose.pose.position.x),
                       float(pose.pose.position.y))
                      for pose in msg.poses]
            self._last_plan = {
                'points': points,
                'frame': str(msg.header.frame_id or 'map'),
                'stamp': _stamp_to_float(msg.header.stamp),
            }
            self._plan_dirty = True
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('plan_cb', 'plan 解析失败: %r' % exc)

    def _on_map(self, msg):
        try:
            info = msg.info
            grid = mapio.MapGrid(
                width=int(info.width),
                height=int(info.height),
                resolution=float(info.resolution),
                origin_x=float(info.origin.position.x),
                origin_y=float(info.origin.position.y),
                origin_yaw=_yaw_from_quaternion(info.origin.orientation),
                frame_id=str(msg.header.frame_id or 'map'),
                stamp=_stamp_to_float(msg.header.stamp),
                data=tuple(int(value) for value in msg.data),
            )
            changed, meta = self.state.set_map(grid, received_ts=time.time())
            if changed:
                # §3.2 map 事件：{"meta": <§3.1 map 对象>}（含 available=true）
                self.controller.event_bus.publish(
                    'map', {'meta': self.state.map_snapshot()})
                version = int(meta.get('version', 0))
                if version == 1:
                    self.controller.log(
                        'info', '已收到 /map 地图（v1，%dx%d @ %.3f m）'
                        % (grid.width, grid.height, grid.resolution))
                self._map_version_logged = version
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('map_cb', '/map 解析失败: %r' % exc)

    def _on_diagnostics(self, msg):
        try:
            items = []
            for status in msg.status:
                items.append({
                    'name': str(status.name),
                    'level': int(status.level),
                    'message': str(status.message),
                    'hardware_id': str(status.hardware_id),
                })
                if str(status.name) == ESTOP_DIAG_NAME:
                    active = self._parse_estop_from_status(status)
                    if active is not None:
                        self.controller.sync_estop_from_diagnostics(active)
            self.state.set_diagnostics(items, received_ts=time.time())
        except Exception as exc:  # pragma: no cover
            self._log_warn_once('diag_cb', '/diagnostics 解析失败: %r' % exc)

    @staticmethod
    def _parse_estop_from_status(status):
        """解析 ``Raspbot Safety: E-Stop`` 的 ``estop`` KeyValue（True/False）。"""
        try:
            values = getattr(status, 'values', None) or []
        except Exception:  # pragma: no cover
            return None
        for item in values:
            key = str(getattr(item, 'key', '')).strip().lower()
            if key == 'estop':
                raw = str(getattr(item, 'value', '')).strip().lower()
                if raw in ('true', '1', 'yes', 'on'):
                    return True
                if raw in ('false', '0', 'no', 'off'):
                    return False
        return None
