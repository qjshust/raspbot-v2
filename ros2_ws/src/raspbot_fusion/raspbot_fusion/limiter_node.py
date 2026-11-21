"""cmd_vel_limiter 节点 —— /cmd_vel_safe 的唯一发布者（契约 CONTRACTS §2）。

订阅 /cmd_vel（Reliable, depth 10）、/estop（Bool, Reliable, depth 1）、
/imu/data（SensorData，仅取 header.stamp 算新鲜度）；
以 control_rate（默认 30 Hz）发布 /cmd_vel_safe（Twist, Reliable）。

安全链路（P0，J-07/SW-F01 三级停车 ≤0.5 s）：
  /cmd_vel 静默超过 input_timeout（0.4 s）→ 输出瞬时归零（不经斜率限幅），
  30 Hz 持续发布零速 → 底盘 A 的 0.5 s 看门狗（cmd_timeout）作为最后兜底。
  总链 ≈0.4 s + 两级传播各 ~33 ms ≈ 0.47 s ≤ 0.54 s 判据。
"""

import time

import rclpy
import rclpy.time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from raspbot_fusion.limiter import VelocityLimiter


class CmdVelLimiterNode(Node):
    """限速节点：限幅 + IMU 掉线降级 + estop 锁存 + 输入超时看门狗。"""

    def __init__(self):
        super().__init__('cmd_vel_limiter')

        # ---- 参数（默认值 = config/limiter.yaml，launch 里以 yaml 传入覆盖）----
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('max_linear', 0.30)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('max_lin_accel', 0.6)
        self.declare_parameter('max_ang_accel', 2.0)
        self.declare_parameter('degraded_max_linear', 0.15)
        self.declare_parameter('imu_timeout', 1.0)
        self.declare_parameter('input_timeout', 0.4)
        self.declare_parameter('warn_interval', 2.0)

        self._limiter = VelocityLimiter(
            max_linear=float(self.get_parameter('max_linear').value),
            max_angular=float(self.get_parameter('max_angular').value),
            max_lin_accel=float(self.get_parameter('max_lin_accel').value),
            max_ang_accel=float(self.get_parameter('max_ang_accel').value),
            degraded_max_linear=float(self.get_parameter('degraded_max_linear').value),
            imu_timeout=float(self.get_parameter('imu_timeout').value),
            input_timeout=float(self.get_parameter('input_timeout').value),
        )

        rate = float(self.get_parameter('control_rate').value)
        if rate <= 0.0:
            raise ValueError('control_rate 必须为正数，收到 %r' % rate)

        # ---- QoS（契约 CONTRACTS §2）----
        reliable10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        reliable1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # ---- 订阅 / 发布 ----
        self._sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd, reliable10)
        self._sub_estop = self.create_subscription(
            Bool, '/estop', self._on_estop, reliable1)
        # /imu/data 仅取 header.stamp 计算新鲜度（SensorData QoS）
        self._sub_imu = self.create_subscription(
            Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)
        self._pub_safe = self.create_publisher(
            Twist, '/cmd_vel_safe', reliable10)

        # ---- 内部状态 ----
        self._target = (0.0, 0.0, 0.0)      # 最近一条 /cmd_vel
        self._last_cmd_t = None             # time.monotonic()，输入新鲜度
        self._last_imu_stamp = None         # rclpy Time，IMU 新鲜度
        self._last_step_t = None
        self._last_warn_t = 0.0
        # 初始视为已知状态，避免开机（传感器/指令尚未到达）误告警
        self._was_imu_stale = True
        self._was_input_stale = True
        self._estop_latched = False

        self._timer = self.create_timer(1.0 / rate, self._step)
        self.get_logger().info(
            'cmd_vel_limiter 启动：%d Hz → /cmd_vel_safe，max_linear=%.2f m/s, '
            'max_angular=%.2f rad/s, degraded=%.2f m/s, imu_timeout=%.1f s, '
            'input_timeout=%.1f s'
            % (int(round(rate)), self._limiter.max_linear,
               self._limiter.max_angular, self._limiter.degraded_max_linear,
               self._limiter.imu_timeout, self._limiter.input_timeout))

    # ---------- 回调 ----------
    def _on_cmd(self, msg):
        self._target = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_cmd_t = time.monotonic()

    def _on_estop(self, msg):
        active = bool(msg.data)
        self._estop_latched = active
        self._limiter.estop(active)
        if active:
            self.get_logger().warning(
                'ESTOP 锁存：此后 /cmd_vel_safe 恒零，直到显式发布 /estop=false')
        else:
            self.get_logger().info('ESTOP 解除：恢复输出（从零平滑加速）')

    def _on_imu(self, msg):
        # 仅记录时间戳用于新鲜度判定（FR-E-02）。
        # 必须转成 rclpy.time.Time 才能与 get_clock().now() 相减（builtin_interfaces
        # 的 Time 不支持减法）；clock_type 与节点时钟保持一致（use_sim_time=false
        # 时为 SYSTEM_TIME），否则 Time - Time 会抛 "different clock types"。
        self._last_imu_stamp = rclpy.time.Time.from_msg(
            msg.header.stamp, clock_type=self.get_clock().clock_type)

    # ---------- 主循环 ----------
    def _step(self):
        now_mono = time.monotonic()
        dt = 0.0 if self._last_step_t is None else (now_mono - self._last_step_t)
        self._last_step_t = now_mono

        imu_age = None
        if self._last_imu_stamp is not None:
            imu_age = (self.get_clock().now() - self._last_imu_stamp).nanoseconds * 1e-9
            if imu_age < 0.0:
                imu_age = 0.0  # 时钟跳变防护：未来时间戳按新鲜处理
        input_age = None if self._last_cmd_t is None else (now_mono - self._last_cmd_t)

        vx, vy, wz = self._limiter.step(
            self._target[0], self._target[1], self._target[2],
            dt, imu_age, input_age)

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0
        msg.angular.z = wz
        self._pub_safe.publish(msg)

        self._warn(now_mono, imu_age, input_age)

    def _warn(self, now_mono, imu_age, input_age):
        """状态转换告警，限频 warn_interval 秒（要求 WARN ≤1/2 s）。"""
        warn_interval = float(self.get_parameter('warn_interval').value)
        limiter = self._limiter
        stale_imu = limiter.imu_stale(imu_age)
        stale_in = limiter.input_stale(input_age)
        can_log = (now_mono - self._last_warn_t) >= warn_interval

        if stale_imu and not self._was_imu_stale and can_log:
            self.get_logger().warning(
                'IMU 掉线（无数据或 age>%.1f s）：线性速度上限降至 %.2f m/s'
                % (limiter.imu_timeout, limiter.degraded_max_linear))
            self._last_warn_t = now_mono
        if not stale_imu and self._was_imu_stale and can_log:
            self.get_logger().info(
                'IMU 恢复：恢复满速上限 %.2f m/s' % limiter.max_linear)
            self._last_warn_t = now_mono
        if stale_in and not self._was_input_stale and can_log:
            self.get_logger().warning(
                '输入指令超时（/cmd_vel 超过 %.1f s 未更新）：瞬时归零'
                % limiter.input_timeout)
            self._last_warn_t = now_mono
        if not stale_in and self._was_input_stale and can_log:
            self.get_logger().info('输入指令恢复')
            self._last_warn_t = now_mono

        self._was_imu_stale = stale_imu
        self._was_input_stale = stale_in

    # ---------- 退出安全 ----------
    def publish_zero(self):
        """任何退出路径补发零速（底盘 0.5 s 看门狗兜底之前先主动停车）。"""
        self._pub_safe.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLimiterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_zero()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
