"""ROS 2 base driver for the Yahboom RASPBOT-V2 mecanum chassis.

The board answers at I2C 0x2B on /dev/i2c-1 and is driven through the vendor
Raspbot_Lib. That library exposes no encoder feedback, so /odom_raw here is a
dead-reckoning integration of the *commanded* velocity. It is fine for teleop
and visualisation and not a substitute for real wheel odometry.

Contract (CONTRACTS v1.2, D-01 fixed):
    subscribe : /cmd_vel_safe (parameter ``cmd_vel_topic``, produced by E),
                /buzzer, /led, /servo_pan, /servo_tilt
    publish   : /odom_raw   (nav_msgs/Odometry, 30 Hz, Reliable)
                /range      (sensor_msgs/Range, 5 Hz, SensorData)
                /line_sensor (std_msgs/UInt8, 5 Hz, Reliable)
                /raspbot_base/diagnostics (diagnostic_updater, 1 Hz; the
                updater default /diagnostics is remapped by base.launch.py,
                WP-H merges it into the single /diagnostics topic)
    no TF     : odom->base_link is owned exclusively by the EKF (WP-E); the
                TransformBroadcaster code was removed here (CONTRACTS v1.2 §3).
"""

import diagnostic_updater
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, ColorRGBA, Float32, UInt8

from .Raspbot_Lib import Raspbot
from .kinematics import clamp_velocity, wheel_pwm
from .odometry import integrate_pose, sanitize_dt, yaw_to_quat
from .safety import (CommandWatchdog, clamp_servo_pan, clamp_servo_tilt,
                     clamp_unit)
from .sensors import decode_line, decode_range

REG_LINE = 0x0A
REG_DIST_HI = 0x1B
REG_DIST_LO = 0x1A
RANGE_MAX = 4.0
SENSOR_WARN_INTERVAL = 5.0  # s: WARN at most once per interval (SW-G08)


class RaspbotBase(Node):

    def __init__(self):
        super().__init__('raspbot_base')

        self.declare_parameter('max_linear', 0.30)
        self.declare_parameter('max_angular', 1.50)
        self.declare_parameter('k_geom', 0.1245)
        self.declare_parameter('pwm_per_mps', 700.0)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('deadband_pwm', 25)
        self.declare_parameter('wheel_signs', [1, 1, 1, 1])
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('sensor_rate', 5.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_safe')
        self.declare_parameter('cmd_resend_interval', 1.0)
        self.declare_parameter('diag_rate', 1.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        def p(name):
            return self.get_parameter(name).value

        self.max_linear = float(p('max_linear'))
        self.max_angular = float(p('max_angular'))
        self.k_geom = float(p('k_geom'))
        self.pwm_per_mps = float(p('pwm_per_mps'))
        self.max_pwm = int(p('max_pwm'))
        self.deadband = int(p('deadband_pwm'))
        self.wheel_signs = tuple(int(s) for s in p('wheel_signs'))
        self.cmd_timeout = float(p('cmd_timeout'))
        self.cmd_vel_topic = str(p('cmd_vel_topic'))
        self.cmd_resend_interval = float(p('cmd_resend_interval'))
        self.diag_rate = float(p('diag_rate'))
        self.odom_frame = str(p('odom_frame'))
        self.base_frame = str(p('base_frame'))

        self.car = Raspbot()
        self.car.Ctrl_Ulatist_Switch(1)

        self._cmd = Twist()
        self._watchdog = CommandWatchdog(self.cmd_timeout)
        self._watchdog.feed(self._now_sec())
        self._last_pwms = None
        self._last_send = self.get_clock().now()

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_integrate = self.get_clock().now()

        # sensor / diagnostics state (SW-G08, SYS-S05 need the error counter)
        self.i2c_error_count = 0
        self._last_sensor_warn = None
        self._sensor_warn_count = 0

        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_odom = self.create_publisher(Odometry, 'odom_raw', odom_qos)
        self.pub_range = self.create_publisher(Range, 'range',
                                               qos_profile_sensor_data)
        self.pub_line = self.create_publisher(UInt8, 'line_sensor', 10)

        self.create_subscription(Twist, self.cmd_vel_topic,
                                 self._on_cmd_vel, 10)
        self.create_subscription(Bool, 'buzzer', self._on_buzzer, 10)
        self.create_subscription(ColorRGBA, 'led', self._on_led, 10)
        self.create_subscription(Float32, 'servo_pan', self._on_pan, 10)
        self.create_subscription(Float32, 'servo_tilt', self._on_tilt, 10)

        self.create_timer(1.0 / float(p('control_rate')), self._control_loop)
        self.create_timer(1.0 / float(p('sensor_rate')), self._sensor_loop)

        # 1 Hz self-diagnostics. The updater default topic is /diagnostics;
        # base.launch.py remaps it to /raspbot_base/diagnostics so the WP-H
        # aggregator stays the single publisher of /diagnostics (CONTRACTS v1.2,
        # SW-C04). Nodes started without the launch file still publish the
        # updater default.
        self.updater = diagnostic_updater.Updater(self, period=self.diag_rate)
        self.updater.setHardwareID('RASPBOT-V2 chassis 0x2B')
        self.updater.add('Chassis', self._diagnostics)

        self.get_logger().info(
            'raspbot_base up: I2C 0x2B, cmd topic=%s, k_geom=%.4f m, '
            '%.0f pwm/(m/s)' % (self.cmd_vel_topic, self.k_geom,
                                self.pwm_per_mps))

    def _now_sec(self):
        """ROS-clock time as float seconds (pure input for the watchdog)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ----------------------------------------------------------------- inputs

    def _on_cmd_vel(self, msg):
        self._cmd = msg
        self._watchdog.feed(self._now_sec())

    def _on_buzzer(self, msg):
        self.car.Ctrl_BEEP_Switch(1 if msg.data else 0)

    def _on_led(self, msg):
        r = int(clamp_unit(msg.r) * 255)
        g = int(clamp_unit(msg.g) * 255)
        b = int(clamp_unit(msg.b) * 255)
        self.car.Ctrl_WQ2812_brightness_ALL(r, g, b)

    def _on_pan(self, msg):
        self.car.Ctrl_Servo(1, int(clamp_servo_pan(msg.data)))

    def _on_tilt(self, msg):
        self.car.Ctrl_Servo(2, int(clamp_servo_tilt(msg.data)))

    # --------------------------------------------------------------- control

    def _control_loop(self):
        now = self.get_clock().now()

        if self._watchdog.stale(self._now_sec()):
            cmd = Twist()  # FR-A-02: command silence means stop
        else:
            cmd = self._cmd

        vx, vy, wz = clamp_velocity(cmd.linear.x, cmd.linear.y, cmd.angular.z,
                                    self.max_linear, self.max_angular)

        pwms = wheel_pwm(vx, vy, self.k_geom * wz,
                         pwm_per_mps=self.pwm_per_mps,
                         max_pwm=self.max_pwm,
                         deadband=self.deadband,
                         wheel_signs=self.wheel_signs)

        # FR-A-06: re-send the four wheel PWMs at least every
        # cmd_resend_interval even when the command did not change (the
        # expansion board forgets its last state after a reset).
        silent = (now - self._last_send).nanoseconds * 1e-9
        if pwms != self._last_pwms or silent > self.cmd_resend_interval:
            for index, value in enumerate(pwms):
                self.car.Ctrl_Muto(index, value)
            self._last_pwms = pwms
            self._last_send = now

        self._publish_odom(vx, vy, wz, now)

    def _publish_odom(self, vx, vy, wz, now):
        dt = sanitize_dt((now - self._last_integrate).nanoseconds * 1e-9)
        self._last_integrate = now
        self._x, self._y, self._theta = integrate_pose(
            self._x, self._y, self._theta, vx, vy, wz, dt)

        _, _, qz, qw = yaw_to_quat(self._theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Open loop: report honest, large covariance.
        odom.pose.covariance[0] = 0.10
        odom.pose.covariance[7] = 0.10
        odom.pose.covariance[35] = 0.20
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.10
        self.pub_odom.publish(odom)

    # --------------------------------------------------------------- sensors

    def _note_sensor_error(self, what):
        self.i2c_error_count += 1
        now = self.get_clock().now()
        if (self._last_sensor_warn is None
                or (now - self._last_sensor_warn).nanoseconds * 1e-9
                >= SENSOR_WARN_INTERVAL):
            self._sensor_warn_count += 1
            self._last_sensor_warn = now
            self.get_logger().warning('sensor read failed: %s' % what)

    def _read_reg(self, reg, length):
        # Raspbot_Lib swallows exceptions and returns None on bus errors, so a
        # None result is our only failure signal: count it, rate-limit the
        # WARN, never raise (SW-G08).
        try:
            data = self.car.read_data_array(reg, length)
        except Exception:
            data = None
        if not data:
            self._note_sensor_error('reg 0x%02X returned no data' % reg)
            return None
        return data

    def _sensor_loop(self):
        try:
            hi = self._read_reg(REG_DIST_HI, 1)
            lo = self._read_reg(REG_DIST_LO, 1)
            line = self._read_reg(REG_LINE, 1)
            if hi is not None and lo is not None:
                self._publish_range(hi[0], lo[0])
            if line is not None:
                self.pub_line.publish(UInt8(data=int(line[0])))
        except Exception as exc:  # never let a bus hiccup kill the node
            self._note_sensor_error(str(exc))

    def _publish_range(self, hi, lo):
        rng = Range()
        rng.header.stamp = self.get_clock().now().to_msg()
        rng.header.frame_id = 'ultrasonic'
        rng.radiation_type = Range.ULTRASOUND
        rng.field_of_view = 0.26
        rng.min_range = 0.02
        rng.max_range = RANGE_MAX
        # 0xFFFF (out of range) and 0 (no echo) both report max_range.
        rng.range = decode_range(hi, lo, max_range=RANGE_MAX)
        self.pub_range.publish(rng)

    # ----------------------------------------------------------- diagnostics

    def _diagnostics(self, stat):
        if self._watchdog.last_feed is None:
            age = 0.0
        else:
            age = max(0.0, self._now_sec() - self._watchdog.last_feed)
        stat.summary(DiagnosticStatus.OK, 'chassis ok')
        stat.add('heartbeat', '1')
        stat.add('i2c_error_count', str(self.i2c_error_count))
        stat.add('last_cmd_age_sec', '%.3f' % age)
        return stat

    # ------------------------------------------------------------------ stop

    def stop(self):
        try:
            for index in range(4):
                self.car.Ctrl_Muto(index, 0)
            self.car.Ctrl_Ulatist_Switch(0)
            self.get_logger().info('stop: wheels zeroed, ultrasonic off')
        except Exception as exc:
            self.get_logger().error('stop failed: %s' % exc)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RaspbotBase()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # SIGTERM / context shutdown: a clean exit, not an error (SW-G05).
        pass
    except rclpy._rclpy_pybind11.RCLError:
        # Lyrical race (SW-F02): a SIGTERM landing inside rclpy.spin's
        # wait-set initialization window raises RCLError("the given context
        # is not valid") instead of ExternalShutdownException. Treat it as a
        # clean shutdown too - exit code 0, wheels stopped in finally.
        pass
    finally:
        if node is not None:
            try:
                node.stop()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
