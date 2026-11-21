"""fake_base — mock chassis node (FR-H-04 / SW-I04).

Replaces raspbot_base (I2C 0x2B) for offline integration: subscribes
/cmd_vel_safe and publishes /odom_raw at `rate` Hz using the same
semi-implicit dead-reckoning as raspbot_base (theta first, then x/y).

By default it does NOT publish TF: odom->base_link is owned by the EKF
(raspbot_fusion / CONTRACTS §3). Set publish_tf:=true only for standalone
tests of this node without the fusion stack.
"""

import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class FakeBase(Node):

    def __init__(self):
        super().__init__('fake_base')

        self.declare_parameter('rate', 30.0)
        self.declare_parameter('noise', 0.0)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        def p(name):
            return self.get_parameter(name).value

        self.rate = float(p('rate'))
        self.noise = float(p('noise'))
        self.cmd_timeout = float(p('cmd_timeout'))
        self.odom_frame = str(p('odom_frame'))
        self.base_frame = str(p('base_frame'))

        self._cmd = Twist()
        self._last_cmd = self.get_clock().now()
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last = self.get_clock().now()

        # 契约：/cmd_vel_safe 由 E 发布（Reliable, depth 10）；/odom_raw 同 QoS
        self.pub_odom = self.create_publisher(Odometry, '/odom_raw', 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self._on_cmd, 10)

        # TF 属 E（CONTRACTS §3），默认不发；仅独立测试时经参数开启
        self.tf_broadcaster = None
        if bool(p('publish_tf')):
            self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            'fake_base up: rate=%.1f Hz noise=%.4f publish_tf=%s (TF owned by E)'
            % (self.rate, self.noise, self.tf_broadcaster is not None))

    # ------------------------------------------------------------------ input

    def _on_cmd(self, msg):
        self._cmd = msg
        self._last_cmd = self.get_clock().now()

    # ------------------------------------------------------------------ tick

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last).nanoseconds * 1e-9
        self._last = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0

        # 与 raspbot_base 相同的超时停车语义（SW-F01）
        silent = (now - self._last_cmd).nanoseconds * 1e-9
        cmd = self._cmd if silent <= self.cmd_timeout else Twist()
        vx = cmd.linear.x
        vy = cmd.linear.y
        wz = cmd.angular.z

        # 半隐式积分：先 theta，再 x/y（与 raspbot_base._publish_odom 一致）
        self._theta += wz * dt
        self._x += (vx * math.cos(self._theta) - vy * math.sin(self._theta)) * dt
        self._y += (vx * math.sin(self._theta) + vy * math.cos(self._theta)) * dt

        # 噪声只扰动发布位姿，不污染积分状态
        x = self._x + random.gauss(0.0, self.noise)
        y = self._y + random.gauss(0.0, self.noise)
        th = self._theta + random.gauss(0.0, self.noise)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(th / 2.0)
        odom.pose.pose.orientation.w = math.cos(th / 2.0)
        # 开环航位推算：诚实的大协方差（与 raspbot_base 同口径）
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

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.z = odom.pose.pose.orientation.z
            tf.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = FakeBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
