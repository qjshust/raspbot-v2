"""odom_relay 节点 —— /odom → /odometry/filtered 轻量转发（FR-E-05 P2）。

不用 topic_tools/relay，保证零额外依赖。消息原样转发（保留 header.stamp、
frame_id、child_frame_id 与协方差），发布节奏 30 Hz 跟随 ekf_node 输出。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class OdomRelayNode(Node):
    def __init__(self):
        super().__init__('odom_relay')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            Odometry, '/odometry/filtered', qos)
        self._sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, qos)
        self.get_logger().info('odom_relay 启动：/odom → /odometry/filtered')

    def _on_odom(self, msg):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
