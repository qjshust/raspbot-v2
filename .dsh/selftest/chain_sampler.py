"""Live sampler: subscribe cmd chain topics and print periodic snapshots.

Usage: chain_sampler.py --seconds 60
Prints every 2s: cmd_vel_nav_raw.vx, cmd_vel_nav.vx, cmd_vel_safe.vx,
odom pose x, collision_monitor action types.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState


class Sampler(Node):
    def __init__(self):
        super().__init__('chain_sampler')
        self.last = {
            'nav_raw': 0.0, 'nav': 0.0, 'safe': 0.0, 'odom_x': 0.0, 'cm': 'n/a'}
        self.cm = 'n/a'
        # nav2 的 cmd 输出是 SensorData QoS；订阅必须匹配，否则静默收不到
        q = qos_profile_sensor_data
        self.create_subscription(Twist, '/cmd_vel_nav_raw',
                                 lambda m: self.last.__setitem__('nav_raw', m.linear.x), q)
        self.create_subscription(Twist, '/cmd_vel_nav',
                                 lambda m: self.last.__setitem__('nav', m.linear.x), q)
        self.create_subscription(Twist, '/cmd_vel_safe',
                                 lambda m: self.last.__setitem__('safe', m.linear.x), q)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: self.last.__setitem__(
                                     'odom_x', m.pose.pose.position.x), 10)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state',
            lambda m: self.__setattr__(
                'cm', ','.join(p.action_type for p in m.polygons)), q)

    def run(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if int(time.monotonic() * 10) % 20 == 0:  # 每 2 s 打印
                print(f"t={time.monotonic():.0f} raw={self.last['nav_raw']:.3f} "
                      f"nav={self.last['nav']:.3f} safe={self.last['safe']:.3f} "
                      f"odom_x={self.last['odom_x']:.3f} cm=[{self.cm}]")
                time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    args = ap.parse_args()
    rclpy.init()
    n = Sampler()
    n.run(args.seconds)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
