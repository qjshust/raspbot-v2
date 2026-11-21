"""FollowPath goal sender with properly stamped path (odom frame).

Usage: send_follow_path.py [--x 1.0] [--n 40] [--duration 45]
Sends /follow_path action with poses stamped "now", then monitors
/cmd_vel_nav_raw and /odom pose x for the duration. Exit 0 if odom moved.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowPath


class Sender(Node):
    def __init__(self, x, n, duration):
        super().__init__('follow_sender')
        self.x = x
        self.n = n
        self.duration = duration
        self.raw = 0.0
        self.raw_count = 0
        self.odom_x = None
        self.odom0 = None
        self.create_subscription(Twist, '/cmd_vel_nav_raw',
                                 self._raw_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.client = ActionClient(self, FollowPath, 'follow_path')

    def _raw_cb(self, m):
        self.raw = m.linear.x
        self.raw_count += 1

    def _odom_cb(self, m):
        self.odom_x = m.pose.pose.position.x
        if self.odom0 is None:
            self.odom0 = m.pose.pose.position.x

    def run(self):
        if not self.client.wait_for_server(20.0):
            print('[FAIL] follow_path server 不可用')
            return 1
        goal = FollowPath.Goal()
        now = self.get_clock().now().to_msg()
        goal.path.header.frame_id = 'odom'
        goal.path.header.stamp = now
        from geometry_msgs.msg import PoseStamped
        for i in range(self.n + 1):
            p = PoseStamped()
            p.header.frame_id = 'odom'
            p.header.stamp = now
            p.pose.position.x = self.x * i / self.n
            p.pose.orientation.w = 1.0
            goal.path.poses.append(p)
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        handle = future.result()
        if handle is None:
            print('[FAIL] goal rejected')
            return 1
        print('[OK] follow_path goal accepted')
        deadline = time.monotonic() + self.duration
        last_print = 0.0
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - last_print >= 2.0:
                r = handle.get_result()
                st = f' status={r.status} code={r.result.error_code}' if r else ''
                print(f"t={time.monotonic():.0f} raw={self.raw:.3f} "
                      f"raw_msgs={self.raw_count} odom_x={self.odom_x} "
                      f"odom0={self.odom0}{st}")
                last_print = time.monotonic()
        r = handle.get_result()
        moved = (self.odom_x is not None and self.odom0 is not None
                 and abs(self.odom_x - self.odom0) > 0.1)
        print(f'[RESULT] status={r.status if r else None} '
              f'code={r.result.error_code if r else None} '
              f'odom0={self.odom0} odom_x={self.odom_x} moved={moved}')
        return 0 if moved else 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=1.0)
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--duration', type=float, default=45.0)
    args = ap.parse_args()
    rclpy.init()
    n = Sender(args.x, args.n, args.duration)
    rc = n.run()
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
