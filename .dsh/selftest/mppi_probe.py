"""Comprehensive MPPI probe: send stamped FollowPath, log raw cmd values,
optimal trajectory first points, and odom — no blocking get_result.

Usage: mppi_probe.py [--x 1.0] [--n 40] [--duration 40]
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath


class Probe(Node):
    def __init__(self, x, n, duration):
        super().__init__('mppi_probe')
        self.x = x
        self.n = n
        self.duration = duration
        self.raw = 0.0
        self.raw_msgs = 0
        self.raw_msgs_rel = 0
        self.raw_msgs_be = 0
        self.odom_x = None
        self.odom0 = None
        self.opt = None
        self.opt_msgs = 0
        # 双 QoS 订阅：Reliable 与 BestEffort 各一份，定位接收问题
        self.create_subscription(Twist, '/cmd_vel_nav_raw',
                                 self._raw_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav_raw',
                                 self._raw_cb2, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Path, '/controller_server/optimal_trajectory',
                                 self._opt_cb, 10)
        self.create_subscription(Path, '/controller_server/optimal_trajectory',
                                 self._opt_cb2, qos_profile_sensor_data)
        self.client = ActionClient(self, FollowPath, 'follow_path')

    def _raw_cb(self, m):
        self.raw = (m.linear.x, m.linear.y, m.angular.z)
        self.raw_msgs += 1
        self.raw_msgs_rel += 1

    def _raw_cb2(self, m):
        self.raw = (m.linear.x, m.linear.y, m.angular.z)
        self.raw_msgs += 1
        self.raw_msgs_be += 1

    def _odom_cb(self, m):
        self.odom_x = m.pose.pose.position.x
        if self.odom0 is None:
            self.odom0 = m.pose.pose.position.x

    def _opt_cb(self, m):
        self.opt = m
        self.opt_msgs += 1

    def _opt_cb2(self, m):
        self.opt = m
        self.opt_msgs += 1

    def send(self):
        goal = FollowPath.Goal()
        now = self.get_clock().now().to_msg()
        goal.path.header.frame_id = 'odom'
        goal.path.header.stamp = now
        for i in range(self.n + 1):
            p = PoseStamped()
            p.header.frame_id = 'odom'
            p.header.stamp = now
            p.pose.position.x = self.x * i / self.n
            p.pose.orientation.w = 1.0
            goal.path.poses.append(p)
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        handle = future.result()
        print(f"[{'OK' if handle else 'FAIL'}] goal "
              f"{'accepted' if handle else 'rejected'}")
        return handle

    def run(self):
        if not self.client.wait_for_server(20.0):
            print('[FAIL] action server 不可用')
            return 1
        self.send()
        deadline = time.monotonic() + self.duration
        last = 0.0
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if now - last >= 3.0:
                opt_pts = None
                if self.opt is not None and len(self.opt.poses) >= 2:
                    p0 = self.opt.poses[0].pose.position
                    p1 = self.opt.poses[1].pose.position
                    opt_pts = (round(p0.x, 3), round(p0.y, 3),
                               round(p1.x, 3), round(p1.y, 3))
                print(f"t={now:.0f} raw={self.raw} "
                      f"raw_msgs={self.raw_msgs}(rel={self.raw_msgs_rel}/"
                      f"be={self.raw_msgs_be}) "
                      f"odom_x={self.odom_x} odom0={self.odom0} "
                      f"opt_msgs={self.opt_msgs} opt[0:2]={opt_pts}")
                last = now
        moved = (self.odom_x is not None and self.odom0 is not None
                 and abs(self.odom_x - self.odom0) > 0.05)
        print(f'[SUMMARY] raw_msgs={self.raw_msgs} odom0={self.odom0} '
              f'odom_x={self.odom_x} moved={moved}')
        return 0 if moved else 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=1.0)
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--duration', type=float, default=40.0)
    args = ap.parse_args()
    rclpy.init()
    n = Probe(args.x, args.n, args.duration)
    rc = n.run()
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
