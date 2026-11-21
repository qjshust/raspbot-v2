"""最终观测器：全进程内、无阻塞、打印图信息与全部环数值。

Usage: final_probe.py [--x 1.0] [--n 40] [--duration 40]
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


class FinalProbe(Node):
    def __init__(self, x, n, duration):
        super().__init__('final_probe')
        self.x = x
        self.n = n
        self.duration = duration
        self.raw = None
        self.raw_rel = 0
        self.raw_be = 0
        self.odom_x = None
        self.odom0 = None
        self.opt_pts = None
        self.opt_msgs = 0
        self.create_subscription(Twist, '/cmd_vel_nav_raw', self._raw_rel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav_raw', self._raw_be_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Path, '/controller_server/optimal_trajectory',
                                 self._opt_cb, 10)
        self.client = ActionClient(self, FollowPath, 'follow_path')

    def _raw_rel_cb(self, m):
        self.raw = (m.linear.x, m.linear.y, m.angular.z)
        self.raw_rel += 1

    def _raw_be_cb(self, m):
        self.raw = (m.linear.x, m.linear.y, m.angular.z)
        self.raw_be += 1

    def _odom_cb(self, m):
        self.odom_x = m.pose.pose.position.x
        if self.odom0 is None:
            self.odom0 = m.pose.pose.position.x

    def _opt_cb(self, m):
        self.opt_msgs += 1
        if len(m.poses) >= 2:
            p0 = m.poses[0].pose.position
            p1 = m.poses[1].pose.position
            self.opt_pts = (round(p0.x, 3), round(p0.y, 3),
                            round(p1.x, 3), round(p1.y, 3))

    def dump_graph(self):
        for topic in ('/cmd_vel_nav_raw', '/odom'):
            pubs = self.get_publishers_info_by_topic(topic)
            print(f'  [{topic}] publishers={len(pubs)}')
            for p in pubs:
                print(f'    node={p.node_name} ns={p.node_namespace} '
                      f'rel={p.qos_profile.reliability.name}')
        print(f'  subs on /cmd_vel_nav_raw = '
              f'{self.count_subscribers("/cmd_vel_nav_raw")}')

    def run(self):
        if not self.client.wait_for_server(20.0):
            print('[FAIL] action server 不可用')
            return 1
        print('== 发目标前的图信息 ==')
        self.dump_graph()
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
        print(f"[{'OK' if handle else 'FAIL'}] goal {'accepted' if handle else 'rejected'}")
        deadline = time.monotonic() + self.duration
        last = 0.0
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now_t = time.monotonic()
            if now_t - last >= 3.0:
                print(f"t={now_t:.0f} raw={self.raw} "
                      f"raw_rel={self.raw_rel} raw_be={self.raw_be} "
                      f"odom0={self.odom0} odom_x={self.odom_x} "
                      f"opt_msgs={self.opt_msgs} opt[0:2]={self.opt_pts}")
                last = now_t
        moved = (self.odom_x is not None and self.odom0 is not None
                 and abs(self.odom_x - self.odom0) > 0.05)
        print(f'[SUMMARY] raw_rel={self.raw_rel} raw_be={self.raw_be} '
              f'odom0={self.odom0} odom_x={self.odom_x} moved={moved}')
        return 0 if moved else 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=1.0)
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--duration', type=float, default=40.0)
    args = ap.parse_args()
    rclpy.init()
    n = FinalProbe(args.x, args.n, args.duration)
    rc = n.run()
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
