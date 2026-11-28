"""straight_line_test.py —— 地面直线行驶精度测试（CAR-M01 类）。

以 0.15 m/s 指令直行 2.0 m（13.3 s），全程记录：
  - /imu/data 偏航角（100 Hz）→ 实测航向漂移
  - /odom_raw 位姿 → 开环里程
  - /scan 最近距离 → 近距保护（<0.18 m 立即停车）
结束后输出：
  指令距离、odom 距离、IMU 航向漂移（最大/末值）、最大偏航角速度
用法: straight_line_test.py [--speed 0.15] [--dist 2.0] [--abort-range 0.18]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan


class StraightTest(Node):
    def __init__(self, abort_range):
        super().__init__('straight_test')
        self.abort_range = abort_range
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.odom = None
        self.yaw = None
        self.yaw0 = None
        self.yaw_samples = []       # (t, yaw)
        self.min_range = float('inf')
        self.aborted = False
        self.create_subscription(Odometry, '/odom_raw', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._scan_cb,
                                 qos_profile_sensor_data)

    def _odom_cb(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _imu_cb(self, m):
        q = m.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.yaw = yaw
        if self.yaw0 is None:
            self.yaw0 = yaw
        self.yaw_samples.append(yaw)

    def _scan_cb(self, m):
        # 近距保护只看前向扇区（±75°）：后方/侧后方的车身反射或家具不应误触
        for i, r in enumerate(m.ranges):
            if not (math.isfinite(r) and m.range_min <= r <= m.range_max):
                continue
            ang = m.angle_min + i * m.angle_increment
            ang = math.degrees(ang)
            ang = (ang + 180.0) % 360.0 - 180.0
            if abs(ang) <= 75.0 and r < self.min_range:
                self.min_range = r

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def drive_straight(self, speed, dist):
        dur = dist / speed
        msg = Twist()
        msg.linear.x = speed
        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < dur and rclpy.ok() and not self.aborted:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.002)
            time.sleep(1.0 / 30.0)
            if self.min_range < self.abort_range:
                self.aborted = True
            if time.monotonic() - last_print >= 2.0:
                last_print = time.monotonic()
                y = self._wrap((self.yaw or 0) - (self.yaw0 or 0))
                print(f'  t={time.monotonic() - t0:5.1f}s odom_x='
                      f'{(self.odom[0] if self.odom else 0):+.3f} '
                      f'imu_dyaw={y:+.3f} min_range={self.min_range:.2f}')
        self.pub.publish(Twist())
        end = time.monotonic() + 0.8
        while time.monotonic() < end and rclpy.ok():
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(1.0 / 30.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--dist', type=float, default=2.0)
    ap.add_argument('--abort-range', type=float, default=0.18)
    args = ap.parse_args()
    rclpy.init()
    n = StraightTest(args.abort_range)

    # 等数据流
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.odom and n.yaw is not None:
            break
    n.yaw0 = n.yaw
    n.yaw_samples.clear()
    o0 = n.odom
    print(f'== 起点 odom=({o0[0]:+.3f},{o0[1]:+.3f}) imu_yaw0={n.yaw:+.3f}')
    print(f'== 直行 {args.dist:.2f} m @ {args.speed:.2f} m/s '
          f'（{args.dist / args.speed:.1f} s，近距保护 {args.abort_range:.2f} m）==')
    t0 = time.monotonic()
    n.drive_straight(args.speed, args.dist)
    dt = time.monotonic() - t0
    rclpy.spin_once(n, timeout_sec=0.2)

    o1 = n.odom
    ds = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    yaw_end = n._wrap(n.yaw - n.yaw0)
    # 最大偏航偏离（相对起点）
    max_dev = 0.0
    for y in n.yaw_samples:
        max_dev = max(max_dev, abs(n._wrap(y - n.yaw0)))
    # 平均航向漂移速率
    rate = max_dev / max(dt, 1e-6)

    print('\n== 结果 ==')
    print(f'  指令距离/时长: {args.dist:.2f} m / {args.dist / args.speed:.1f} s'
          f'（实际 {dt:.1f} s）')
    print(f'  开环 odom 位移: {ds:.3f} m')
    print(f'  IMU 航向末值漂移: {yaw_end:+.4f} rad '
          f'({math.degrees(yaw_end):+.1f}°)')
    print(f'  IMU 航向最大偏离: {max_dev:.4f} rad '
          f'({math.degrees(max_dev):+.1f}°)')
    print(f'  平均漂移速率: {rate:.4f} rad/s（{math.degrees(rate):.2f} °/s）')
    print(f'  全程最近障碍距离: {n.min_range:.2f} m'
          f'{"（已触发保护停车！）" if n.aborted else ""}')
    ok = (not n.aborted) and (dt >= args.dist / args.speed * 0.9)
    print(f"\n[{'PASS' if ok else 'FAIL'}] 直行执行完成"
          f"（航向漂移由机械/地面打滑决定，需人工卷尺量实际距离与横向偏差）")
    n.pub.publish(Twist())
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
