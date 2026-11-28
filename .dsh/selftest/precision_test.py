"""precision_test.py —— 地面旋转/横移精度测试（CAR-M 类）。

分段：
  1) 原地左转 90°（cmd wz=0.5 rad/s × 3.14 s）→ IMU 实测转角
  2) 原地右转 90°（回到原朝向）
  3) 左横移 0.40 m（cmd vy=0.10 m/s × 4 s）
  4) 右横移 0.40 m（回原位）
记录 IMU 偏航（物理证据）与 odom（开环），逐段打印并判定。
用法: precision_test.py [--yaw-speed 0.5] [--lin 0.10]
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


class Precision(Node):
    def __init__(self, abort_range=0.15):
        super().__init__('precision_test')
        self.abort_range = abort_range
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.odom = None
        self.yaw = None
        self.min_front = float('inf')
        self.aborted = False
        self.create_subscription(Odometry, '/odom_raw', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._front_cb,
                                 qos_profile_sensor_data)

    def _odom_cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def _imu_cb(self, m):
        q = m.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _front_cb(self, m):
        for i, r in enumerate(m.ranges):
            if not (math.isfinite(r) and m.range_min <= r <= m.range_max):
                continue
            ang = math.degrees(m.angle_min + i * m.angle_increment)
            ang = (ang + 180.0) % 360.0 - 180.0
            if abs(ang) <= 75.0 and r < self.abort_range:
                self.aborted = True

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(1.0 / 30.0)

    def drive(self, vx, vy, wz, sec):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok() and not self.aborted:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.002)
            time.sleep(1.0 / 30.0)
        self.pub.publish(Twist())
        self.spin_for(0.7)

    def seg(self, name, vx, vy, wz, sec, expect_yaw=None):
        a = (self.yaw, self.odom)
        self.drive(vx, vy, wz, sec)
        b = (self.yaw, self.odom)
        dyaw = self._wrap(b[0] - a[0]) if a[0] is not None else None
        dox = (b[1][0] - a[1][0]) if a[1] and b[1] else None
        doy = (b[1][1] - a[1][1]) if a[1] and b[1] else None
        err = (dyaw - expect_yaw) if (dyaw is not None
                                      and expect_yaw is not None) else None
        print(f'  {name}: imu_dyaw={dyaw:+.3f} rad '
              f'({math.degrees(dyaw):+.1f}°)' if dyaw is not None else
              f'  {name}: imu 无数据')
        if dox is not None:
            print(f'        odom Δ=({dox:+.3f},{doy:+.3f}) m')
        if err is not None:
            print(f'        目标 {expect_yaw:+.3f} rad，误差 {err:+.3f} rad '
                  f'({math.degrees(err):+.1f}°)')
        return {'dyaw': dyaw, 'err': err, 'dox': dox, 'doy': doy,
                'aborted': self.aborted}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaw-speed', type=float, default=0.5)
    ap.add_argument('--lin', type=float, default=0.10)
    args = ap.parse_args()
    rclpy.init()
    n = Precision()
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.odom and n.yaw is not None:
            break

    q = math.pi / 2
    t_rot = q / args.yaw_speed
    t_lin = 0.40 / args.lin
    print(f'== 旋转/横移精度测试（wz={args.yaw_speed} rad/s, '
          f'v={args.lin} m/s）==')
    r1 = n.seg('left+90°', 0.0, 0.0, args.yaw_speed, t_rot, expect_yaw=+q)
    r2 = n.seg('right-90°', 0.0, 0.0, -args.yaw_speed, t_rot, expect_yaw=-q)
    r3 = n.seg('strafe-left', 0.0, +args.lin, 0.0, t_lin)
    r4 = n.seg('strafe-right', 0.0, -args.lin, 0.0, t_lin)

    print('\n== 判定 ==')
    checks = []
    if r1['err'] is not None:
        checks.append(('旋转 +90° 误差 ≤ 15°', abs(r1['err']) <= math.radians(15),
                       f"{math.degrees(r1['err']):+.1f}°"))
    if r2['err'] is not None:
        checks.append(('旋转 -90° 误差 ≤ 15°', abs(r2['err']) <= math.radians(15),
                       f"{math.degrees(r2['err']):+.1f}°"))
    if r3['doy'] is not None:
        checks.append(('左横移 odom y>0.30', r3['doy'] > 0.30,
                       f"{r3['doy']:+.3f} m"))
    if r4['doy'] is not None:
        checks.append(('右横移 odom y<-0.30', r4['doy'] < -0.30,
                       f"{r4['doy']:+.3f} m"))
    checks.append(('全程前向无近障保护触发', not n.aborted,
                   '未触发' if not n.aborted else '触发'))
    passed = 0
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}  ({detail})")
        passed += 1 if ok else 0
    total = len(checks)
    print(f"\n[{'PASS' if passed == total else 'FAIL'}] "
          f'旋转/横移精度：{passed}/{total}')
    n.pub.publish(Twist())
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed == total else 1)


if __name__ == '__main__':
    main()
