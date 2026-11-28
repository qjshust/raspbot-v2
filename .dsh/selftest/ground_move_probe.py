"""ground_move_probe.py —— 落地低速运动探针（C++ 节点链路）。

通过 /cmd_vel_teleop 注入分段指令，采集：
  /odom_raw（开环积分位姿）、/imu/data（真实偏航角——物理转动证据）、
  /scan（距离变化——真实平移/转动的物理证据）、/cmd_vel_safe（指令链）。
输出每段的前后差值表，判断"指令→物理运动"是否成立。

用法: ground_move_probe.py [--scale 1.0]   (scale 缩放速度/时长，默认 1.0)
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

SPEED = 0.10      # m/s 平移（低速）
YAW_SPEED = 0.40  # rad/s 旋转


class GroundProbe(Node):
    def __init__(self, scale=1.0):
        super().__init__('ground_move_probe')
        self.scale = scale
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.odom = None          # (x, y, theta)
        self.yaw = None           # rad（HWT101 绝对偏航）
        self.scan = None          # (min_valid, median_valid)
        self.safe = 0.0
        self.create_subscription(Odometry, '/odom_raw', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_safe', self._safe_cb, 10)

    # ---------------- callbacks ----------------
    def _odom_cb(self, m):
        q = m.pose.pose.orientation
        # yaw from quaternion
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def _imu_cb(self, m):
        q = m.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan_cb(self, m):
        vals = [r for r in m.ranges
                if math.isfinite(r) and m.range_min <= r <= m.range_max]
        if vals:
            vals.sort()
            self.scan = (vals[0], vals[len(vals) // 2])

    def _safe_cb(self, m):
        self.safe = m.linear.x

    # ---------------- helpers ----------------
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
            rclpy.spin_once(self, timeout_sec=0.005)

    def drive(self, vx, vy, wz, sec):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.002)
            time.sleep(1.0 / 30.0)
        self.pub.publish(Twist())   # 该段结束立即发零
        self.spin_for(0.6)          # 等待链路归零稳定

    def snapshot(self):
        return {'odom': self.odom, 'yaw': self.yaw, 'scan': self.scan}

    def phase(self, name, vx, vy, wz, sec):
        a = self.snapshot()
        t0 = time.monotonic()
        self.drive(vx * self.scale, vy * self.scale, wz * self.scale,
                   sec * self.scale)
        b = self.snapshot()
        res = {'name': name}
        if a['odom'] and b['odom']:
            res['dx'] = b['odom'][0] - a['odom'][0]
            res['dy'] = b['odom'][1] - a['odom'][1]
            res['dtheta'] = self._wrap(b['odom'][2] - a['odom'][2])
        if a['yaw'] is not None and b['yaw'] is not None:
            res['dyaw_imu'] = self._wrap(b['yaw'] - a['yaw'])
        if a['scan'] and b['scan']:
            res['dmin'] = b['scan'][0] - a['scan'][0]
            res['dmed'] = b['scan'][1] - a['scan'][1]
        res['dt'] = time.monotonic() - t0
        return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scale', type=float, default=1.0)
    ap.add_argument('--phases', type=str, default='all')
    args = ap.parse_args()
    rclpy.init()
    n = GroundProbe(args.scale)

    print('== 等待数据流就绪（odom/imu/scan）==')
    for _ in range(100):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.odom and n.yaw is not None and n.scan:
            break
    ok = n.odom is not None and n.yaw is not None
    print(f'  odom={n.odom is not None} imu={n.yaw is not None} '
          f'scan={n.scan is not None}')

    phases = [
        ('fwd',   SPEED, 0.0, 0.0, 1.5),
        ('back', -SPEED, 0.0, 0.0, 1.5),
        ('left',  0.0, SPEED, 0.0, 1.2),
        ('right', 0.0, -SPEED, 0.0, 1.2),
        ('rotl',  0.0, 0.0, YAW_SPEED, 1.2),
        ('rotr',  0.0, 0.0, -YAW_SPEED, 1.2),
    ]
    if args.phases != 'all':
        want = set(args.phases.split(','))
        phases = [p for p in phases if p[0] in want]

    results = []
    for name, vx, vy, wz, sec in phases:
        print(f'== 段 {name}: vx={vx * args.scale:.2f} vy={vy * args.scale:.2f} '
              f'wz={wz * args.scale:.2f} 时长={sec * args.scale:.1f}s ==')
        r = n.phase(name, vx, vy, wz, sec)
        results.append(r)
        print(f'  odom Δ=({r.get("dx", 0):+.3f},{r.get("dy", 0):+.3f}) '
              f'dθ={r.get("dtheta", 0):+.3f} | imu dyaw='
              f'{r.get("dyaw_imu", float("nan")):+.3f} rad | '
              f'scan Δmin={r.get("dmin", 0):+.3f} m Δmed={r.get("dmed", 0):+.3f} m')

    # 判定（物理证据）：
    print('\n== 判定（物理证据）==')
    checks = []
    by = {r['name']: r for r in results}
    if 'rotl' in by and 'rotr' in by:
        rl = by['rotl'].get('dyaw_imu', 0.0)
        rr = by['rotr'].get('dyaw_imu', 0.0)
        checks.append(('rotl 物理左转（IMU yaw 增大 >0.15 rad）', rl > 0.15, f'{rl:+.3f}'))
        checks.append(('rotr 物理右转（IMU yaw 减小 < -0.15 rad）', rr < -0.15, f'{rr:+.3f}'))
    if 'fwd' in by:
        f = by['fwd']
        moved = abs(f.get('dx', 0)) > 0.05 or abs(f.get('dmed', 0)) > 0.02
        checks.append(('fwd 前进（odom x>0.05 或 scan 变化>0.02m）', moved,
                       f"dx={f.get('dx', 0):+.3f} dmed={f.get('dmed', 0):+.3f}"))
    if 'back' in by:
        b = by['back']
        moved = b.get('dx', 0) < -0.05 or abs(b.get('dmed', 0)) > 0.02
        checks.append(('back 后退（odom x<-0.05 或 scan 变化）', moved,
                       f"dx={b.get('dx', 0):+.3f} dmed={b.get('dmed', 0):+.3f}"))
    if 'left' in by and 'right' in by:
        l, r = by['left'], by['right']
        checks.append(('left 横移（odom y>0.03）', l.get('dy', 0) > 0.03,
                       f"dy={l.get('dy', 0):+.3f}"))
        checks.append(('right 横移（odom y<-0.03）', r.get('dy', 0) < -0.03,
                       f"dy={r.get('dy', 0):+.3f}"))

    passed = 0
    for name, ok_, detail in checks:
        print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}  ({detail})")
        passed += 1 if ok_ else 0
    print(f"\n[{'PASS' if passed == len(checks) else 'FAIL'}] "
          f'地面运动探针：{passed}/{len(checks)}')
    n.pub.publish(Twist())
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed == len(checks) else 1)


if __name__ == '__main__':
    main()
