"""scan_inspect.py —— 看静止时 /scan 的最近距离与角度分布（判断自反射/近障）。"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node('scan_inspect')
latest = {}


def cb(m):
    latest['m'] = m


n.create_subscription(LaserScan, '/scan', cb, qos_profile_sensor_data)
for _ in range(100):
    rclpy.spin_once(n, timeout_sec=0.1)
    if 'm' in latest:
        break
m = latest.get('m')
if m is None:
    print('[FAIL] 无 /scan 数据')
    raise SystemExit(1)
pts = []
for i, r in enumerate(m.ranges):
    if math.isfinite(r) and m.range_min <= r <= m.range_max:
        ang = math.degrees(m.angle_min + i * m.angle_increment)
        ang = (ang + 180.0) % 360.0 - 180.0
        pts.append((r, ang))
pts.sort()
print(f'== /scan 概览：{len(pts)} 个有效点，range_min={m.range_min} range_max={m.range_max:.1f}')
print('== 最近的 10 个点（距离 m, 角度° 0=正前/左正）==')
for r, a in pts[:10]:
    print(f'  {r:6.3f} m @ {a:+7.1f}°')
# 前向扇区（±30°）最近
front = [(r, a) for r, a in pts if abs(a) <= 30.0]
front.sort()
print('== 前向 ±30° 最近 5 点 ==')
for r, a in front[:5]:
    print(f'  {r:6.3f} m @ {a:+7.1f}°')
# 各扇区最近距离
print('== 各扇区最近距离 ==')
for lo, hi in ((-180, -120), (-120, -60), (-60, 0), (0, 60), (60, 120), (120, 180)):
    seg = [r for r, a in pts if lo <= a < hi]
    seg.sort()
    print(f'  [{lo:+4d}°,{hi:+4d}°): {seg[0]:.3f} m' if seg else
          f'  [{lo:+4d}°,{hi:+4d}°): 无')
n.destroy_node()
rclpy.shutdown()
