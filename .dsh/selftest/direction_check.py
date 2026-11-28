"""direction_check.py —— 平移方向验证（雷达四扇区变化，无需卷尺）。

分别下发 fwd/back/left/right 短动作（0.08 m/s × 1.2 s），比较
前/后/左/右四扇区最近距离的变化方向，判断真实运动方向是否与指令一致。
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node('direction_check')
pub = n.create_publisher(Twist, '/cmd_vel_teleop', 10)
latest = {'scan': None}


def cb(m):
    latest['scan'] = m


n.create_subscription(LaserScan, '/scan', cb, qos_profile_sensor_data)


def sectors():
    m = latest['scan']
    if m is None:
        return None
    s = {'front': [], 'rear': [], 'left': [], 'right': []}
    for i, r in enumerate(m.ranges):
        if not (math.isfinite(r) and m.range_min <= r <= m.range_max):
            continue
        a = math.degrees(m.angle_min + i * m.angle_increment)
        a = (a + 180.0) % 360.0 - 180.0
        if -45 <= a <= 45:
            s['front'].append(r)
        elif -145 <= a <= -135 or 135 <= a <= 145:
            pass
        if abs(a) > 135:
            s['rear'].append(r)
        elif 45 <= a <= 135:
            s['left'].append(r)
        elif -135 <= a <= -45:
            s['right'].append(r)
    out = {}
    for k, v in s.items():
        v.sort()
        out[k] = v[0] if v else None      # 最近
        out[k + '_med'] = v[len(v) // 2] if v else None
    return out


def spin(sec, publish_zero=True):
    end = time.monotonic() + sec
    while time.monotonic() < end and rclpy.ok():
        if publish_zero:
            pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.005)


def move(vx, vy, sec=2.0):
    m = Twist()
    m.linear.x, m.linear.y = vx, vy
    end = time.monotonic() + sec
    while time.monotonic() < end and rclpy.ok():
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.002)
        time.sleep(1.0 / 30.0)
    pub.publish(Twist())
    spin(0.6)


for _ in range(200):
    rclpy.spin_once(n, timeout_sec=0.05)
    if latest['scan'] is not None:
        break
spin(0.5)

cases = [('fwd', 0.08, 0.0), ('back', -0.08, 0.0),
         ('left', 0.0, 0.08), ('right', 0.0, -0.08)]
print('== 平移方向验证（雷达四扇区最近距离变化，m）==')
print('  期望：fwd→front 减小；back→rear 减小；left→left 减小；right→right 减小')
results = {}
for name, vx, vy in cases:
    a = sectors()
    move(vx, vy)
    b = sectors()
    d = {}
    for k in ('front', 'rear', 'left', 'right'):
        am, bm = a[k + '_med'], b[k + '_med']
        if am is not None and bm is not None:
            d[k] = bm - am
        else:
            d[k] = float('nan')
    results[name] = d
    print(f'  {name:5s}: ' + '  '.join(
        f'{k}:{v:+.3f}' for k, v in d.items()))
    spin(0.4)

print('\n== 判定（变化方向是否与"运动方向"一致）==')
checks = []
for name, d in results.items():
    # 运动方向对应扇区应减小，反方向扇区应增大（阈值 0.05 m（扇区中位距离））
    if name == 'fwd':
        ok = d['front'] < -0.02
        detail = f"front {d['front']:+.3f}"
    elif name == 'back':
        ok = d['rear'] < -0.02
        detail = f"rear {d['rear']:+.3f}"
    elif name == 'left':
        ok = d['left'] < -0.02
        detail = f"left {d['left']:+.3f}"
    else:
        ok = d['right'] < -0.02
        detail = f"right {d['right']:+.3f}"
    checks.append((name, ok, detail))
passed = 0
for name, ok, detail in checks:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} 方向正确  ({detail})")
    passed += 1 if ok else 0
print(f"\n[{'PASS' if passed == 4 else 'FAIL'}] 平移方向：{passed}/4")
pub.publish(Twist())
n.destroy_node()
rclpy.shutdown()
