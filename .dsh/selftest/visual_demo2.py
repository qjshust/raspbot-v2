"""visual_demo2.py —— 编号动作序列 v2（蜂鸣器计数 + LED 颜色 + 大屏输出）。

动作编号（蜂鸣器哔 N 声 + LED 颜色）：
  1 前进（绿）/ 2 后退（红）/ 3 左移（蓝）/ 4 右移（黄）/ 5 左转（紫）/ 6 右转（青）
每段：LED+哔声提示 → 动作 2.0 s（旋转 1.5 s）→ 停车 2.2 s
全程前向雷达近距保护（<0.12 m 立即停并结束）。
用法: visual_demo2.py [--lin 0.08] [--yaw 0.20]
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
from std_msgs.msg import Bool, ColorRGBA

ap = argparse.ArgumentParser()
ap.add_argument('--lin', type=float, default=0.08)
ap.add_argument('--yaw', type=float, default=0.20)
args = ap.parse_args()

rclpy.init()
n = Node('visual_demo2')
pub = n.create_publisher(Twist, '/cmd_vel_teleop', 10)
buzzer = n.create_publisher(Bool, '/buzzer', 10)
led = n.create_publisher(ColorRGBA, '/led', 10)
st = {'odom': None, 'yaw': None, 'front': float('inf'), 'abort': False}

n.create_subscription(
    Odometry, '/odom_raw',
    lambda m: st.__setitem__(
        'odom', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)


def imu_cb(m):
    q = m.orientation
    st['yaw'] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def scan_cb(m):
    mn = float('inf')
    for i, r in enumerate(m.ranges):
        if not (math.isfinite(r) and m.range_min <= r <= m.range_max):
            continue
        a = math.degrees(m.angle_min + i * m.angle_increment)
        a = (a + 180.0) % 360.0 - 180.0
        if abs(a) <= 75.0 and r < mn:
            mn = r
    st['front'] = mn
    if mn < 0.12:
        st['abort'] = True


n.create_subscription(Imu, '/imu/data', imu_cb, qos_profile_sensor_data)
n.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data)


def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end and rclpy.ok() and not st['abort']:
        pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.005)
        time.sleep(1.0 / 30.0)


def beep(n_times):
    for _ in range(n_times):
        buzzer.publish(Bool(data=True))
        for _ in range(8):
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(0.02)
        buzzer.publish(Bool(data=False))
        for _ in range(10):
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(0.02)


def set_led(r, g, b):
    led.publish(ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0))
    rclpy.spin_once(n, timeout_sec=0.02)


def seg(vx, vy, wz, sec):
    a = dict(st)
    m = Twist()
    m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
    end = time.monotonic() + sec
    while time.monotonic() < end and rclpy.ok() and not st['abort']:
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.002)
        time.sleep(1.0 / 30.0)
    pub.publish(Twist())
    spin(0.5)
    b = dict(st)
    do = (b['odom'][0] - a['odom'][0], b['odom'][1] - a['odom'][1]) \
        if a['odom'] and b['odom'] else (0, 0)
    dyaw = ((b['yaw'] - a['yaw'] + math.pi) % (2 * math.pi) - math.pi) \
        if a['yaw'] is not None else 0.0
    return do, dyaw


CODES = {
    '1': ('前进 →（绿）', (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
    '2': ('后退 ←（红）', (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
    '3': ('左移 ↰（蓝）', (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    '4': ('右移 ↱（黄）', (1.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
    '5': ('左转 ⟲（紫）', (0.6, 0.0, 1.0), (0.0, 0.0, 1.0)),
    '6': ('右转 ⟳（青）', (0.0, 1.0, 1.0), (0.0, 0.0, -1.0)),
}

spin(1.0)
print('=' * 60, flush=True)
print('== 编号动作序列 v2：蜂鸣器哔声编号（1 声=1 号动作…）+ LED 颜色 ==', flush=True)
print('== 每个动作前先亮灯+哔声，随后车动作 2 秒，再停 2 秒 ==', flush=True)
print('=' * 60, flush=True)

for code in ('1', '2', '3', '4', '5', '6'):
    if st['abort']:
        print('!! 前向近距保护触发，提前结束', flush=True)
        break
    name, color, (fx, fy, fz) = CODES[code]
    vx = args.lin * fx if fz == 0 else 0.0
    vy = args.lin * fy if fz == 0 else 0.0
    wz = args.yaw * fz
    dur = 1.5 if fz != 0 else 2.0
    print(f'\n>>> {code} {name}：哔 {int(code)} 声 ...', flush=True)
    set_led(*color)
    beep(int(code))
    print(f'    动作 {dur:.1f}s：vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}',
          flush=True)
    do, dyaw = seg(vx, vy, wz, dur)
    print(f'    结束（停车）：odom Δ=({do[0]:+.3f},{do[1]:+.3f}) m，'
          f'IMU Δyaw={math.degrees(dyaw):+.0f}°', flush=True)
    spin(2.2)

set_led(0.0, 0.0, 0.0)
pub.publish(Twist())
print('\n' + '=' * 60, flush=True)
print('== 序列结束（已停车，LED 熄灭）==', flush=True)
print('== 请告诉我：每个编号动作，车实际往哪走了 ==', flush=True)
print('=' * 60, flush=True)
n.destroy_node()
rclpy.shutdown()
