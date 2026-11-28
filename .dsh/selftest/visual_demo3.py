"""visual_demo3.py —— 编号动作序列 v3（前进/后退 30 cm，适配较小场地）。

动作编号（蜂鸣器哔 N 声 + LED 颜色）：
  1 前进 30 cm（绿）/ 2 后退 30 cm（红）/ 3 左移 15 cm（蓝）/ 4 右移 15 cm（黄）
  5 左转（紫）/ 6 右转（青）
每段：LED+哔声提示 → 动作（按距离/速度计算时长）→ 停车 2.2 s
前向雷达近距保护（<0.12 m 立即停并结束）。
用法: visual_demo3.py [--speed 0.08] [--yaw 0.20] [--dist-fb 0.30] [--dist-lr 0.15]
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
ap.add_argument('--speed', type=float, default=0.08)
ap.add_argument('--yaw', type=float, default=0.20)
ap.add_argument('--dist-fb', type=float, default=0.30)
ap.add_argument('--dist-lr', type=float, default=0.15)
args = ap.parse_args()

rclpy.init()
n = Node('visual_demo3')
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


t_fb = args.dist_fb / args.speed      # 30 cm @ 0.08 = 3.75 s
t_lr = args.dist_lr / args.speed      # 15 cm @ 0.08 = 1.875 s
t_rot = 0.30 / args.yaw               # 0.30 rad @ 0.20 = 1.5 s

CODES = {
    '1': ('前进 30cm（绿）', (0.0, 1.0, 0.0), (args.speed, 0.0, 0.0), t_fb),
    '2': ('后退 30cm（红）', (1.0, 0.0, 0.0), (-args.speed, 0.0, 0.0), t_fb),
    '3': ('左移 15cm（蓝）', (0.0, 0.0, 1.0), (0.0, args.speed, 0.0), t_lr),
    '4': ('右移 15cm（黄）', (1.0, 1.0, 0.0), (0.0, -args.speed, 0.0), t_lr),
    '5': ('左转（紫）', (0.6, 0.0, 1.0), (0.0, 0.0, args.yaw), t_rot),
    '6': ('右转（青）', (0.0, 1.0, 1.0), (0.0, 0.0, -args.yaw), t_rot),
}

spin(1.0)
print('=' * 62, flush=True)
print('== 编号动作序列 v3：1=前进30cm 2=后退30cm 3=左移15cm 4=右移15cm', flush=True)
print('== 5=左转 6=右转；每段前 LED 亮色 + 哔 N 声提示 ==', flush=True)
print(f'== 参数: 速度 {args.speed} m/s，旋转 {args.yaw} rad/s，'
      f'前向保护 0.12 m ==', flush=True)
print('=' * 62, flush=True)
rows = []
for code in ('1', '2', '3', '4', '5', '6'):
    if st['abort']:
        print('!! 前向近距保护触发，提前结束', flush=True)
        break
    name, color, (vx, vy, wz), dur = CODES[code]
    print(f'\n>>> {code} {name}：哔 {int(code)} 声 ...', flush=True)
    set_led(*color)
    beep(int(code))
    print(f'    动作 {dur:.2f}s：vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f}',
          flush=True)
    r0 = dict(st)
    do, dyaw = seg(vx, vy, wz, dur)
    front0 = r0['front']
    front1 = st['front']
    print(f'    结束：odom Δ=({do[0]:+.3f},{do[1]:+.3f}) m，'
          f'IMU Δyaw={math.degrees(dyaw):+.0f}°，'
          f'前向最近 {front0:.2f}→{front1:.2f} m', flush=True)
    rows.append((code, name, do[0], do[1], math.degrees(dyaw)))
    spin(2.2)

set_led(0.0, 0.0, 0.0)
pub.publish(Twist())
print('\n' + '=' * 62, flush=True)
print('== 汇总（指令 → odom/IMU 记录）==', flush=True)
for code, name, dx, dy, dyaw in rows:
    print(f'  {code} {name}: odom Δ=({dx:+.3f},{dy:+.3f}) m  '
          f'IMU Δyaw={dyaw:+.0f}°', flush=True)
print('== 序列结束（已停车，LED 熄灭）==', flush=True)
print('== 请按编号告诉我车实际往哪走了 ==', flush=True)
print('=' * 62, flush=True)
n.destroy_node()
rclpy.shutdown()
