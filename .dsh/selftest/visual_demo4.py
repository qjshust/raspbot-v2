"""visual_demo4.py —— 标定后编号动作序列（有效速度/有效角速度算时长）。

标定数据（2026-11-28 实测 + 用户目视确认）：
  pwm_per_mps=120, deadband_pwm=25, k_geom=0.1245
  → 平移最小有效速度 = 25/120 = 0.2083 m/s
  → 旋转最小有效角速度 = 0.2083/0.1245 = 1.673 rad/s（96°/s）

动作（哔 N 声 + LED 颜色）：
  1 前进 30cm（绿）/ 2 后退 30cm（红）/ 3 左移 15cm（蓝）/ 4 右移 15cm（黄）
  5 左转 90°（紫）/ 6 右转 90°（青）
用法: visual_demo4.py [--dist-fb 0.30] [--dist-lr 0.15] [--deg 90]
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
ap.add_argument('--dist-fb', type=float, default=0.30)
ap.add_argument('--dist-lr', type=float, default=0.15)
ap.add_argument('--deg', type=float, default=90.0)
ap.add_argument('--pwm-per-mps', type=float, default=120.0)
ap.add_argument('--deadband', type=float, default=25.0)
ap.add_argument('--k-geom', type=float, default=0.1245)
ap.add_argument('--cmd', type=float, default=0.05)
args = ap.parse_args()

V_EFF = max(abs(args.cmd), args.deadband / args.pwm_per_mps)     # 0.2083
W_EFF = max(abs(args.cmd), args.deadband / args.pwm_per_mps / args.k_geom)  # 1.673
T_FB = args.dist_fb / V_EFF
T_LR = args.dist_lr / V_EFF
T_ROT = math.radians(args.deg) / W_EFF

rclpy.init()
n = Node('visual_demo4')
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
        if abs(a) <= 60.0 and r < mn:
            mn = r
    st['front'] = mn
    if mn < 0.10:
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
        for _ in range(6):
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(0.02)
        buzzer.publish(Bool(data=False))
        for _ in range(8):
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
    '1': ('前进 30cm（绿）', (0.0, 1.0, 0.0), (args.cmd, 0.0, 0.0), T_FB),
    '2': ('后退 30cm（红）', (1.0, 0.0, 0.0), (-args.cmd, 0.0, 0.0), T_FB),
    '3': ('左移 15cm（蓝）', (0.0, 0.0, 1.0), (0.0, args.cmd, 0.0), T_LR),
    '4': ('右移 15cm（黄）', (1.0, 1.0, 0.0), (0.0, -args.cmd, 0.0), T_LR),
    '5': ('左转 90°（紫）', (0.6, 0.0, 1.0), (0.0, 0.0, W_EFF), T_ROT),
    '6': ('右转 90°（青）', (0.0, 1.0, 1.0), (0.0, 0.0, -W_EFF), T_ROT),
}

spin(1.0)
print('=' * 64, flush=True)
print('== 标定后动作序列：1=前进30cm 2=后退30cm 3=左移15cm 4=右移15cm '
      '5=左转90° 6=右转90° ==', flush=True)
print(f'== 有效速度 {V_EFF:.3f} m/s，有效角速度 {W_EFF:.3f} rad/s '
      f'（{math.degrees(W_EFF):.0f}°/s）==', flush=True)
print(f'== 时长：平移 {T_FB:.2f}s（30cm）/ {T_LR:.2f}s（15cm），'
      f'旋转 {T_ROT:.2f}s（90°）==', flush=True)
print('=' * 64, flush=True)
rows = []
for code in ('1', '2', '3', '4', '5', '6'):
    if st['abort']:
        print('!! 近距保护触发，提前结束', flush=True)
        break
    name, color, (vx, vy, wz), dur = CODES[code]
    print(f'\n>>> {code} {name}：哔 {int(code)} 声 ...', flush=True)
    set_led(*color)
    beep(int(code))
    do, dyaw = seg(vx, vy, wz, dur)
    print(f'    动作 {dur:.2f}s → odom Δ=({do[0]:+.3f},{do[1]:+.3f}) m，'
          f'IMU Δyaw={math.degrees(dyaw):+.0f}°', flush=True)
    rows.append((code, name, do[0], do[1], math.degrees(dyaw)))
    spin(2.0)

set_led(0.0, 0.0, 0.0)
pub.publish(Twist())
print('\n' + '=' * 64, flush=True)
print('== 序列结束（已停车）==', flush=True)
print('== 请按编号反馈：每段实际方向和大致幅度（如"1 前进约30cm"）==', flush=True)
print('=' * 64, flush=True)
n.destroy_node()
rclpy.shutdown()
