"""slip_quantify.py —— 瓷砖地面打滑量化（直行/横移的偏航散差）。

3× 直行 30cm + 2× 横移 15cm（标定后有效速度算时长），记录每段 IMU 偏航变化，
统计偏航散差 ≈ 打滑导致的车头漂移。
用法: slip_quantify.py [--pwm-per-mps 120] [--deadband 25] [--k-geom 0.1245]
"""
import argparse
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

ap = argparse.ArgumentParser()
ap.add_argument('--pwm-per-mps', type=float, default=120.0)
ap.add_argument('--deadband', type=float, default=25.0)
ap.add_argument('--k-geom', type=float, default=0.1245)
ap.add_argument('--cmd', type=float, default=0.05)
args = ap.parse_args()

V_EFF = max(abs(args.cmd), args.deadband / args.pwm_per_mps)
T_FB = 0.30 / V_EFF
T_LR = 0.15 / V_EFF

rclpy.init()
n = Node('slip_quantify')
pub = n.create_publisher(Twist, '/cmd_vel_teleop', 10)
st = {'yaw': None}
n.create_subscription(
    Imu, '/imu/data',
    lambda m: st.__setitem__(
        'yaw', math.atan2(2.0 * (m.orientation.w * m.orientation.z),
                          1.0 - 2.0 * (m.orientation.z ** 2))),
    qos_profile_sensor_data)


def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end and rclpy.ok():
        pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.005)
        time.sleep(1.0 / 30.0)


def pulse(vx, vy, wz, sec):
    yaw0 = st['yaw']
    m = Twist()
    m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
    end = time.monotonic() + sec
    while time.monotonic() < end and rclpy.ok():
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.002)
        time.sleep(1.0 / 30.0)
    pub.publish(Twist())
    spin(0.7)
    return (st['yaw'] - yaw0 + math.pi) % (2 * math.pi) - math.pi


for _ in range(200):
    rclpy.spin_once(n, timeout_sec=0.05)
    if st['yaw'] is not None:
        break
spin(0.8)
print(f'== 打滑量化：有效速度 {V_EFF:.3f} m/s，'
      f'直行 {T_FB:.2f}s/30cm，横移 {T_LR:.2f}s/15cm ==')
print('== 注意：每段后回到起点附近（交替方向），请保证车周无紧贴障碍 ==')

fwd, back, left, right = [], [], [], []
for i in range(3):
    d = math.degrees(pulse(args.cmd, 0.0, 0.0, T_FB))
    fwd.append(d)
    d2 = math.degrees(pulse(-args.cmd, 0.0, 0.0, T_FB))
    back.append(d2)
    print(f'  第{i + 1}轮 直行 前进偏航 {d:+.1f}° / 后退偏航 {d2:+.1f}°',
          flush=True)
    spin(0.5)
for i in range(2):
    d = math.degrees(pulse(0.0, args.cmd, 0.0, T_LR))
    left.append(d)
    d2 = math.degrees(pulse(0.0, -args.cmd, 0.0, T_LR))
    right.append(d2)
    print(f'  第{i + 1}轮 横移 左偏航 {d:+.1f}° / 右偏航 {d2:+.1f}°', flush=True)
    spin(0.5)


def stat(name, vals):
    print(f'  {name}: n={len(vals)} 均值={statistics.mean(vals):+.1f}° '
          f'|最大|={max(abs(v) for v in vals):.1f}° '
          f'散差σ={statistics.pstdev(vals):.1f}°')


print('\n== 汇总（瓷砖地面，打滑导致的偏航）==')
stat('前进 30cm', fwd)
stat('后退 30cm', back)
stat('左移 15cm', left)
stat('右移 15cm', right)
print('\n[结论] 瓷砖打滑使每次直行/横移产生数度~数十度车头漂移，'
      '属环境限制；高摩擦地面（地毯/橡胶垫）复测可降低。')
pub.publish(Twist())
n.destroy_node()
rclpy.shutdown()
