"""visual_demo.py —— 目视方向确认序列（每段 1 秒，段间停 1 秒）。

编号动作：1 前进 / 2 后退 / 3 左移 / 4 右移 / 5 左转 / 6 右转
每段同时记录 odom 与 IMU 偏航，作为记录（方向真值由观察者提供）。
用法: visual_demo.py [--speed 0.10] [--yaw 0.30] [--sec 1.0]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

ap = argparse.ArgumentParser()
ap.add_argument('--speed', type=float, default=0.10)
ap.add_argument('--yaw', type=float, default=0.30)
ap.add_argument('--sec', type=float, default=1.0)
args = ap.parse_args()

rclpy.init()
n = Node('visual_demo')
pub = n.create_publisher(Twist, '/cmd_vel_teleop', 10)
st = {'odom': None, 'yaw': None}
n.create_subscription(
    Odometry, '/odom_raw',
    lambda m: st.__setitem__('odom', (m.pose.pose.position.x,
                                     m.pose.pose.position.y)), 10)
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


def seg(vx, vy, wz, sec):
    a = dict(st)
    m = Twist()
    m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
    end = time.monotonic() + sec
    while time.monotonic() < end and rclpy.ok():
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.002)
        time.sleep(1.0 / 30.0)
    pub.publish(Twist())
    spin(0.8)
    b = dict(st)
    do = (b['odom'][0] - a['odom'][0], b['odom'][1] - a['odom'][1]) \
        if a['odom'] and b['odom'] else (0, 0)
    dyaw = ((b['yaw'] - a['yaw'] + math.pi) % (2 * math.pi) - math.pi) \
        if a['yaw'] is not None else 0
    return do, dyaw


spin(1.0)
print('== 目视方向确认序列（每段 1.0 s，段间停 1 s）==')
seq = [
    ('1 前进', args.speed, 0.0, 0.0),
    ('2 后退', -args.speed, 0.0, 0.0),
    ('3 左移', 0.0, args.speed, 0.0),
    ('4 右移', 0.0, -args.speed, 0.0),
    ('5 左转', 0.0, 0.0, args.yaw),
    ('6 右转', 0.0, 0.0, -args.yaw),
]
print('  请观察车并记录每段实际方向（正前/正后/左/右/逆时针/顺时针）')
for name, vx, vy, wz in seq:
    print(f'>> {name} 开始 ...', flush=True)
    do, dyaw = seg(vx, vy, wz, args.sec)
    print(f'   {name} 结束: odom Δ=({do[0]:+.3f},{do[1]:+.3f}) m, '
          f'IMU Δyaw={dyaw:+.2f} rad ({math.degrees(dyaw):+.0f}°)', flush=True)
    spin(1.0)
print('== 序列结束（已停车）==')
pub.publish(Twist())
n.destroy_node()
rclpy.shutdown()
