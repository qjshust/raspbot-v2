"""rot_trace.py —— 单段旋转实时轨迹（cmd/safe/陀螺/偏航），用于定位旋转行为。

用法: rot_trace.py [--wz 0.5] [--sec 2.0] [--sign 1]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

rclpy.init()
ap = argparse.ArgumentParser()
ap.add_argument('--wz', type=float, default=0.5)
ap.add_argument('--sec', type=float, default=2.0)
ap.add_argument('--sign', type=float, default=1.0)
args = ap.parse_args()

n = Node('rot_trace')
pub = n.create_publisher(Twist, '/cmd_vel_teleop', 10)
st = {'yaw': None, 'wz_gyro': 0.0, 'safe': 0.0, 'odom_theta': 0.0}


def imu_cb(m):
    q = m.orientation
    st['yaw'] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    st['wz_gyro'] = m.angular_velocity.z


def safe_cb(m):
    st['safe'] = m.angular.z


n.create_subscription(Imu, '/imu/data', imu_cb, qos_profile_sensor_data)
n.create_subscription(Twist, '/cmd_vel_safe', safe_cb, 10)
for _ in range(200):
    rclpy.spin_once(n, timeout_sec=0.05)
    if st['yaw'] is not None:
        break
yaw0 = st['yaw']
wz_cmd = args.wz * args.sign
print(f'== 旋转轨迹：cmd wz={wz_cmd:+.2f} rad/s × {args.sec:.1f}s')
print(f'   yaw0={yaw0:+.3f} rad')
m = Twist()
m.angular.z = wz_cmd
t0 = time.monotonic()
last = -1.0
gyro_int = 0.0
while time.monotonic() - t0 < args.sec and rclpy.ok():
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.002)
    now = time.monotonic() - t0
    gyro_int += st['wz_gyro'] * 0.02
    if int(now * 5) != int(last * 5):
        last = now
        dyaw = (st['yaw'] - yaw0 + math.pi) % (2 * math.pi) - math.pi
        print(f'  t={now:4.1f}s safe_wz={st["safe"]:+.3f} '
              f'gyro_z={st["wz_gyro"]:+.3f} yaw={st["yaw"]:+.3f} '
              f'Δyaw={dyaw:+.3f}')
    time.sleep(1.0 / 30.0)
pub.publish(Twist())
t_end = time.monotonic() + 0.5
while time.monotonic() < t_end:
    pub.publish(Twist())
    rclpy.spin_once(n, timeout_sec=0.01)
    time.sleep(1.0 / 30.0)
dyaw = (st['yaw'] - yaw0 + math.pi) % (2 * math.pi) - math.pi
print(f'== 结果: Δyaw(IMU)={dyaw:+.3f} rad ({math.degrees(dyaw):+.1f}°)，'
      f'指令 {wz_cmd * args.sec:+.3f} rad ({math.degrees(wz_cmd * args.sec):+.1f}°)，'
      f'陀螺积分≈{gyro_int:+.3f} rad')
n.destroy_node()
rclpy.shutdown()
