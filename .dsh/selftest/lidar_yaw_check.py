"""lidar_yaw_check.py —— 用雷达圆周互相关独立测量车体旋转角。

原理：雷达固定随车体旋转；车体转 Δθ 时，同一房间的扫描特征整体平移 −Δθ。
对两次扫描做圆周互相关，取峰值偏移 = 车体转角（独立于 IMU/odom 的真值）。
对比：指令角 / IMU 偏航变化 / 雷达互相关角。

用法: lidar_yaw_check.py [--wz 0.2] [--sec 2.0] [--sign 1]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan

ap = argparse.ArgumentParser()
ap.add_argument('--wz', type=float, default=0.2)
ap.add_argument('--sec', type=float, default=2.0)
ap.add_argument('--sign', type=float, default=1.0)
args = ap.parse_args()

rclpy.init()
n = Node('lidar_yaw_check')
pub = n.create_publisher(Twist, '/cmd_vel_teleop', 10)
st = {'scan': None, 'yaw': None}
n.create_subscription(LaserScan, '/scan',
                      lambda m: st.__setitem__('scan', m),
                      qos_profile_sensor_data)
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
        rclpy.spin_once(n, timeout_sec=0.01)
        time.sleep(1.0 / 30.0)


def grab_scan(avg=3):
    """取 avg 帧平均（按束平均，无效点跳过）。"""
    frames = []
    t_end = time.monotonic() + 3.0
    while len(frames) < avg and time.monotonic() < t_end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.02)
        if st['scan'] is not None:
            frames.append(st['scan'])
            st['scan'] = None
    if not frames:
        return None
    nb = len(frames[0].ranges)
    summ = [0.0] * nb
    cnt = [0] * nb
    for f in frames:
        for i, r in enumerate(f.ranges):
            if math.isfinite(r) and f.range_min <= r <= f.range_max:
                summ[i] += r
                cnt[i] += 1
    return [summ[i] / cnt[i] if cnt[i] else float('nan') for i in range(nb)]


def circ_shift(a, b, max_deg=170.0, step=1.0):
    """圆周互相关：返回 b 相对 a 的最佳角位移（度）。"""
    nb = len(a)
    deg_per_beam = 360.0 / nb
    best_off, best_score = None, -1e18
    max_off = int(max_deg / deg_per_beam)
    for off in range(-max_off, max_off + 1, max(1, int(step / deg_per_beam))):
        s = 0.0
        cnt = 0
        for i in range(nb):
            j = (i + off) % nb
            va, vb = a[i], b[j]
            if not (math.isfinite(va) and math.isfinite(vb)):
                continue
            d = va - vb
            s -= d * d          # 差值平方和最小 = 相关最大
            cnt += 1
        if cnt < nb * 0.3:
            continue
        score = -s / cnt
        if score > best_score:
            best_score, best_off = score, off
    return None if best_off is None else best_off * deg_per_beam


spin(1.0)
print('== 静止取基准扫描 ==')
a = grab_scan()
if a is None:
    print('[FAIL] 无 /scan')
    raise SystemExit(1)
yaw0 = st['yaw']
wz = args.wz * args.sign
print(f'== 旋转：cmd wz={wz:+.2f} rad/s × {args.sec:.1f}s '
      f'（期望 {wz * args.sec:+.2f} rad = {math.degrees(wz * args.sec):+.1f}°）')
m = Twist()
m.angular.z = wz
end = time.monotonic() + args.sec
while time.monotonic() < end and rclpy.ok():
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.002)
    time.sleep(1.0 / 30.0)
pub.publish(Twist())
spin(1.0)
b = grab_scan()
yaw1 = st['yaw']
dyaw_imu = (yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi
shift = circ_shift(a, b)
cmd_deg = math.degrees(wz * args.sec)
print('\n== 结果 ==')
print(f'  指令角      : {cmd_deg:+.1f}°')
print(f'  IMU 偏航变化: {math.degrees(dyaw_imu):+.1f}°')
if shift is None:
    print('  雷达互相关  : 失败（特征不足）')
else:
    print(f'  雷达互相关  : {-shift:+.1f}°  （车体转角；扫描特征向相反方向平移）')
    print(f'  指令/雷达比 : '
          f'{(cmd_deg / -shift) if abs(shift) > 1e-6 else float("nan"):.2f}x')
    print(f'  IMU/雷达差  : {math.degrees(dyaw_imu) - (-shift):+.1f}°')
n.destroy_node()
rclpy.shutdown()
