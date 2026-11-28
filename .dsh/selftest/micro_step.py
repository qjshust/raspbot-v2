"""micro_step.py —— 微量步进 + 雷达自测距（无需卷尺的闭环标定）。

流程：
  1. 记录基准扫描 A；
  2. 按"有效速度"计算脉冲时长（有效速度 = max(|cmd|, deadband/pwm_per_mps)）；
  3. 发一次短脉冲（默认目标 10 cm）；
  4. 记录扫描 B，用距离变化最小二乘估计车体实际平移量：
       Δr_i ≈ -d·cos(θ_i)  →  d = -Σ cosθ·Δr / Σcos²θ（鲁棒迭代剔除外点）
  5. 输出：指令距离 / 雷达实测位移 / 比例，并给出建议的 pwm_per_mps。

用法: micro_step.py [--dist 0.10] [--cmd 0.05] [--pwm-per-mps 120] [--deadband 25]
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
ap.add_argument('--dist', type=float, default=0.10)
ap.add_argument('--cmd', type=float, default=0.05)
ap.add_argument('--pwm-per-mps', type=float, default=120.0)
ap.add_argument('--deadband', type=float, default=25.0)
ap.add_argument('--sign', type=float, default=1.0)
args = ap.parse_args()

rclpy.init()
n = Node('micro_step')
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


def spin(sec, keep_zero=True):
    end = time.monotonic() + sec
    while time.monotonic() < end and rclpy.ok():
        if keep_zero:
            pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.005)
        time.sleep(1.0 / 30.0)


def grab_scan(avg=4):
    frames = []
    t_end = time.monotonic() + 4.0
    while len(frames) < avg and time.monotonic() < t_end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.02)
        if st['scan'] is not None:
            frames.append(st['scan'])
            st['scan'] = None
    if not frames:
        return None
    nb = len(frames[0].ranges)
    out = [float('nan')] * nb
    for i in range(nb):
        vals = [f.ranges[i] for f in frames
                if math.isfinite(f.ranges[i])
                and f.range_min <= f.ranges[i] <= f.range_max]
        if vals:
            out[i] = sum(vals) / len(vals)
    m = frames[0]
    angles = []
    for i in range(nb):
        a = m.angle_min + i * m.angle_increment
        a = (a + math.pi) % (2 * math.pi) - math.pi
        angles.append(a)
    return out, angles


def measure_translation(a, b, huber_iters=3):
    """距离变化最小二乘估计沿 +x 的位移 d（米）。"""
    ra, angles = a
    rb, _ = b
    cos_t, dr = [], []
    for i in range(len(ra)):
        if math.isfinite(ra[i]) and math.isfinite(rb[i]):
            cos_t.append(math.cos(angles[i]))
            dr.append(rb[i] - ra[i])
    if len(cos_t) < 60:
        return None, 0
    keep = list(range(len(cos_t)))
    d = None
    for _ in range(huber_iters):
        num = sum(cos_t[i] * dr[i] for i in keep)
        den = sum(cos_t[i] ** 2 for i in keep)
        if den < 1e-6:
            return None, len(keep)
        d = -num / den
        resid = [abs(dr[i] + d * cos_t[i]) for i in keep]
        resid_sorted = sorted(resid)
        thr = max(0.02, 2.0 * resid_sorted[len(resid_sorted) // 2])
        keep = [i for i in keep if abs(dr[i] + d * cos_t[i]) <= thr]
    return d, len(keep)


spin(1.0)
print('== 取基准扫描 A（4 帧平均）==')
A = grab_scan()
if A is None:
    print('[FAIL] 无 /scan')
    raise SystemExit(1)
yaw0 = st['yaw']

eff = max(abs(args.cmd), args.deadband / args.pwm_per_mps)
target = args.dist
dur = target / eff
print(f'== 参数：pwm_per_mps={args.pwm_per_mps:.0f} deadband={args.deadband:.0f} '
      f'→ 有效速度 {eff:.3f} m/s')
print(f'== 目标位移 {target * 100:.1f} cm，cmd {args.cmd * args.sign:+.3f} '
      f'm/s × {dur:.2f} s')
m = Twist()
m.linear.x = args.cmd * args.sign
t0 = time.monotonic()
while time.monotonic() - t0 < dur and rclpy.ok():
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.002)
    time.sleep(1.0 / 30.0)
pub.publish(Twist())
spin(0.8)

print('== 取扫描 B ==')
B = grab_scan()
yaw1 = st['yaw']
dyaw = (yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi
d_est, npts = measure_translation(A, B)

print('\n== 结果 ==')
print(f'  指令位移      : {target * 100:+.1f} cm（{dur:.2f} s @ {eff:.3f} m/s 有效速度）')
print(f'  IMU 偏航变化  : {math.degrees(dyaw):+.1f}°'
      f'{"  ⚠ 偏转较大，位移估计可能偏" if abs(dyaw) > 0.15 else ""}')
if d_est is None:
    print('  雷达实测位移  : 估计失败（有效点不足）')
else:
    print(f'  雷达实测位移  : {d_est * 100:+.1f} cm（沿车头方向，内点 {npts} 个）')
    if abs(target) > 1e-6:
        ratio = d_est / target
        print(f'  实测/指令     : {ratio:.2f}x')
        pwm_now = args.pwm_per_mps
        # 实际速度偏大 → pwm_per_mps 应调小（成比例）
        suggest = pwm_now / ratio if ratio > 0.2 else None
        if suggest:
            print(f'  建议 pwm_per_mps ≈ {suggest:.0f}'
                  f'（当前 {pwm_now:.0f}，偏差 {(ratio - 1) * 100:+.0f}%）')
pub.publish(Twist())
n.destroy_node()
rclpy.shutdown()
