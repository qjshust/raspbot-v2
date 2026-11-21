# -*- coding: utf-8 -*-
"""快速自检：WitMotion 帧解析 + ImuSample 兼容性（不需要 ROS / 硬件）。"""
import sys, struct, math
sys.path.insert(0, r"E:\code\raspberrypi\ros2_ws\src\raspbot_imu")
sys.stdout.reconfigure(encoding="utf-8", errors="replace")

import numpy as np
from raspbot_imu.attitude import ImuSample
from raspbot_imu.sources import WitMotionParser


def frame(kind: int, x: int, y: int, z: int) -> bytes:
    body = bytes([0x55, kind]) + struct.pack("<3h", x, y, z) + b"\x00\x00"
    return body + bytes([sum(body[:10]) & 0xFF])


p = WitMotionParser(gyro_fs_dps=2000.0, accel_fs_g=16.0)

# 1) 角度帧：Z = 8192 -> 45°
p.feed(frame(0x53, 0, 0, 8192))
assert p.seen_angle and abs(p.angle_deg[2] - 45.0) < 1e-9, p.angle_deg
print("angle 0x53  ->", p.angle_deg.round(4), "seen_angle", p.seen_angle)

# 2) 角速度帧：Z = 16384 -> fs/2 = 1000 dps = 17.45 rad/s
p.feed(frame(0x52, 0, 0, 16384))
print("gyro  0x52  ->", p.gyro.round(4), "rad/s (期望 z≈17.45)")
assert abs(p.gyro[2] - math.radians(1000.0)) < 1e-6

# 3) 加速度帧
p.feed(frame(0x51, 0, 0, 2048))
print("accel 0x51  ->", p.accel.round(4), "m/s^2 (期望 z≈9.81)")

# 4) 前置垃圾字节 + 坏校验，仍应重新同步
bad = b"\x00\x13\x37" + frame(0x53, 0, 0, 4096)
p.feed(bad)
print("resync     ->", p.angle_deg.round(4), "(期望 z=22.5)")

# 5) ImuSample 向后兼容：3 参 / 4 参
s3 = ImuSample(1.0, [0, 0, 0], [0, 0, 9.8])
s4 = ImuSample(2.0, [0, 0, 0.1], [0, 0, 9.8], yaw_deg=22.5)
assert s3.yaw_deg is None and s4.yaw_deg == 22.5
print("ImuSample  ->", s3.yaw_deg, s4.yaw_deg, "OK")
print("ALL OK")

