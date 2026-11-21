#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HWT101 台架静态检查：采一段角度数据，给出噪声、漂移和帧率。

用途：装车前先在桌上放平跑 60 秒，判断这颗模块「静得下来」还是「抖得厉害」，
把结果记下来作为以后换安装方式/加减震的对照基线。

用法（树莓派上，pyserial 已随驱动装好）：
    python3 hwt101_static_check.py --seconds 60          # 默认 /dev/raspbot_imu（udev 固定名）
    python3 hwt101_static_check.py --port /dev/ttyUSB0 --seconds 60 --baud 115200
    python3 hwt101_static_check.py --selftest          # 无硬件，验证脚本本身

判定参考（地面车，静止）：
    角度标准差 < 0.05° 且 漂移 < 0.10°/min  → 很好
    标准差 < 0.20° 且 漂移 < 0.50°/min     → 可用，SLAM 前建议 10–30Hz 低通
    再差就要查：是否水平、是否被线束拽着、附近有没有电机/电源线
"""

from __future__ import annotations

import argparse
import math
import statistics
import struct
import sys
import time


class Parser:
    """维特智能 0x55 帧（11 字节，小端 int16，校验 = 前 10 字节和的低 8 位）。"""

    ACCEL, GYRO, ANGLE = 0x51, 0x52, 0x53

    def __init__(self) -> None:
        self.buf = bytearray()
        self.frames = 0
        self.yaw_deg = None
        # 本次 feed() 解析出的全部角度值（一个串口 read 通常含多帧）
        self.recent: list[float] = []

    def feed(self, data: bytes) -> int:
        """喂入字节流，返回本次解析出的角度帧数（顺带更新 yaw_deg / recent）。"""
        self.buf.extend(data)
        self.recent = []
        n = 0
        while True:
            i = self.buf.find(b"\x55")
            if i < 0:
                self.buf.clear()
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < 11:
                break
            f = bytes(self.buf[:11])
            if (sum(f[:10]) & 0xFF) != f[10]:
                del self.buf[0]
                continue
            del self.buf[:11]
            self.frames += 1
            if f[1] == self.ANGLE:
                # 角度是对 full-scale 32768 归一化，Z 轴即航向
                z = struct.unpack("<3h", f[2:8])[2]
                self.yaw_deg = z * (180.0 / 32768.0)
                self.recent.append(self.yaw_deg)
                n += 1
        return n


def slope_per_min(t: list[float], y: list[float]) -> float:
    """最小二乘斜率，换算成 °/min。"""
    n = len(t)
    if n < 3:
        return 0.0
    tm = sum(t) / n
    ym = sum(y) / n
    num = sum((ti - tm) * (yi - ym) for ti, yi in zip(t, y))
    den = sum((ti - tm) ** 2 for ti in t)
    if den <= 0:
        return 0.0
    return num / den * 60.0


def report(t0: float, samples: list[tuple[float, float]], frames: int) -> int:
    if len(samples) < 10:
        print(f"只收到 {len(samples)} 个角度帧，先检查波特率 / 接线。")
        return 1
    ts = [s[0] - t0 for s in samples]
    ys = [s[1] for s in samples]
    span = ts[-1] - ts[0]
    rate = len(samples) / span if span > 0 else float("nan")

    std = statistics.pstdev(ys)
    drift = slope_per_min(ts, ys)
    delta = ys[-1] - ys[0]

    print(f"总帧数     : {frames}（其中角度帧 {len(samples)}）")
    print(f"时间跨度   : {span:.1f} s，角度帧率 {rate:.1f} Hz")
    print(f"航向均值   : {statistics.mean(ys):+.3f}°")
    print(f"标准差     : {std:.4f}°   （峰峰值 {max(ys) - min(ys):.4f}°）")
    print(f"线性漂移   : {drift:+.4f} °/min   （首尾差 {delta:+.4f}°）")

    if std < 0.05 and abs(drift) < 0.10:
        print("判定：很好——这颗模块静止状态可以直接用。")
    elif std < 0.20 and abs(drift) < 0.50:
        print("判定：可用。建议给航向加 10–30Hz 低通，或让 EKF 更信雷达。")
    else:
        print("判定：偏差偏大。依次检查：模块是否水平、线束是否在拽、"
              "是否靠近电机线/电源、载板是否松动。")
    if rate < 50:
        print(f"提示：只有 {rate:.0f} Hz。用维特上位机把输出速率从 10Hz 改到 100Hz。")
    return 0


def run_serial(args) -> int:
    try:
        import serial  # pyserial
    except ImportError:
        print("缺少 pyserial：pip install pyserial")
        return 2
    try:
        port = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as exc:  # noqa: BLE001
        print(f"打不开串口 {args.port}: {exc}")
        print("检查：enable_uart=1、console 是否占用、用户是否在 dialout 组。")
        return 2

    p = Parser()
    samples: list[tuple[float, float]] = []
    t0 = time.monotonic()
    print(f"采样 {args.seconds:.0f} s，保持模块静止、车不要动……")
    try:
        while time.monotonic() - t0 < args.seconds:
            data = port.read(256)
            if not data:
                continue
            if p.feed(data):
                # 一次 read 可能带回多帧：按帧计数，时间戳用本批的到达时刻近似，
                # 否则帧率会被 read 轮询频率（~20 Hz）严重低估。
                now = time.monotonic()
                samples.extend((now, yaw) for yaw in p.recent)
    finally:
        port.close()
    return report(t0, samples, p.frames)


def selftest() -> int:
    """无硬件自检：合成 100Hz、10 秒、带噪声与 0.5°/min 漂移的数据。"""
    p = Parser()
    samples: list[tuple[float, float]] = []
    t0 = 0.0
    for i in range(1000):
        t = i * 0.01
        yaw = 30.0 + 0.5 * (t / 60.0) + 0.02 * math.sin(i / 7.0)
        raw = int(round(yaw / (180.0 / 32768.0)))
        body = bytes([0x55, 0x53]) + struct.pack("<3h", 0, 0, raw) + b"\x00\x00"
        p.feed(body + bytes([sum(body[:10]) & 0xFF]))
        if p.yaw_deg is not None:
            samples.append((t0 + t, p.yaw_deg))
    rc = report(t0, samples, p.frames)
    print("selftest: 期望 ~100Hz、均值 ≈30°、漂移 ≈+0.5°/min")
    return rc


def main() -> int:
    ap = argparse.ArgumentParser(description="HWT101 静态噪声/漂移检查")
    ap.add_argument("--port", default="/dev/raspbot_imu")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    return selftest() if args.selftest else run_serial(args)


if __name__ == "__main__":
    sys.exit(main())

