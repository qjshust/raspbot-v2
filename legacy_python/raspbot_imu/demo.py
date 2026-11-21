"""离线演示：合成数据 -> 姿态估计 -> 误差统计（可选曲线图）。

用法::

    imu_demo --mode yaw --duration 12 --out imu_attitude_demo.png
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from .attitude import (
    MahonyAHRS,
    YawEstimator,
    estimate_gyro_bias,
    quat_yaw,
    wrap_pi,
)
from .sources import MockSource

RATE_HZ = 100.0


def run(duration: float = 12.0, mode: str = 'yaw', plot: bool = True,
        out: str = 'imu_attitude_demo.png') -> dict:
    """跑一段合成轨迹并返回误差统计。"""
    if mode not in ('yaw', 'mahony'):
        raise ValueError('mode 必须是 yaw 或 mahony')

    # 标定源与运行源分开，保证轨迹从 t=0 开始
    calib_source = MockSource(rate_hz=RATE_HZ, seed=1)
    bias = estimate_gyro_bias([calib_source.read().gyro for _ in range(150)])
    source = MockSource(rate_hz=RATE_HZ, seed=2)

    if mode == 'mahony':
        estimator = MahonyAHRS(kp=0.8, ki=0.02)
        estimator.bias = bias
    else:
        estimator = YawEstimator(bias=bias)

    times: list[float] = []
    truth: list[float] = []
    estimated: list[float] = []
    while source.time < duration:
        sample = source.read()
        if mode == 'mahony':
            estimator.update(sample.gyro, sample.accel, 1.0 / RATE_HZ)
            estimated.append(quat_yaw(estimator.q))
        else:
            estimated.append(estimator.update(sample.gyro, sample.accel, 1.0 / RATE_HZ))
        times.append(sample.t)
        truth.append(source.true_yaw)

    t = np.asarray(times)
    est_unwrapped = np.unwrap(np.asarray(estimated))
    truth_unwrapped = np.unwrap(np.asarray(truth))
    error = np.array([wrap_pi(e - g) for e, g in zip(est_unwrapped, truth_unwrapped)])

    rmse_deg = math.degrees(float(np.sqrt(np.mean(error ** 2))))
    max_deg = math.degrees(float(np.max(np.abs(error))))

    print(f'模式            : {mode}')
    print(f'标定零偏 (rad/s): x={bias[0]:+.5f} y={bias[1]:+.5f} z={bias[2]:+.5f}')
    print(f'时长            : {duration:.1f} s，{len(t)} 帧')
    print(f'航向 RMSE       : {rmse_deg:.3f} °')
    print(f'航向最大误差    : {max_deg:.3f} °')
    print(f'终点误差        : {math.degrees(error[-1]):+.3f} °')

    if plot:
        _plot(t, truth_unwrapped, est_unwrapped, error, out)

    return {'rmse_deg': rmse_deg, 'max_deg': max_deg, 'frames': len(t)}


def _plot(t: np.ndarray, truth: np.ndarray, est: np.ndarray,
          error: np.ndarray, out: str) -> None:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('未安装 matplotlib，跳过绘图')
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax1.plot(t, np.degrees(truth), 'k--', label='true yaw')
    ax1.plot(t, np.degrees(est), 'C0', label='estimated yaw')
    ax1.set_ylabel('yaw (deg)')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best')

    ax2.plot(t, np.degrees(error), 'C3')
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('error (deg)')
    ax2.grid(True, alpha=0.3)

    fig.suptitle('RASPBOT-V2 IMU yaw estimation (synthetic data)')
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f'曲线已保存：{out}')


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description='IMU 姿态解算离线演示')
    parser.add_argument('--duration', type=float, default=12.0)
    parser.add_argument('--mode', choices=['yaw', 'mahony'], default='yaw')
    parser.add_argument('--out', default='imu_attitude_demo.png')
    parser.add_argument('--no-plot', action='store_true')
    ns = parser.parse_args(args)
    run(duration=ns.duration, mode=ns.mode, plot=not ns.no_plot, out=ns.out)


if __name__ == '__main__':
    main()
