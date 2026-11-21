"""姿态解算核心：四元数工具、Mahony 互补滤波、地面车 yaw 估计。

设计约束（REP-103）：
- 机体坐标系：x 向前、y 向左、z 向上。
- 四元数 q = [w, x, y, z]，表示 body -> world 的旋转。
- 陀螺单位 rad/s，加速度计单位 m/s^2；静止时加速度计读数约为 (0, 0, +g)。

本模块不依赖 ROS 与硬件，可单独导入做单元测试或离线回放。

两种估计器的适用场景：
- MahonyAHRS：6 轴融合，输出完整姿态（roll/pitch/yaw），适合有俯仰/横滚
  需求的平台，也可将来复用到无人机；线性加速度大时会污染 tilt。
- YawEstimator：只积分 Z 轴角速度，适合平地轮式底盘。不依赖加速度计，
  不会被加减速带偏；代价是没有绝对航向参考，航向会随时间漂移，
  必须靠静止零偏标定 + 雷达/地图匹配兜底。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

GRAVITY = 9.80665


@dataclass
class ImuSample:
    """一帧 IMU 数据。

    Attributes:
        t: 单调时钟时间戳，单位秒。
        gyro: 3x1 角速度，rad/s。
        accel: 3x1 比力，m/s^2（静止时约为重力反作用力）。
        yaw_deg: 器件自带的航向角（度，绕 +Z 逆时针为正）；没有就为 None。
            HWT101 这类内置卡尔曼解算的模块会直接给出，比我们自己积分更准。
    """

    t: float
    gyro: np.ndarray
    accel: np.ndarray
    yaw_deg: float | None = None

    def __post_init__(self) -> None:
        self.gyro = np.asarray(self.gyro, dtype=float).reshape(3)
        self.accel = np.asarray(self.accel, dtype=float).reshape(3)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """归一化四元数；零四元数退化为单位旋转。"""
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """四元数乘法 a ⊗ b（先施加 b 再施加 a）。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """body -> world 旋转矩阵。"""
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_from_yaw(yaw: float) -> np.ndarray:
    """绕世界 Z 轴旋转 yaw 的四元数。"""
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def quat_yaw(q: np.ndarray) -> float:
    """提取偏航角，范围 (-pi, pi]。"""
    w, x, y, z = quat_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_roll_pitch(q: np.ndarray) -> tuple[float, float]:
    """提取横滚、俯仰角（ZYX 欧拉序），范围分别 (-pi, pi] 与 [-pi/2, pi/2]。"""
    w, x, y, z = quat_normalize(q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    return roll, pitch


def wrap_pi(angle: float) -> float:
    """把角度折回 (-pi, pi]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _integrate(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """用指数映射精确积分四元数，避免小角度近似误差。"""
    rate = float(np.linalg.norm(omega))
    if rate < 1e-12 or dt <= 0.0:
        return quat_normalize(q)
    angle = rate * dt
    axis = omega / rate
    half = 0.5 * angle
    sin_half = math.sin(half)
    delta = np.array(
        [math.cos(half), sin_half * axis[0], sin_half * axis[1], sin_half * axis[2]]
    )
    return quat_normalize(quat_multiply(q, delta))


class MahonyAHRS:
    """6 轴 Mahony 互补滤波。

    加速度计只提供重力方向，修正 roll/pitch；yaw 没有绝对参考，靠陀螺积分，
    因此航向零偏必须在启动时标定（见 estimate_gyro_bias），否则会持续漂移。
    """

    def __init__(
        self,
        kp: float = 0.8,
        ki: float = 0.02,
        accel_gate: float = 0.25,
        max_bias: float = 0.5,
    ) -> None:
        """
        Args:
            kp: 比例增益，越大收敛越快、越容易被振动/线性加速度带偏。
            ki: 积分增益，用于估计 roll/pitch 方向的陀螺零偏。
            accel_gate: 允许参与修正的加速度模长误差比例（相对 g）。
            max_bias: 零偏估计限幅，rad/s。
        """
        self.kp = float(kp)
        self.ki = float(ki)
        self.accel_gate = float(accel_gate)
        self.max_bias = float(max_bias)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)

    def reset(self, yaw: float = 0.0) -> None:
        """重置姿态到水平、给定航向。"""
        self.q = quat_from_yaw(yaw)
        self.bias = np.zeros(3)

    def update(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> np.ndarray:
        """推进一个采样周期，返回更新后的四元数。"""
        gyro = np.asarray(gyro, dtype=float).reshape(3)
        accel = np.asarray(accel, dtype=float).reshape(3)
        if dt <= 0.0:
            return self.q.copy()

        omega = gyro - self.bias
        accel_norm = float(np.linalg.norm(accel))
        # 只在加速度模长接近 g 时使用重力修正，滤掉加减速与冲击。
        if accel_norm > 1e-3 and abs(accel_norm - GRAVITY) <= self.accel_gate * GRAVITY:
            measured = accel / accel_norm
            # 估计的机体坐标系重力方向 = R^T * [0, 0, 1]
            estimated = quat_to_matrix(self.q).T @ np.array([0.0, 0.0, 1.0])
            error = np.cross(measured, estimated)
            omega = omega + self.kp * error
            self.bias = np.clip(
                self.bias - self.ki * error * dt, -self.max_bias, self.max_bias
            )

        self.q = _integrate(self.q, omega, dt)
        return self.q.copy()


class YawEstimator:
    """地面车航向估计：积分 Z 轴角速度 + 静止零偏跟踪。

    不做重力修正，因此加减速不会引入 tilt 误差；静止时用慢速 EMA 更新
    Z 轴零偏，抑制长时间停放后的漂移。运动中的零偏漂移无法自愈，
    需要雷达/地图匹配提供绝对航向修正。
    """

    def __init__(
        self,
        bias: np.ndarray | None = None,
        stationary_gyro: float = 0.02,
        stationary_accel: float = 0.12,
        bias_tau: float = 8.0,
    ) -> None:
        """
        Args:
            bias: 初始三轴零偏，rad/s。
            stationary_gyro: 判定静止的角速度模长阈值，rad/s。
            stationary_accel: 判定静止的加速度模长误差比例（相对 g）。
            bias_tau: 静止时零偏 EMA 时间常数，秒；越大越保守。
        """
        self.bias = np.zeros(3) if bias is None else np.asarray(bias, dtype=float).reshape(3).copy()
        self.stationary_gyro = float(stationary_gyro)
        self.stationary_accel = float(stationary_accel)
        self.bias_tau = float(bias_tau)
        self.yaw = 0.0
        self._still_time = 0.0

    def reset(self, yaw: float = 0.0) -> None:
        self.yaw = wrap_pi(yaw)
        self._still_time = 0.0

    def update(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> float:
        """推进一个采样周期，返回折回 (-pi, pi] 的航向角（rad）。"""
        gyro = np.asarray(gyro, dtype=float).reshape(3)
        accel = np.asarray(accel, dtype=float).reshape(3)
        if dt <= 0.0:
            return self.yaw

        self.yaw = wrap_pi(self.yaw + (gyro[2] - self.bias[2]) * dt)

        accel_norm = float(np.linalg.norm(accel))
        still = (
            float(np.linalg.norm(gyro)) < self.stationary_gyro
            and abs(accel_norm - GRAVITY) < self.stationary_accel * GRAVITY
        )
        if still:
            self._still_time += dt
            if self._still_time > 0.5:
                alpha = min(1.0, dt / self.bias_tau)
                self.bias[2] += alpha * (gyro[2] - self.bias[2])
        else:
            self._still_time = 0.0
        return self.yaw


def estimate_gyro_bias(
    samples: np.ndarray | list[np.ndarray], max_std: float = 0.05
) -> np.ndarray:
    """由静止样本估计三轴陀螺零偏。

    Args:
        samples: 形状 (N, 3) 的角速度序列，rad/s。
        max_std: 允许的单轴标准差上限；超过说明采样期间在动，直接报错。

    Returns:
        3x1 零偏，rad/s。

    Raises:
        ValueError: 样本不足或噪声过大。
    """
    data = np.asarray(samples, dtype=float)
    if data.ndim != 2 or data.shape[1] != 3 or data.shape[0] < 10:
        raise ValueError(f"静止标定需要至少 10 帧 (N,3) 数据，收到 {data.shape}")
    mean = data.mean(axis=0)
    std = data.std(axis=0)
    if float(std.max()) > max_std:
        raise ValueError(
            f"静止标定失败：角速度标准差 {std.max():.4f} rad/s 超过 {max_std}，"
            "采样期间车辆可能在动"
        )
    return mean
