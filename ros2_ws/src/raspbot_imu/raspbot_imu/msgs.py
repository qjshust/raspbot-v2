"""IMU 消息组装纯函数（与 ROS 无关，可离线单测）。

``build_imu_dict`` 把姿态解算结果打包成普通 dict，节点（node.py）再据此
组装 ``sensor_msgs/Imu``。本模块**不 import sensor_msgs**，因此 SW-U14 可以
在没有 ROS 的环境里直接验证。

字段约定（与 sensor_msgs/Imu 一致）：
- ``orientation``：四元数 ``[w, x, y, z]``；
- ``orientation_covariance`` / ``angular_velocity_covariance`` /
  ``linear_acceleration_covariance``：3x3 行主序的 9 元列表，均为对角阵；
  方位协方差对角为 ``[tilt, tilt, yaw]``（yaw 模式下 roll/pitch 未知、给大值）。
"""

from __future__ import annotations

from typing import Iterable


def _vec3(values: Iterable, name: str) -> list[float]:
    """转成 3 元 float 列表；长度不对抛 ValueError。"""
    out = [float(v) for v in values]
    if len(out) != 3:
        raise ValueError(f"{name} 应为 3 元向量，收到 {len(out)} 元")
    return out


def build_imu_dict(
    now_sec: float,
    frame_id: str,
    quat: Iterable,
    gyro: Iterable,
    accel: Iterable,
    yaw_var: float,
    tilt_var: float,
    gyro_var: float,
    accel_var: float,
) -> dict:
    """组装一帧 IMU 数据字典。

    Args:
        now_sec: 采样时刻，epoch 秒（float）。
        frame_id: 坐标系名（契约：``imu_link``）。
        quat: 四元数 ``[w, x, y, z]``。
        gyro: 角速度 ``[x, y, z]``，rad/s。
        accel: 线加速度 ``[x, y, z]``，m/s^2。
        yaw_var: 航向方差（occupancy_covariance 的 Z 对角元）。
        tilt_var: 横滚/俯仰方差（occupancy_covariance 的 X/Y 对角元）。
        gyro_var: 角速度方差（对角）。
        accel_var: 线加速度方差（对角）。

    Returns:
        dict：``stamp_sec``、``frame_id``、``orientation``、
        ``angular_velocity``、``linear_acceleration`` 以及三个 3x3 对角
        协方差（9 元行主序列表）。全部数值为普通 Python float。
    """
    q = [float(v) for v in quat]
    if len(q) != 4:
        raise ValueError(f"quat 应为 4 元四元数 [w,x,y,z]，收到 {len(q)} 元")

    tilt = float(tilt_var)
    yaw = float(yaw_var)
    gv = float(gyro_var)
    av = float(accel_var)

    return {
        "stamp_sec": float(now_sec),
        "frame_id": str(frame_id),
        "orientation": q,
        "angular_velocity": _vec3(gyro, "gyro"),
        "linear_acceleration": _vec3(accel, "accel"),
        "orientation_covariance": [
            tilt, 0.0, 0.0,
            0.0, tilt, 0.0,
            0.0, 0.0, yaw,
        ],
        "angular_velocity_covariance": [
            gv, 0.0, 0.0,
            0.0, gv, 0.0,
            0.0, 0.0, gv,
        ],
        "linear_acceleration_covariance": [
            av, 0.0, 0.0,
            0.0, av, 0.0,
            0.0, 0.0, av,
        ],
    }
