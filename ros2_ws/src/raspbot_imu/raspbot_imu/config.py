"""参数校验纯函数（与 ROS 无关，可离线单测）。

节点把 ``mode`` / ``angle_source`` / ``source`` 三个契约参数交给
``validate_config`` 校验：非法值抛 ``ValueError``，报错信息里包含全部
可选值（SW-U15 / SW-G04 的判据）。合法时返回规范化（小写）后的三元组，
节点直接用返回值，避免各处重复 lower()。
"""

from __future__ import annotations

MODE_CHOICES = ("yaw", "mahony")
ANGLE_SOURCE_CHOICES = ("device", "gyro")
SOURCE_CHOICES = ("mock", "witmotion", "icm42688")


def validate_config(
    mode: str,
    angle_source: str,
    source: str,
) -> tuple[str, str, str]:
    """校验 IMU 节点关键参数。

    Args:
        mode: 姿态模式，``yaw`` 或 ``mahony``。
        angle_source: 航向来源，``device``（模块自带角度）或 ``gyro``（自积分）。
        source: 数据源，``mock`` / ``witmotion`` / ``icm42688``。

    Returns:
        (mode, angle_source, source) 规范化三元组（小写、去首尾空白）。

    Raises:
        ValueError: 任一参数不在可选值内，报错信息包含全部可选值。
    """
    mode_n = str(mode).strip().lower()
    if mode_n not in MODE_CHOICES:
        raise ValueError(
            f"mode 非法：可选值 yaw / mahony，收到 {mode!r}"
        )

    angle_n = str(angle_source).strip().lower()
    if angle_n not in ANGLE_SOURCE_CHOICES:
        raise ValueError(
            f"angle_source 非法：可选值 device / gyro，收到 {angle_source!r}"
        )

    source_n = str(source).strip().lower()
    if source_n not in SOURCE_CHOICES:
        raise ValueError(
            f"source 非法：可选值 mock / witmotion / icm42688，收到 {source!r}"
        )

    return mode_n, angle_n, source_n
