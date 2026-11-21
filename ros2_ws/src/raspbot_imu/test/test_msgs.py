"""IMU 消息组装测试（SW-U14，纯函数、无 ROS）。"""

from __future__ import annotations

import math

import pytest

from raspbot_imu.attitude import GRAVITY, quat_from_yaw
from raspbot_imu.msgs import build_imu_dict

# 与 config/imu.yaml 一致（HWT101 实际部署值）
YAW_VAR = 0.0003
TILT_VAR = 1.0
GYRO_VAR = 0.0004
ACCEL_VAR = 0.04


def make_dict(yaw: float = 0.0) -> dict:
    return build_imu_dict(
        now_sec=1_750_000_000.123,
        frame_id='imu_link',
        quat=quat_from_yaw(yaw),
        gyro=[0.1, -0.2, 0.3],
        accel=[0.0, 0.0, GRAVITY],
        yaw_var=YAW_VAR,
        tilt_var=TILT_VAR,
        gyro_var=GYRO_VAR,
        accel_var=ACCEL_VAR,
    )


def test_frame_id_and_stamp() -> None:
    data = make_dict()
    assert data['frame_id'] == 'imu_link'
    assert data['stamp_sec'] == pytest.approx(1_750_000_000.123)


def test_quaternion_norm_is_unit_for_different_yaws() -> None:
    for yaw in (0.0, 0.4, -1.2, 2.9, math.pi, -math.pi + 0.01):
        data = make_dict(yaw=yaw)
        w, x, y, z = data['orientation']
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        assert abs(norm - 1.0) <= 1e-6


def test_orientation_passes_through() -> None:
    q = quat_from_yaw(0.7)
    data = build_imu_dict(0.0, 'imu_link', q, [0, 0, 0], [0, 0, 0],
                          0.0, 0.0, 0.0, 0.0)
    assert data['orientation'] == [q[0], q[1], q[2], q[3]]


def test_covariance_diagonals_match_yaml() -> None:
    data = make_dict()
    assert data['orientation_covariance'] == [
        TILT_VAR, 0.0, 0.0,
        0.0, TILT_VAR, 0.0,
        0.0, 0.0, YAW_VAR,
    ]
    assert data['angular_velocity_covariance'] == [
        GYRO_VAR, 0.0, 0.0,
        0.0, GYRO_VAR, 0.0,
        0.0, 0.0, GYRO_VAR,
    ]
    assert data['linear_acceleration_covariance'] == [
        ACCEL_VAR, 0.0, 0.0,
        0.0, ACCEL_VAR, 0.0,
        0.0, 0.0, ACCEL_VAR,
    ]


def test_gyro_accel_passthrough_as_plain_floats() -> None:
    data = make_dict()
    assert data['angular_velocity'] == [0.1, -0.2, 0.3]
    assert data['linear_acceleration'] == [0.0, 0.0, GRAVITY]
    assert all(isinstance(v, float) for v in data['angular_velocity'])


def test_rejects_wrong_lengths() -> None:
    with pytest.raises(ValueError, match='quat'):
        build_imu_dict(0.0, 'imu_link', [1.0, 0.0, 0.0], [0, 0, 0], [0, 0, 0],
                       0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match='gyro'):
        build_imu_dict(0.0, 'imu_link', [1.0, 0.0, 0.0, 0.0], [0, 0], [0, 0, 0],
                       0.0, 0.0, 0.0, 0.0)
