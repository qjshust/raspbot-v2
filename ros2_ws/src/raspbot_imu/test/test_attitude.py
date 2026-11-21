"""姿态解算单元测试（无硬件、无 ROS）。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from raspbot_imu.attitude import (
    GRAVITY,
    ImuSample,
    MahonyAHRS,
    YawEstimator,
    estimate_gyro_bias,
    quat_from_yaw,
    quat_multiply,
    quat_roll_pitch,
    quat_to_matrix,
    quat_yaw,
    wrap_pi,
)


def test_quat_yaw_roundtrip() -> None:
    for yaw in (-3.0, -1.2, 0.0, 0.4, 2.5):
        assert abs(wrap_pi(quat_yaw(quat_from_yaw(yaw)) - yaw)) < 1e-9


def test_quat_compose_yaw() -> None:
    q = quat_multiply(quat_from_yaw(0.3), quat_from_yaw(0.2))
    assert abs(wrap_pi(quat_yaw(q) - 0.5)) < 1e-9


def test_quat_matrix_rotates_body_x_to_heading() -> None:
    yaw = 1.2
    rotation = quat_to_matrix(quat_from_yaw(yaw))
    np.testing.assert_allclose(
        rotation @ np.array([1.0, 0.0, 0.0]),
        [math.cos(yaw), math.sin(yaw), 0.0],
        atol=1e-9,
    )
    np.testing.assert_allclose(
        rotation @ np.array([0.0, 0.0, 1.0]), [0.0, 0.0, 1.0], atol=1e-9
    )


def test_wrap_pi() -> None:
    assert wrap_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_pi(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_imu_sample_accepts_lists() -> None:
    sample = ImuSample(1.0, [0, 0, 1], [0, 0, GRAVITY])
    assert sample.gyro.shape == (3,)
    assert sample.accel[2] == pytest.approx(GRAVITY)


def test_bias_estimate_static() -> None:
    rng = np.random.default_rng(7)
    bias = np.array([0.010, -0.020, 0.030])
    samples = bias + rng.normal(0.0, 0.002, (2000, 3))
    estimated = estimate_gyro_bias(samples)
    np.testing.assert_allclose(estimated, bias, atol=2e-3)


def test_bias_estimate_rejects_motion() -> None:
    rng = np.random.default_rng(8)
    samples = rng.normal(0.0, 0.2, (500, 3))
    with pytest.raises(ValueError, match='静止标定'):
        estimate_gyro_bias(samples)


def test_bias_estimate_rejects_too_few_samples() -> None:
    with pytest.raises(ValueError, match='至少 10 帧'):
        estimate_gyro_bias(np.zeros((5, 3)))


def test_yaw_integrates_rate() -> None:
    estimator = YawEstimator()
    rate = math.pi / 2.0  # 90 °/s
    yaw = 0.0
    for _ in range(200):  # 2 s
        yaw = estimator.update([0.0, 0.0, rate], [0.0, 0.0, GRAVITY], 0.01)
    assert abs(abs(yaw) - math.pi) < 0.01


def test_yaw_compensates_known_bias() -> None:
    bias = np.array([0.0, 0.0, 0.02])
    estimator = YawEstimator(bias=bias)
    for _ in range(100):  # 1 s，真实角速度 0.5 rad/s
        yaw = estimator.update([0.0, 0.0, 0.5 + bias[2]], [0.0, 0.0, GRAVITY], 0.01)
    assert yaw == pytest.approx(0.5, abs=1e-9)


def test_yaw_tracks_bias_when_stationary() -> None:
    estimator = YawEstimator()
    for _ in range(3000):  # 30 s 静止，真实零偏 0.01 rad/s
        estimator.update([0.0, 0.0, 0.01], [0.0, 0.0, GRAVITY], 0.01)
    assert estimator.bias[2] > 0.006
    assert estimator.bias[2] < 0.011


def test_yaw_does_not_track_bias_when_moving() -> None:
    estimator = YawEstimator()
    for _ in range(3000):
        estimator.update([0.0, 0.0, 0.5], [0.0, 0.0, GRAVITY], 0.01)
    assert abs(estimator.bias[2]) < 1e-6


def test_mahony_levels_from_rest() -> None:
    ahrs = MahonyAHRS()
    for _ in range(1000):  # 10 s
        ahrs.update([0.0, 0.0, 0.0], [0.0, 0.0, GRAVITY], 0.01)
    roll, pitch = quat_roll_pitch(ahrs.q)
    assert abs(roll) < 0.01
    assert abs(pitch) < 0.01


def test_mahony_recovers_roll_from_gravity() -> None:
    roll = math.radians(30.0)
    # 机体绕 x 轴滚转 30° 时，加速度计读数 = R^T * [0,0,g]
    accel = np.array([0.0, GRAVITY * math.sin(roll), GRAVITY * math.cos(roll)])
    ahrs = MahonyAHRS(kp=2.0, ki=0.0)
    for _ in range(2000):  # 20 s
        ahrs.update([0.0, 0.0, 0.0], accel, 0.01)
    estimated_roll, estimated_pitch = quat_roll_pitch(ahrs.q)
    assert estimated_roll == pytest.approx(roll, abs=0.02)
    assert abs(estimated_pitch) < 0.02


def test_mahony_gates_linear_acceleration() -> None:
    ahrs = MahonyAHRS(kp=2.0, ki=0.0)
    for _ in range(1000):
        ahrs.update([0.0, 0.0, 0.0], [0.0, 0.0, GRAVITY], 0.01)
    for _ in range(100):  # 1 s 的 3 g 冲击，不应把姿态带偏
        ahrs.update([0.0, 0.0, 0.0], [0.0, 0.0, 3.0 * GRAVITY], 0.01)
    roll, pitch = quat_roll_pitch(ahrs.q)
    assert abs(roll) < 0.02
    assert abs(pitch) < 0.02
