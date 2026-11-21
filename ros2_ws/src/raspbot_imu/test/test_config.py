"""参数校验测试（SW-U15，纯函数、无 ROS）。"""

from __future__ import annotations

import pytest

from raspbot_imu.config import validate_config


def test_rejects_unknown_mode_with_choices() -> None:
    with pytest.raises(ValueError) as exc:
        validate_config('foo', 'device', 'witmotion')
    message = str(exc.value)
    assert 'yaw' in message and 'mahony' in message


def test_rejects_unknown_angle_source_with_choices() -> None:
    with pytest.raises(ValueError) as exc:
        validate_config('yaw', 'bad', 'witmotion')
    message = str(exc.value)
    assert 'device' in message and 'gyro' in message


def test_rejects_unknown_source_with_choices() -> None:
    with pytest.raises(ValueError) as exc:
        validate_config('yaw', 'device', 'unknown')
    message = str(exc.value)
    assert 'mock' in message and 'witmotion' in message and 'icm42688' in message


@pytest.mark.parametrize(
    ('mode', 'angle_source', 'source'),
    [
        ('yaw', 'device', 'witmotion'),
        ('yaw', 'gyro', 'icm42688'),
        ('mahony', 'gyro', 'mock'),
        ('YAW', 'Device', 'WITMOTION'),  # 大小写归一化
        (' mahony ', 'gyro', 'mock'),    # 首尾空白归一化
    ],
)
def test_valid_combinations_pass(mode: str, angle_source: str, source: str) -> None:
    assert validate_config(mode, angle_source, source) == (
        mode.strip().lower(),
        angle_source.strip().lower(),
        source.strip().lower(),
    )
