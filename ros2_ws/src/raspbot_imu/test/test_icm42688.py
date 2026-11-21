"""ICM-42688-P 数据解析测试（纯函数，不需要 SPI 硬件）。"""

from __future__ import annotations

import math
import struct

import numpy as np
import pytest

from raspbot_imu.icm42688 import (
    ACCEL_FS_CODES,
    GYRO_FS_CODES,
    ODR_CODES,
    ImuDataInvalidError,
    parse_burst,
    _nearest_code,
)


def make_frame(ax=0, ay=0, az=0, gx=0, gy=0, gz=0, temp=0) -> bytes:
    """构造 0x1D..0x2A 的 14 字节大端数据。"""
    return struct.pack('>7h', temp, ax, ay, az, gx, gy, gz)


def test_parse_scaling() -> None:
    gyro, accel = parse_burst(make_frame(az=16384, gz=16384), 2000.0, 4.0)
    # 16384 / 32768 = 0.5 -> 2 g，1000 °/s
    assert accel[2] == pytest.approx(0.5 * 4.0 * 9.80665)
    assert gyro[2] == pytest.approx(1000.0 * math.pi / 180.0)


def test_parse_negative_values() -> None:
    gyro, accel = parse_burst(make_frame(ax=-16384, gz=-16384), 2000.0, 4.0)
    assert accel[0] == pytest.approx(-0.5 * 4.0 * 9.80665)
    assert gyro[2] == pytest.approx(-1000.0 * math.pi / 180.0)


def test_parse_rejects_invalid_marker() -> None:
    with pytest.raises(ImuDataInvalidError):
        parse_burst(make_frame(az=-32768), 2000.0, 4.0)


def test_parse_rejects_short_buffer() -> None:
    with pytest.raises(ValueError, match='14'):
        parse_burst(b'\x00' * 10, 2000.0, 4.0)


def test_config_codes_match_datasheet() -> None:
    assert GYRO_FS_CODES[500.0] == 2
    assert GYRO_FS_CODES[2000.0] == 0
    assert ACCEL_FS_CODES[4.0] == 2
    assert ACCEL_FS_CODES[16.0] == 0
    assert ODR_CODES[1000.0] == 6
    assert ODR_CODES[100.0] == 8
    assert ODR_CODES[500.0] == 15


def test_nearest_code_accepts_int_and_float() -> None:
    assert _nearest_code(GYRO_FS_CODES, 500, 'gyro_fs_dps') == 2
    assert _nearest_code(ODR_CODES, 100.0, 'odr_hz') == 8


def test_nearest_code_rejects_unsupported() -> None:
    with pytest.raises(ValueError, match='不支持'):
        _nearest_code(GYRO_FS_CODES, 123.0, 'gyro_fs_dps')
