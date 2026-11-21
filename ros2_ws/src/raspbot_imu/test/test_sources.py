"""数据源测试：维特帧解析与 Mock 数据源（无硬件）。"""

from __future__ import annotations

import math
import struct

import numpy as np
import pytest

from raspbot_imu.attitude import GRAVITY
from raspbot_imu.sources import MockSource, WitMotionParser, make_source


def wm_frame(kind: int, x: int = 0, y: int = 0, z: int = 0) -> bytes:
    payload = struct.pack('<3h', x, y, z) + b'\x00\x00'
    body = bytes([0x55, kind]) + payload
    return body + bytes([sum(body) & 0xFF])


def test_parser_accel_and_gyro() -> None:
    parser = WitMotionParser(accel_fs_g=16.0, gyro_fs_dps=2000.0)
    assert parser.feed(wm_frame(WitMotionParser.ACCEL, z=16384)) is True
    assert parser.seen_accel is True
    assert parser.accel[2] == pytest.approx(8.0 * GRAVITY, rel=1e-6)

    assert parser.feed(wm_frame(WitMotionParser.GYRO, z=16384)) is True
    assert parser.gyro[2] == pytest.approx(1000.0 * math.pi / 180.0, rel=1e-6)


def test_parser_resyncs_after_garbage() -> None:
    parser = WitMotionParser()
    assert parser.feed(b'\xde\xad\xbe\xef' + wm_frame(WitMotionParser.GYRO, x=3277)) is True
    assert parser.gyro[0] == pytest.approx(200.0 * math.pi / 180.0, rel=1e-3)


def test_parser_handles_split_frame() -> None:
    parser = WitMotionParser()
    frame = wm_frame(WitMotionParser.ACCEL, z=8192)
    assert parser.feed(frame[:6]) is False
    assert parser.feed(frame[6:]) is True
    assert parser.accel[2] == pytest.approx(4.0 * GRAVITY, rel=1e-6)


def test_parser_rejects_bad_checksum() -> None:
    parser = WitMotionParser()
    frame = bytearray(wm_frame(WitMotionParser.ACCEL, z=16384))
    frame[-1] = (frame[-1] + 1) & 0xFF
    assert parser.feed(bytes(frame)) is False
    assert parser.seen_accel is False


def test_mock_source_rate_and_motion() -> None:
    source = MockSource(rate_hz=100.0)
    samples = [source.read() for _ in range(100)]
    assert samples[0].t == pytest.approx(0.0)
    assert samples[-1].t == pytest.approx(0.99)
    assert source.yaw_rate(2.5) == pytest.approx(math.radians(45.0))
    assert source.yaw_rate(0.5) == 0.0
    assert source.yaw_rate(8.0) == pytest.approx(math.radians(-60.0))


def test_mock_source_bias_and_noise() -> None:
    source = MockSource(rate_hz=100.0, gyro_noise=0.0, accel_noise=0.0)
    for _ in range(200):  # 前 2 s 静止，只有零偏
        sample = source.read()
    np.testing.assert_allclose(sample.gyro, [0.010, -0.005, 0.020], atol=1e-12)
    np.testing.assert_allclose(sample.accel, [0.0, 0.0, GRAVITY], atol=1e-12)


def test_make_source_factory() -> None:
    assert isinstance(make_source('mock'), MockSource)
    with pytest.raises(ValueError, match='未知数据源'):
        make_source('does-not-exist')
