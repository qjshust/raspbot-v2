"""IMU 数据源：Mock（离线/测试）、WitMotion 串口（HWT101 等）、ICM-42688 SPI。

统一接口 ``read() -> ImuSample | None``，节点只依赖这个接口，换传感器只需
换一个 source 实现，姿态解算与 ROS 接口不动。
"""

from __future__ import annotations

import abc
import math
import struct
import time

import numpy as np

from .attitude import GRAVITY, ImuSample


class ImuSource(abc.ABC):
    """IMU 数据源基类。"""

    #: 是否提供加速度计数据；WitMotion 的角度型模块可能只输出角度+角速度。
    has_accel: bool = True

    @abc.abstractmethod
    def read(self) -> ImuSample | None:
        """非阻塞读取一帧；暂无数据返回 None。"""

    def close(self) -> None:
        """释放底层资源。"""


class MockSource(ImuSource):
    """合成运动数据源，用于离线测试与 demo。

    航向按分段角速度积分，便于和估计值逐点比较：

    - 0.0–2.0 s 静止
    - 2.0–5.0 s 以 +45 °/s 旋转
    - 5.0–7.0 s 静止
    - 7.0–9.0 s 以 -60 °/s 旋转
    """

    def __init__(
        self,
        rate_hz: float = 100.0,
        bias: tuple[float, float, float] = (0.010, -0.005, 0.020),
        gyro_noise: float = 0.002,
        accel_noise: float = 0.05,
        seed: int = 0,
    ) -> None:
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz
        self.bias = np.asarray(bias, dtype=float).reshape(3)
        self.gyro_noise = float(gyro_noise)
        self.accel_noise = float(accel_noise)
        self._rng = np.random.default_rng(seed)
        self._t = 0.0
        self._yaw = 0.0

    def yaw_rate(self, t: float) -> float:
        if 2.0 <= t < 5.0:
            return math.radians(45.0)
        if 7.0 <= t < 9.0:
            return math.radians(-60.0)
        return 0.0

    @property
    def true_yaw(self) -> float:
        return self._yaw

    @property
    def time(self) -> float:
        return self._t

    def read(self) -> ImuSample:
        t = self._t
        rate = self.yaw_rate(t)
        self._yaw = (self._yaw + rate * self.dt + math.pi) % (2.0 * math.pi) - math.pi
        self._t += self.dt

        gyro = np.array([0.0, 0.0, rate]) + self.bias
        gyro += self._rng.normal(0.0, self.gyro_noise, 3)
        accel = np.array([0.0, 0.0, GRAVITY])
        accel += self._rng.normal(0.0, self.accel_noise, 3)
        return ImuSample(t, gyro, accel)

    def close(self) -> None:
        pass


class WitMotionParser:
    """维特智能 0x55 帧解析器（JY/HWT 系列通用）。

    帧格式：``0x55 | type | d0..d7 | checksum``，共 11 字节；
    校验和 = 前 10 字节之和的低 8 位。数据为小端 int16。
    """

    ACCEL = 0x51
    GYRO = 0x52
    ANGLE = 0x53

    def __init__(self, accel_fs_g: float = 16.0, gyro_fs_dps: float = 2000.0) -> None:
        self.accel_fs_g = float(accel_fs_g)
        self.gyro_fs_dps = float(gyro_fs_dps)
        self._buf = bytearray()
        self.gyro = np.zeros(3)
        self.accel = np.zeros(3)
        self.angle_deg = np.zeros(3)
        self.seen_accel = False
        self.seen_angle = False

    def feed(self, data: bytes) -> bool:
        """喂入字节流；解析到至少一帧返回 True。"""
        self._buf.extend(data)
        got = False
        while True:
            start = self._buf.find(b'\x55')
            if start < 0:
                self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < 11:
                break
            frame = bytes(self._buf[:11])
            if (sum(frame[:10]) & 0xFF) != frame[10]:
                del self._buf[0]  # 校验失败，丢弃一个字节重新同步
                continue
            del self._buf[:11]
            self._handle(frame)
            got = True
        if len(self._buf) > 4096:  # 防御性上限，避免异常数据撑爆内存
            self._buf.clear()
        return got

    def _handle(self, frame: bytes) -> None:
        kind = frame[1]
        x, y, z = struct.unpack("<3h", frame[2:8])
        if kind == self.GYRO:
            scale = self.gyro_fs_dps / 32768.0 * math.pi / 180.0
            self.gyro = np.array([x, y, z], dtype=float) * scale
        elif kind == self.ACCEL:
            self.seen_accel = True
            scale = self.accel_fs_g / 32768.0 * GRAVITY
            self.accel = np.array([x, y, z], dtype=float) * scale
        elif kind == self.ANGLE:
            self.seen_angle = True
            self.angle_deg = np.array([x, y, z], dtype=float) * (180.0 / 32768.0)


class WitMotionSource(ImuSource):
    """维特智能串口模块（HWT101 / JY901 等）。

    默认按 115200 8N1 打开；HWT101 出厂输出速率是 10 Hz，需要先用维特
    上位机或协议把速率改到 100 Hz。
    """

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 0.0,
        accel_fs_g: float = 16.0,
        gyro_fs_dps: float = 2000.0,
    ) -> None:
        try:
            import serial  # pyserial
        except ImportError as exc:  # pragma: no cover - 依赖真实串口
            raise RuntimeError("缺少 pyserial：pip install pyserial") from exc
        self._serial = serial.Serial(port, baud, timeout=timeout)
        self._parser = WitMotionParser(accel_fs_g=accel_fs_g, gyro_fs_dps=gyro_fs_dps)

    @property
    def has_accel(self) -> bool:  # type: ignore[override]
        return self._parser.seen_accel

    def read(self) -> ImuSample | None:
        data = self._serial.read(256)
        if not data:
            return None
        if not self._parser.feed(data):
            return None
        accel = self._parser.accel if self._parser.seen_accel else np.zeros(3)
        yaw = float(self._parser.angle_deg[2]) if self._parser.seen_angle else None
        return ImuSample(time.monotonic(), self._parser.gyro.copy(), accel.copy(), yaw)

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:  # noqa: BLE001 - 关闭失败不应影响退出
            pass


def make_source(kind: str, **kwargs) -> ImuSource:
    """按名称构造数据源。"""
    kind = kind.lower().strip()
    if kind == "mock":
        return MockSource(**kwargs)
    if kind == "witmotion":
        return WitMotionSource(**kwargs)
    if kind == "icm42688":
        from .icm42688 import ICM42688

        driver = ICM42688(**kwargs)
        driver.open()
        return _SpiSource(driver)
    raise ValueError(f"未知数据源 {kind!r}，可选：mock / witmotion / icm42688")


class _SpiSource(ImuSource):
    """把 ICM42688 驱动包装成 ImuSource。"""

    def __init__(self, driver) -> None:
        self._driver = driver

    def read(self) -> ImuSample | None:
        try:
            return self._driver.read()
        except Exception:  # noqa: BLE001 - 单帧无效时跳过，等待下一帧
            return None

    def close(self) -> None:
        self._driver.close()
