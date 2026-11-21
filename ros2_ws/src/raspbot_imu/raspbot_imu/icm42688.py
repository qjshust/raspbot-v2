"""ICM-42688-P SPI 驱动（树莓派 /dev/spidev0.0）。

寄存器地址与位定义来自 TDK ICM-42688-P 数据手册，并与 Linux 内核
``drivers/iio/imu/inv_icm42600`` 驱动交叉核对：

- WHO_AM_I(0x75) == 0x47
- 数据区 0x1D..0x2A：温度(2) + 加速度 XYZ(6) + 陀螺 XYZ(6)，均为大端 int16
- PWR_MGMT0(0x4E)：bit3:2 陀螺模式、bit1:0 加速度模式，0b11 = 低噪声
- GYRO_CONFIG0(0x4F) / ACCEL_CONFIG0(0x50)：bit7:5 量程、bit3:0 输出速率

SPI 时序：模式 3、MSB first、最高 24 MHz（这里默认 2 MHz 留余量）。
读寄存器时首字节置 bit7；突发读地址自增。

仅支持 Linux（依赖 spidev）；导入本模块不会触发硬件访问。
"""

from __future__ import annotations

import math
import struct
import time

import numpy as np

from .attitude import GRAVITY, ImuSample

# --- 寄存器地址（User Bank 0） ---
REG_DEVICE_CONFIG = 0x11
REG_INT_CONFIG = 0x14
REG_TEMP_DATA = 0x1D
REG_ACCEL_DATA_X = 0x1F
REG_GYRO_DATA_X = 0x25
REG_INT_STATUS = 0x2D
REG_PWR_MGMT0 = 0x4E
REG_GYRO_CONFIG0 = 0x4F
REG_ACCEL_CONFIG0 = 0x50
REG_GYRO_ACCEL_CONFIG0 = 0x52
REG_INT_CONFIG1 = 0x64
REG_INT_SOURCE0 = 0x65
REG_WHO_AM_I = 0x75

WHO_AM_I_ICM42688 = 0x47

# --- 配置字段 ---
PWR_MGMT0_GYRO_LN_ACCEL_LN = 0x0F  # 陀螺/加速度均低噪声模式，温度使能
INT_CONFIG_PUSH_PULL_ACTIVE_HIGH = 0x03
INT_SOURCE0_UI_DRDY_INT1 = 0x08
INT_STATUS_DATA_RDY = 0x08

GYRO_FS_CODES = {
    2000.0: 0,
    1000.0: 1,
    500.0: 2,
    250.0: 3,
    125.0: 4,
    62.5: 5,
    31.25: 6,
    15.625: 7,
}
ACCEL_FS_CODES = {16.0: 0, 8.0: 1, 4.0: 2, 2.0: 3}
ODR_CODES = {
    8000.0: 3,
    4000.0: 4,
    2000.0: 5,
    1000.0: 6,
    500.0: 15,
    200.0: 7,
    100.0: 8,
    50.0: 9,
    25.0: 10,
    12.5: 11,
}

_DATA_INVALID = -32768


class ImuDataInvalidError(ValueError):
    """传感器返回了数据无效标记（-32768）。"""


def _nearest_code(table: dict[float, int], value: float, name: str) -> int:
    for key in table:
        if math.isclose(key, float(value), rel_tol=1e-6):
            return table[key]
    raise ValueError(f"不支持的 {name}={value}，可选：{sorted(table)}")


def parse_burst(
    buf: bytes | bytearray, gyro_fs_dps: float, accel_fs_g: float
) -> tuple[np.ndarray, np.ndarray]:
    """解析从 0x1D 起连续读取的 14 字节。

    Returns:
        (gyro_rad_s, accel_m_s2)，各为 3x1。

    Raises:
        ImuDataInvalidError: 任一轴为 -32768（数据无效）。
        ValueError: 长度不足。
    """
    if len(buf) < 14:
        raise ValueError(f"数据长度应为 14 字节，收到 {len(buf)}")
    _, ax, ay, az, gx, gy, gz = struct.unpack(">7h", bytes(buf[:14]))
    raw_accel = (ax, ay, az)
    raw_gyro = (gx, gy, gz)
    if any(v == _DATA_INVALID for v in raw_accel + raw_gyro):
        raise ImuDataInvalidError("ICM-42688-P 返回数据无效标记")

    gyro = np.array(raw_gyro, dtype=float) * (gyro_fs_dps / 32768.0) * math.pi / 180.0
    accel = np.array(raw_accel, dtype=float) * (accel_fs_g / 32768.0) * GRAVITY
    return gyro, accel


class ICM42688:
    """ICM-42688-P 轮询驱动。"""

    def __init__(
        self,
        bus: int = 0,
        device: int = 0,
        speed_hz: int = 2_000_000,
        gyro_fs_dps: float = 500.0,
        accel_fs_g: float = 4.0,
        odr_hz: float = 1000.0,
    ) -> None:
        self.bus = int(bus)
        self.device = int(device)
        self.speed_hz = int(speed_hz)
        self.gyro_fs_dps = float(gyro_fs_dps)
        self.accel_fs_g = float(accel_fs_g)
        self.odr_hz = float(odr_hz)
        self._spi = None
        self.last_temperature_c: float | None = None

    # --- 生命周期 ---
    def open(self) -> None:
        try:
            import spidev  # 延迟导入，便于在无硬件的机器上导入本模块
        except ImportError as exc:  # pragma: no cover - 依赖真实硬件
            raise RuntimeError(
                "缺少 spidev：在树莓派上执行 `sudo apt install python3-spidev` "
                "或 `pip install spidev`，并确认 config.txt 中有 dtparam=spi=on"
            ) from exc

        self._spi = spidev.SpiDev()
        self._spi.open(self.bus, self.device)
        self._spi.max_speed_hz = self.speed_hz
        self._spi.mode = 3
        self._spi.cshigh = False
        self._reset()
        self._configure()

    def close(self) -> None:
        if self._spi is not None:
            try:
                self._spi.close()
            finally:
                self._spi = None

    # --- 寄存器访问 ---
    def _read_reg(self, reg: int) -> int:
        assert self._spi is not None
        data = self._spi.xfer2([0x80 | (reg & 0x7F), 0x00])
        return int(data[1])

    def _write_reg(self, reg: int, value: int) -> None:
        assert self._spi is not None
        self._spi.xfer2([reg & 0x7F, value & 0xFF])

    def _read_burst(self, reg: int, length: int) -> bytes:
        assert self._spi is not None
        data = self._spi.xfer2([0x80 | (reg & 0x7F)] + [0x00] * length)
        return bytes(data[1:])

    # --- 初始化 ---
    def _reset(self) -> None:
        self._write_reg(REG_DEVICE_CONFIG, 0x01)
        time.sleep(0.100)  # 软复位后等待电源稳定

    def _configure(self) -> None:
        if self._read_reg(REG_WHO_AM_I) != WHO_AM_I_ICM42688:
            raise RuntimeError(
                "WHO_AM_I 不匹配，未检测到 ICM-42688-P（检查 SPI 接线/片选/供电）"
            )

        self._write_reg(REG_PWR_MGMT0, PWR_MGMT0_GYRO_LN_ACCEL_LN)
        time.sleep(0.080)  # 陀螺启动 60 ms + 加速度 20 ms

        gyro_code = _nearest_code(GYRO_FS_CODES, self.gyro_fs_dps, "gyro_fs_dps")
        accel_code = _nearest_code(ACCEL_FS_CODES, self.accel_fs_g, "accel_fs_g")
        odr_code = _nearest_code(ODR_CODES, self.odr_hz, "odr_hz")

        self._write_reg(REG_GYRO_CONFIG0, (gyro_code << 5) | odr_code)
        self._write_reg(REG_ACCEL_CONFIG0, (accel_code << 5) | odr_code)
        self._write_reg(REG_GYRO_ACCEL_CONFIG0, 0x00)  # 滤波器带宽 = ODR/2
        self._write_reg(REG_INT_CONFIG, INT_CONFIG_PUSH_PULL_ACTIVE_HIGH)
        self._write_reg(REG_INT_SOURCE0, INT_SOURCE0_UI_DRDY_INT1)
        self._write_reg(REG_INT_CONFIG1, 0x00)
        time.sleep(0.050)

    # --- 数据读取 ---
    def data_ready(self) -> bool:
        return bool(self._read_reg(REG_INT_STATUS) & INT_STATUS_DATA_RDY)

    def read(self, t: float | None = None) -> ImuSample:
        """读取一帧。数据无效时抛 ImuDataInvalidError。"""
        buf = self._read_burst(REG_TEMP_DATA, 14)
        gyro, accel = parse_burst(buf, self.gyro_fs_dps, self.accel_fs_g)
        self.last_temperature_c = struct.unpack(">h", buf[0:2])[0] / 132.48 + 25.0
        return ImuSample(time.monotonic() if t is None else t, gyro, accel)
