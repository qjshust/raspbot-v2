/// ICM-42688-P 数据解析测试（1:1 映射 Python 版 test/test_icm42688.py，7 用例）。
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "raspbot_imu/icm42688.hpp"

using namespace raspbot_imu;

namespace {

/// 构造 0x1D..0x2A 的 14 字节大端数据（温度 + 加速度 XYZ + 陀螺 XYZ）。
std::vector<std::uint8_t> make_frame(std::int16_t ax = 0, std::int16_t ay = 0,
                                     std::int16_t az = 0, std::int16_t gx = 0,
                                     std::int16_t gy = 0, std::int16_t gz = 0,
                                     std::int16_t temp = 0) {
  std::vector<std::uint8_t> buf(14, 0);
  const std::int16_t values[7] = {temp, ax, ay, az, gx, gy, gz};
  for (int i = 0; i < 7; ++i) {
    buf[2 * i] = static_cast<std::uint8_t>((values[i] >> 8) & 0xFF);
    buf[2 * i + 1] = static_cast<std::uint8_t>(values[i] & 0xFF);
  }
  return buf;
}

}  // namespace

TEST(Icm42688, ParseScaling) {
  const auto buf = make_frame(0, 0, 16384, 0, 0, 16384);
  const auto [gyro, accel] =
      parse_burst(buf.data(), buf.size(), 2000.0, 4.0);
  // 16384 / 32768 = 0.5 -> 2 g，1000 °/s
  EXPECT_NEAR(accel[2], 0.5 * 4.0 * 9.80665, 1e-12);
  EXPECT_NEAR(gyro[2], 1000.0 * kPi / 180.0, 1e-12);
}

TEST(Icm42688, ParseNegativeValues) {
  const auto buf = make_frame(-16384, 0, 0, 0, 0, -16384);
  const auto [gyro, accel] =
      parse_burst(buf.data(), buf.size(), 2000.0, 4.0);
  EXPECT_NEAR(accel[0], -0.5 * 4.0 * 9.80665, 1e-12);
  EXPECT_NEAR(gyro[2], -1000.0 * kPi / 180.0, 1e-12);
}

TEST(Icm42688, ParseRejectsInvalidMarker) {
  const auto buf = make_frame(0, 0, -32768);
  EXPECT_THROW(parse_burst(buf.data(), buf.size(), 2000.0, 4.0),
               ImuDataInvalidError);
}

TEST(Icm42688, ParseRejectsShortBuffer) {
  const std::vector<std::uint8_t> buf(10, 0);
  try {
    parse_burst(buf.data(), buf.size(), 2000.0, 4.0);
    FAIL() << "应当抛出 std::invalid_argument";
  } catch (const std::invalid_argument& exc) {
    EXPECT_NE(std::string(exc.what()).find("14"), std::string::npos);
  }
}

TEST(Icm42688, ConfigCodesMatchDatasheet) {
  EXPECT_EQ(kGyroFsCodes.at(500.0), 2);
  EXPECT_EQ(kGyroFsCodes.at(2000.0), 0);
  EXPECT_EQ(kAccelFsCodes.at(4.0), 2);
  EXPECT_EQ(kAccelFsCodes.at(16.0), 0);
  EXPECT_EQ(kOdrCodes.at(1000.0), 6);
  EXPECT_EQ(kOdrCodes.at(100.0), 8);
  EXPECT_EQ(kOdrCodes.at(500.0), 15);
}

TEST(Icm42688, NearestCodeAcceptsIntAndFloat) {
  EXPECT_EQ(nearest_code(kGyroFsCodes, 500.0, "gyro_fs_dps"), 2);
  EXPECT_EQ(nearest_code(kOdrCodes, 100.0, "odr_hz"), 8);
}

TEST(Icm42688, NearestCodeRejectsUnsupported) {
  try {
    nearest_code(kGyroFsCodes, 123.0, "gyro_fs_dps");
    FAIL() << "应当抛出 std::invalid_argument";
  } catch (const std::invalid_argument& exc) {
    EXPECT_NE(std::string(exc.what()).find("不支持"), std::string::npos);
  }
}
