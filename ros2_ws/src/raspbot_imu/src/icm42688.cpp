#include "raspbot_imu/icm42688.hpp"

#include <cmath>
#include <sstream>

namespace raspbot_imu {

const std::map<double, int> kGyroFsCodes = {
    {2000.0, 0}, {1000.0, 1}, {500.0, 2}, {250.0, 3},
    {125.0, 4},  {62.5, 5},   {31.25, 6}, {15.625, 7},
};
const std::map<double, int> kAccelFsCodes = {
    {16.0, 0}, {8.0, 1}, {4.0, 2}, {2.0, 3},
};
const std::map<double, int> kOdrCodes = {
    {8000.0, 3}, {4000.0, 4}, {2000.0, 5}, {1000.0, 6}, {500.0, 15},
    {200.0, 7},  {100.0, 8},  {50.0, 9},   {25.0, 10},  {12.5, 11},
};

int nearest_code(const std::map<double, int>& table, double value,
                 const std::string& name) {
  for (const auto& kv : table) {
    const double tol = 1e-6 * std::max(std::fabs(kv.first), std::fabs(value));
    if (std::fabs(kv.first - value) <= tol) {
      return kv.second;
    }
  }
  std::ostringstream oss;
  oss << "不支持的 " << name << "=" << value << "，可选：";
  bool first = true;
  for (const auto& kv : table) {
    if (!first) {
      oss << ", ";
    }
    oss << kv.first;
    first = false;
  }
  throw std::invalid_argument(oss.str());
}

std::pair<Vec3, Vec3> parse_burst(const std::uint8_t* buf, std::size_t len,
                                  double gyro_fs_dps, double accel_fs_g) {
  if (len < 14) {
    throw std::invalid_argument("数据长度应为 14 字节，收到 " +
                                std::to_string(len));
  }
  const auto i16be = [&buf](std::size_t off) {
    return static_cast<std::int16_t>((buf[off] << 8) | buf[off + 1]);
  };
  // 布局：温度(2) + 加速度 XYZ(6) + 陀螺 XYZ(6)
  const std::int16_t raw_accel[3] = {i16be(2), i16be(4), i16be(6)};
  const std::int16_t raw_gyro[3] = {i16be(8), i16be(10), i16be(12)};
  for (int i = 0; i < 3; ++i) {
    if (raw_accel[i] == -32768 || raw_gyro[i] == -32768) {
      throw ImuDataInvalidError("ICM-42688-P 返回数据无效标记");
    }
  }
  const double gyro_scale = gyro_fs_dps / 32768.0 * kPi / 180.0;
  const double accel_scale = accel_fs_g / 32768.0 * kGravity;
  Vec3 gyro{raw_gyro[0] * gyro_scale, raw_gyro[1] * gyro_scale,
            raw_gyro[2] * gyro_scale};
  Vec3 accel{raw_accel[0] * accel_scale, raw_accel[1] * accel_scale,
             raw_accel[2] * accel_scale};
  return {gyro, accel};
}

}  // namespace raspbot_imu
