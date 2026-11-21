#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include "raspbot_imu/types.hpp"

namespace raspbot_imu {

/// 传感器返回了数据无效标记（-32768）。
struct ImuDataInvalidError : public std::invalid_argument {
  explicit ImuDataInvalidError(const std::string& what_arg)
      : std::invalid_argument(what_arg) {}
};

// 量程/输出速率配置码表（与 TDK 数据手册及内核驱动核对）。
extern const std::map<double, int> kGyroFsCodes;
extern const std::map<double, int> kAccelFsCodes;
extern const std::map<double, int> kOdrCodes;

/// 按表查找最接近的配置码；找不到抛 std::invalid_argument（信息含可选值）。
int nearest_code(const std::map<double, int>& table, double value,
                 const std::string& name);

/// 解析从 0x1D 起连续读取的 14 字节（温度 2 + 加速度 6 + 陀螺 6，大端 int16）。
///
/// 返回 (gyro_rad_s, accel_m_s2)。数据无效抛 ImuDataInvalidError，
/// 长度不足抛 std::invalid_argument。
std::pair<Vec3, Vec3> parse_burst(const std::uint8_t* buf, std::size_t len,
                                  double gyro_fs_dps, double accel_fs_g);

}  // namespace raspbot_imu
