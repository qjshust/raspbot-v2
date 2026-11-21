#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "raspbot_imu/types.hpp"

namespace raspbot_imu {

/// 维特智能 0x55 帧解析器（JY/HWT 系列通用，对应 Python 版 WitMotionParser）。
///
/// 帧格式：``0x55 | type | d0..d7 | checksum``，共 11 字节；
/// 校验和 = 前 10 字节之和的低 8 位。数据为小端 int16。
class WitMotionParser {
 public:
  static constexpr std::uint8_t kAccel = 0x51;
  static constexpr std::uint8_t kGyro = 0x52;
  static constexpr std::uint8_t kAngle = 0x53;

  explicit WitMotionParser(double accel_fs_g = 16.0, double gyro_fs_dps = 2000.0);

  /// 喂入字节流；解析到至少一帧返回 true。
  bool feed(const std::uint8_t* data, std::size_t n);
  bool feed(const std::vector<std::uint8_t>& data) {
    return feed(data.data(), data.size());
  }

  Vec3 gyro{0.0, 0.0, 0.0};       // rad/s
  Vec3 accel{0.0, 0.0, 0.0};      // m/s^2
  Vec3 angle_deg{0.0, 0.0, 0.0};  // 度
  bool seen_accel = false;
  bool seen_angle = false;
  double accel_fs_g;
  double gyro_fs_dps;

 private:
  void handle(const std::uint8_t frame[11]);

  std::vector<std::uint8_t> buf_;
};

}  // namespace raspbot_imu
