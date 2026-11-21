#include "raspbot_imu/witmotion.hpp"

#include <algorithm>
#include <cmath>

namespace raspbot_imu {

WitMotionParser::WitMotionParser(double accel_fs_g, double gyro_fs_dps)
    : accel_fs_g(accel_fs_g), gyro_fs_dps(gyro_fs_dps) {}

bool WitMotionParser::feed(const std::uint8_t* data, std::size_t n) {
  buf_.insert(buf_.end(), data, data + n);
  bool got = false;
  while (true) {
    std::size_t start = buf_.size();
    for (std::size_t i = 0; i < buf_.size(); ++i) {
      if (buf_[i] == 0x55) {
        start = i;
        break;
      }
    }
    if (start == buf_.size()) {  // 没有帧头
      buf_.clear();
      break;
    }
    if (start > 0) {
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(start));
    }
    if (buf_.size() < 11) {
      break;
    }
    std::uint8_t frame[11];
    std::copy_n(buf_.begin(), 11, frame);
    std::uint32_t sum = 0;
    for (int i = 0; i < 10; ++i) {
      sum += frame[i];
    }
    if ((sum & 0xFF) != frame[10]) {  // 校验失败，丢弃一个字节重新同步
      buf_.erase(buf_.begin());
      continue;
    }
    buf_.erase(buf_.begin(), buf_.begin() + 11);
    handle(frame);
    got = true;
  }
  if (buf_.size() > 4096) {  // 防御性上限，避免异常数据撑爆内存
    buf_.clear();
  }
  return got;
}

void WitMotionParser::handle(const std::uint8_t frame[11]) {
  const std::uint8_t kind = frame[1];
  const auto i16 = [&frame](std::size_t off) {
    return static_cast<std::int16_t>(frame[2 + off] |
                                     (frame[3 + off] << 8));
  };
  const std::int16_t x = i16(0);
  const std::int16_t y = i16(2);
  const std::int16_t z = i16(4);
  if (kind == kGyro) {
    const double scale = gyro_fs_dps / 32768.0 * kPi / 180.0;
    gyro = {x * scale, y * scale, z * scale};
  } else if (kind == kAccel) {
    seen_accel = true;
    const double scale = accel_fs_g / 32768.0 * kGravity;
    accel = {x * scale, y * scale, z * scale};
  } else if (kind == kAngle) {
    seen_angle = true;
    const double scale = 180.0 / 32768.0;
    angle_deg = {x * scale, y * scale, z * scale};
  }
}

}  // namespace raspbot_imu
