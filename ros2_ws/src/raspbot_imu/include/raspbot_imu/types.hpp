#pragma once

#include <array>
#include <optional>

namespace raspbot_imu {

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // [w, x, y, z]
using Mat33 = std::array<double, 9>; // 行主序 3x3

/// 一帧 IMU 数据（对应 Python 版 attitude.ImuSample）。
struct ImuSample {
  double t = 0.0;
  Vec3 gyro{0.0, 0.0, 0.0};
  Vec3 accel{0.0, 0.0, 0.0};
  /// 器件自带航向角（度，绕 +Z 逆时针为正）；没有则无值。
  std::optional<double> yaw_deg;

  ImuSample() = default;
  ImuSample(double t, const Vec3& gyro, const Vec3& accel,
            std::optional<double> yaw_deg = std::nullopt)
      : t(t), gyro(gyro), accel(accel), yaw_deg(yaw_deg) {}
};

}  // namespace raspbot_imu
