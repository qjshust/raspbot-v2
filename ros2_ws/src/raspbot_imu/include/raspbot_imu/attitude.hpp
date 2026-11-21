#pragma once

#include <optional>
#include <vector>

#include "raspbot_imu/types.hpp"

namespace raspbot_imu {

/// 6 轴 Mahony 互补滤波（对应 Python 版 attitude.MahonyAHRS）。
class MahonyAHRS {
 public:
  explicit MahonyAHRS(double kp = 0.8, double ki = 0.02,
                      double accel_gate = 0.25, double max_bias = 0.5);

  /// 重置姿态到水平、给定航向。
  void reset(double yaw = 0.0);

  /// 推进一个采样周期，返回更新后的四元数。
  Quat update(const Vec3& gyro, const Vec3& accel, double dt);

  Quat q{1.0, 0.0, 0.0, 0.0};
  Vec3 bias{0.0, 0.0, 0.0};
  double kp;
  double ki;
  double accel_gate;
  double max_bias;
};

/// 地面车航向估计：积分 Z 轴角速度 + 静止零偏跟踪（对应 Python 版 YawEstimator）。
class YawEstimator {
 public:
  explicit YawEstimator(std::optional<Vec3> bias = std::nullopt,
                        double stationary_gyro = 0.02,
                        double stationary_accel = 0.12,
                        double bias_tau = 8.0);

  void reset(double yaw = 0.0);

  /// 推进一个采样周期，返回折回 (-pi, pi] 的航向角（rad）。
  double update(const Vec3& gyro, const Vec3& accel, double dt);

  Vec3 bias{0.0, 0.0, 0.0};
  double yaw = 0.0;
  double stationary_gyro;
  double stationary_accel;
  double bias_tau;

 private:
  double still_time_ = 0.0;
};

/// 由静止样本估计三轴陀螺零偏；样本不足或噪声过大抛 std::invalid_argument。
Vec3 estimate_gyro_bias(const std::vector<Vec3>& samples, double max_std = 0.05);

}  // namespace raspbot_imu
