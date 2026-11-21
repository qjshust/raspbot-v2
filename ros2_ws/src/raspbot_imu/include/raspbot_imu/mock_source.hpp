#pragma once

#include <random>

#include "raspbot_imu/sources.hpp"
#include "raspbot_imu/types.hpp"

namespace raspbot_imu {

/// 合成运动数据源（对应 Python 版 sources.MockSource）。
///
/// 航向按分段角速度积分：0–2 s 静止；2–5 s +45 °/s；5–7 s 静止；7–9 s -60 °/s。
class MockSource : public ImuSource {
 public:
  explicit MockSource(double rate_hz = 100.0,
                      const Vec3& bias = {0.010, -0.005, 0.020},
                      double gyro_noise = 0.002,
                      double accel_noise = 0.05,
                      unsigned int seed = 0);

  double yaw_rate(double t) const;

  double true_yaw() const { return yaw_; }
  double time() const { return t_; }

  std::optional<ImuSample> read() override;

 private:
  double rate_hz_;
  double dt_;
  double gyro_noise_;
  double accel_noise_;
  Vec3 bias_;
  double t_ = 0.0;
  double yaw_ = 0.0;
  std::mt19937 rng_;
  std::normal_distribution<double> gyro_dist_;
  std::normal_distribution<double> accel_dist_;
};

}  // namespace raspbot_imu
