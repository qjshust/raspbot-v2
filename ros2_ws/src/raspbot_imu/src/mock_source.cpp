#include "raspbot_imu/mock_source.hpp"

#include <algorithm>
#include <cmath>

namespace raspbot_imu {

MockSource::MockSource(double rate_hz, const Vec3& bias, double gyro_noise,
                       double accel_noise, unsigned int seed)
    : rate_hz_(rate_hz),
      dt_(1.0 / rate_hz),
      gyro_noise_(gyro_noise),
      accel_noise_(accel_noise),
      bias_(bias),
      rng_(seed),
      // normal_distribution 的 stddev 必须 > 0（gtest 用例会以 0 噪声构造）
      gyro_dist_(0.0, std::max(gyro_noise, 1e-12)),
      accel_dist_(0.0, std::max(accel_noise, 1e-12)) {}

double MockSource::yaw_rate(double t) const {
  if (t >= 2.0 && t < 5.0) {
    return kPi / 180.0 * 45.0;
  }
  if (t >= 7.0 && t < 9.0) {
    return -kPi / 180.0 * 60.0;
  }
  return 0.0;
}

std::optional<ImuSample> MockSource::read() {
  const double t = t_;
  const double rate = yaw_rate(t);

  // 折回 (-pi, pi]（与 Python 的 floor-mod 语义一致）
  double y = std::fmod(yaw_ + rate * dt_ + kPi, 2.0 * kPi);
  if (y < 0.0) {
    y += 2.0 * kPi;
  }
  yaw_ = y - kPi;
  t_ += dt_;

  Vec3 gyro{0.0, 0.0, rate};
  Vec3 accel{0.0, 0.0, kGravity};
  for (int i = 0; i < 3; ++i) {
    gyro[i] = bias_[i] + gyro[i] +
              (gyro_noise_ > 0.0 ? gyro_dist_(rng_) : 0.0);
    accel[i] = accel[i] + (accel_noise_ > 0.0 ? accel_dist_(rng_) : 0.0);
  }
  return ImuSample(t, gyro, accel);
}

}  // namespace raspbot_imu
