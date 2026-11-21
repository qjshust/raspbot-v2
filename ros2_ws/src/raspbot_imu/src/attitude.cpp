#include "raspbot_imu/attitude.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "raspbot_imu/quaternion.hpp"

namespace raspbot_imu {
namespace {

double norm3(const Vec3& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
  return {
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
  };
}

/// 指数映射精确积分四元数（对应 Python 版 attitude._integrate）。
Quat integrate(const Quat& q, const Vec3& omega, double dt) {
  const double rate = norm3(omega);
  if (rate < 1e-12 || dt <= 0.0) {
    return quat_normalize(q);
  }
  const double half = 0.5 * rate * dt;
  const double sin_half = std::sin(half);
  const Quat delta{std::cos(half),
                   sin_half * omega[0] / rate,
                   sin_half * omega[1] / rate,
                   sin_half * omega[2] / rate};
  return quat_normalize(quat_multiply(q, delta));
}

}  // namespace

MahonyAHRS::MahonyAHRS(double kp, double ki, double accel_gate, double max_bias)
    : kp(kp), ki(ki), accel_gate(accel_gate), max_bias(max_bias) {}

void MahonyAHRS::reset(double yaw) {
  q = quat_from_yaw(yaw);
  bias = {0.0, 0.0, 0.0};
}

Quat MahonyAHRS::update(const Vec3& gyro, const Vec3& accel, double dt) {
  if (dt <= 0.0) {
    return q;
  }

  Vec3 omega{gyro[0] - bias[0], gyro[1] - bias[1], gyro[2] - bias[2]};
  const double accel_norm = norm3(accel);
  // 只在加速度模长接近 g 时使用重力修正，滤掉加减速与冲击。
  if (accel_norm > 1e-3 &&
      std::fabs(accel_norm - kGravity) <= accel_gate * kGravity) {
    const Vec3 measured{accel[0] / accel_norm, accel[1] / accel_norm,
                        accel[2] / accel_norm};
    // 估计的机体坐标系重力方向 = R^T * [0, 0, 1]
    const Vec3 estimated = mat_t_vec(quat_to_matrix(q), {0.0, 0.0, 1.0});
    const Vec3 error = cross3(measured, estimated);
    for (int i = 0; i < 3; ++i) {
      omega[i] += kp * error[i];
      bias[i] = std::clamp(bias[i] - ki * error[i] * dt, -max_bias, max_bias);
    }
  }

  q = integrate(q, omega, dt);
  return q;
}

YawEstimator::YawEstimator(std::optional<Vec3> bias, double stationary_gyro,
                           double stationary_accel, double bias_tau)
    : stationary_gyro(stationary_gyro),
      stationary_accel(stationary_accel),
      bias_tau(bias_tau) {
  if (bias) {
    this->bias = *bias;
  }
}

void YawEstimator::reset(double yaw) {
  this->yaw = wrap_pi(yaw);
  still_time_ = 0.0;
}

double YawEstimator::update(const Vec3& gyro, const Vec3& accel, double dt) {
  if (dt <= 0.0) {
    return yaw;
  }

  yaw = wrap_pi(yaw + (gyro[2] - bias[2]) * dt);

  const double accel_norm = norm3(accel);
  const bool still = norm3(gyro) < stationary_gyro &&
                     std::fabs(accel_norm - kGravity) <
                         stationary_accel * kGravity;
  if (still) {
    still_time_ += dt;
    if (still_time_ > 0.5) {
      const double alpha = std::min(1.0, dt / bias_tau);
      bias[2] += alpha * (gyro[2] - bias[2]);
    }
  } else {
    still_time_ = 0.0;
  }
  return yaw;
}

Vec3 estimate_gyro_bias(const std::vector<Vec3>& samples, double max_std) {
  const std::size_t n = samples.size();
  if (n < 10) {
    throw std::invalid_argument(
        "静止标定需要至少 10 帧 (N,3) 数据，收到 " + std::to_string(n) +
        " 帧");
  }

  Vec3 mean{0.0, 0.0, 0.0};
  for (const Vec3& s : samples) {
    for (int i = 0; i < 3; ++i) {
      mean[i] += s[i];
    }
  }
  for (int i = 0; i < 3; ++i) {
    mean[i] /= static_cast<double>(n);
  }

  Vec3 sd{0.0, 0.0, 0.0};
  for (const Vec3& s : samples) {
    for (int i = 0; i < 3; ++i) {
      const double d = s[i] - mean[i];
      sd[i] += d * d;
    }
  }
  double sd_max = 0.0;
  for (int i = 0; i < 3; ++i) {
    sd[i] = std::sqrt(sd[i] / static_cast<double>(n));
    sd_max = std::max(sd_max, sd[i]);
  }
  if (sd_max > max_std) {
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "静止标定失败：角速度标准差 %.4f rad/s 超过 %.4f，"
                  "采样期间车辆可能在动",
                  sd_max, max_std);
    throw std::invalid_argument(buf);
  }
  return mean;
}

}  // namespace raspbot_imu
