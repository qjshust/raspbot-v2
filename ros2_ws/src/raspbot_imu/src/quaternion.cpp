#include "raspbot_imu/quaternion.hpp"

#include <algorithm>
#include <cmath>

namespace raspbot_imu {

Quat quat_normalize(const Quat& q) {
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] +
                             q[3] * q[3]);
  if (n < 1e-12) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

Quat quat_multiply(const Quat& a, const Quat& b) {
  return {
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  };
}

Mat33 quat_to_matrix(const Quat& q_in) {
  const Quat q = quat_normalize(q_in);
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {
      1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
      2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
      2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y),
  };
}

Vec3 mat_vec(const Mat33& m, const Vec3& v) {
  return {
      m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
      m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
      m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  };
}

Vec3 mat_t_vec(const Mat33& m, const Vec3& v) {
  return {
      m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
      m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
      m[2] * v[0] + m[5] * v[1] + m[8] * v[2],
  };
}

Quat quat_from_yaw(double yaw) {
  const double half = 0.5 * yaw;
  return {std::cos(half), 0.0, 0.0, std::sin(half)};
}

double quat_yaw(const Quat& q_in) {
  const Quat q = quat_normalize(q_in);
  return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                    1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

std::pair<double, double> quat_roll_pitch(const Quat& q_in) {
  const Quat q = quat_normalize(q_in);
  const double roll =
      std::atan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
  const double sin_pitch = 2.0 * (q[0] * q[2] - q[3] * q[1]);
  const double pitch =
      std::asin(std::max(-1.0, std::min(1.0, sin_pitch)));
  return {roll, pitch};
}

double wrap_pi(double angle) {
  double r = std::fmod(angle + kPi, 2.0 * kPi);
  if (r < 0.0) {
    r += 2.0 * kPi;
  }
  return r - kPi;
}

}  // namespace raspbot_imu
