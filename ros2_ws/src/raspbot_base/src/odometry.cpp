#include "raspbot_base/odometry.hpp"

#include <cmath>

namespace raspbot_base {

double sanitize_dt(double dt) {
  if (dt <= 0.0 || dt > 1.0) {
    return 0.0;
  }
  return dt;
}

Pose2D integrate_pose(double x, double y, double theta, double vx, double vy,
                      double wz, double dt) {
  const double th = theta + wz * dt;
  return {x + (vx * std::cos(th) - vy * std::sin(th)) * dt,
          y + (vx * std::sin(th) + vy * std::cos(th)) * dt, th};
}

std::array<double, 4> yaw_to_quat(double theta) {
  return {0.0, 0.0, std::sin(theta / 2.0), std::cos(theta / 2.0)};
}

}  // namespace raspbot_base
