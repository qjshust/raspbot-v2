// Pure dead-reckoning helpers for /odom_raw (no ROS types).
//
// The chassis has no wheel encoders, so the node integrates the *commanded*
// velocity - open loop, drift included. Mirrors raspbot_base/odometry.py.
#pragma once

#include <array>

namespace raspbot_base {

struct Pose2D {
  double x;
  double y;
  double theta;
};

/// Guard the integration time step.
///
/// dt <= 0 (clock jump backwards) or dt > 1.0 s (long stall) returns 0.0 so
/// the pose cannot jump; otherwise the original value is returned unchanged.
double sanitize_dt(double dt);

/// Semi-implicit Euler step; order identical to the legacy Python code:
/// first theta += wz*dt, then x/y are updated with the *new* theta:
///   x += (vx*cos(theta) - vy*sin(theta)) * dt
///   y += (vx*sin(theta) + vy*cos(theta)) * dt
Pose2D integrate_pose(double x, double y, double theta, double vx, double vy,
                      double wz, double dt);

/// Planar yaw -> unit quaternion (x, y, z, w) with x = y = 0.
std::array<double, 4> yaw_to_quat(double theta);

}  // namespace raspbot_base
