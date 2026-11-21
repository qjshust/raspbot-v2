// Copyright (c) 2026 RASPBOT-V2 WP-H

#include "raspbot_mock/kinematics.hpp"

#include <cmath>

namespace raspbot_mock
{

double sanitize_dt(double dt)
{
  return (dt <= 0.0 || dt > 1.0) ? 0.0 : dt;
}

Pose2D integrate_pose(const Pose2D & pose, double vx, double vy, double wz, double dt)
{
  Pose2D out;
  out.theta = pose.theta + wz * dt;
  out.x = pose.x + (vx * std::cos(out.theta) - vy * std::sin(out.theta)) * dt;
  out.y = pose.y + (vx * std::sin(out.theta) + vy * std::cos(out.theta)) * dt;
  return out;
}

Quat yaw_to_quat(double theta)
{
  Quat q;
  q.z = std::sin(theta / 2.0);
  q.w = std::cos(theta / 2.0);
  return q;
}

}  // namespace raspbot_mock
