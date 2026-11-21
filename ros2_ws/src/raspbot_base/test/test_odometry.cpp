// 1:1 mapping of legacy test/test_odometry.py (SW-U05, SW-U06, 9 cases).
#include <gtest/gtest.h>

#include <cmath>

#include "raspbot_base/odometry.hpp"

using raspbot_base::Pose2D;
using raspbot_base::integrate_pose;
using raspbot_base::sanitize_dt;
using raspbot_base::yaw_to_quat;

namespace {

// ------------------------------------------------------------- SW-U05

TEST(SwU05, StraightExact) {
  // 0.2 m/s for 10 s, single step: x must be exactly 2.0
  const Pose2D p = integrate_pose(0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 10.0);
  EXPECT_DOUBLE_EQ(p.x, 2.0);
  EXPECT_DOUBLE_EQ(p.y, 0.0);
  EXPECT_DOUBLE_EQ(p.theta, 0.0);
}

TEST(SwU05, StraightMultistep) {
  double x = 0.0, y = 0.0, th = 0.0;
  for (int i = 0; i < 100; ++i) {
    const Pose2D p = integrate_pose(x, y, th, 0.2, 0.0, 0.0, 0.1);
    x = p.x;
    y = p.y;
    th = p.theta;
  }
  EXPECT_NEAR(x, 2.0, 1e-9);
  EXPECT_NEAR(y, 0.0, 1e-9);
  EXPECT_NEAR(th, 0.0, 1e-9);
}

TEST(SwU05, Diagonal) {
  const Pose2D p = integrate_pose(0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 2.0);
  EXPECT_NEAR(p.x, 0.4, 1e-9);
  EXPECT_NEAR(p.y, 0.4, 1e-9);
  EXPECT_DOUBLE_EQ(p.theta, 0.0);
}

TEST(SwU05, RotateInPlace) {
  const Pose2D p = integrate_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 2.0);
  EXPECT_DOUBLE_EQ(p.x, 0.0);
  EXPECT_DOUBLE_EQ(p.y, 0.0);
  EXPECT_NEAR(p.theta, 1.0, 1e-9);
}

TEST(SwU05, StrafeAndTurnRightHandRule) {
  // +vy strafes left, +wz yaws CCW (top view): the heading rotates left so
  // the leftward velocity gains a -x (backward) component.
  const Pose2D p = integrate_pose(0.0, 0.0, 0.0, 0.0, 0.2, 0.5, 0.5);
  EXPECT_NEAR(p.theta, 0.25, 1e-9);
  const double expected_x =
      (0.0 * std::cos(0.25) - 0.2 * std::sin(0.25)) * 0.5;
  const double expected_y =
      (0.0 * std::sin(0.25) + 0.2 * std::cos(0.25)) * 0.5;
  EXPECT_NEAR(p.x, expected_x, 1e-9);
  EXPECT_NEAR(p.y, expected_y, 1e-9);
  EXPECT_LT(p.x, 0.0);
  EXPECT_GT(p.y, 0.0);
}

TEST(SwU05, MultistepMatchesReference) {
  // 400 x 5 ms steps; the reference recomputes the same discrete map
  const double vx = 0.1, vy = 0.05, wz = 0.3, dt = 0.005;
  double xr = 0.0, yr = 0.0, thr = 0.0;
  for (int i = 0; i < 400; ++i) {
    thr = thr + wz * dt;
    xr = xr + (vx * std::cos(thr) - vy * std::sin(thr)) * dt;
    yr = yr + (vx * std::sin(thr) + vy * std::cos(thr)) * dt;
  }
  double x = 0.0, y = 0.0, th = 0.0;
  for (int i = 0; i < 400; ++i) {
    const Pose2D p = integrate_pose(x, y, th, vx, vy, wz, dt);
    x = p.x;
    y = p.y;
    th = p.theta;
  }
  EXPECT_NEAR(x, xr, 1e-9);
  EXPECT_NEAR(y, yr, 1e-9);
  EXPECT_NEAR(th, thr, 1e-9);
}

TEST(YawToQuat, IdentityAndQuarterTurn) {
  const auto q0 = yaw_to_quat(0.0);
  EXPECT_DOUBLE_EQ(q0[0], 0.0);
  EXPECT_DOUBLE_EQ(q0[1], 0.0);
  EXPECT_DOUBLE_EQ(q0[2], 0.0);
  EXPECT_DOUBLE_EQ(q0[3], 1.0);
  const auto q = yaw_to_quat(std::acos(-1.0) / 2.0);
  EXPECT_NEAR(q[0], 0.0, 1e-9);
  EXPECT_NEAR(q[1], 0.0, 1e-9);
  EXPECT_NEAR(q[2], std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(q[3], std::sqrt(0.5), 1e-9);
}

// ------------------------------------------------------------- SW-U06

TEST(SwU06, SanitizeDt) {
  EXPECT_DOUBLE_EQ(sanitize_dt(0.0), 0.0);
  EXPECT_DOUBLE_EQ(sanitize_dt(-0.001), 0.0);
  EXPECT_DOUBLE_EQ(sanitize_dt(-10.0), 0.0);
  EXPECT_DOUBLE_EQ(sanitize_dt(1.5), 0.0);
  EXPECT_DOUBLE_EQ(sanitize_dt(100.0), 0.0);
  EXPECT_DOUBLE_EQ(sanitize_dt(1.0), 1.0);
  EXPECT_DOUBLE_EQ(sanitize_dt(0.5), 0.5);
}

TEST(SwU06, NoPoseJumpOnBadDt) {
  const double x0 = 1.0, y0 = 2.0, th0 = 0.5;
  for (const double bad : {0.0, -0.1, 1.5, 100.0}) {
    const Pose2D p =
        integrate_pose(x0, y0, th0, 0.2, 0.1, 0.3, sanitize_dt(bad));
    EXPECT_DOUBLE_EQ(p.x, x0);
    EXPECT_DOUBLE_EQ(p.y, y0);
    EXPECT_DOUBLE_EQ(p.theta, th0);
  }
}

}  // namespace
