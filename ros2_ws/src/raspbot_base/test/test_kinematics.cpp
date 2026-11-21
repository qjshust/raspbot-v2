// 1:1 mapping of legacy test/test_kinematics.py (SW-U01..U04, 19 cases) plus
// two extras (banker's rounding and the vendor motor-command mapping).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "raspbot_base/kinematics.hpp"

using raspbot_base::Cmd3;
using raspbot_base::MotorCmd;
using raspbot_base::WheelSpeeds;
using raspbot_base::clamp_velocity;
using raspbot_base::motor_command;
using raspbot_base::round_half_even;
using raspbot_base::wheel_linear_velocities;
using raspbot_base::wheel_pwm;

namespace {

constexpr double kK = 0.1245;  // k_geom from config/params.yaml

void expect_pwm(const std::array<int, 4>& actual, int e0, int e1, int e2,
                int e3) {
  EXPECT_EQ(actual[0], e0);
  EXPECT_EQ(actual[1], e1);
  EXPECT_EQ(actual[2], e2);
  EXPECT_EQ(actual[3], e3);
}

void expect_ws_close(const WheelSpeeds& actual, const WheelSpeeds& expected) {
  EXPECT_NEAR(actual.l1, expected.l1, 1e-9);
  EXPECT_NEAR(actual.l2, expected.l2, 1e-9);
  EXPECT_NEAR(actual.r1, expected.r1, 1e-9);
  EXPECT_NEAR(actual.r2, expected.r2, 1e-9);
}

// ------------------------------------------------------------- SW-U01

TEST(SwU01, Forward) {
  expect_ws_close(wheel_linear_velocities(0.2, 0.0, 0.0, kK),
                  {0.2, 0.2, 0.2, 0.2});
}

TEST(SwU01, Backward) {
  expect_ws_close(wheel_linear_velocities(-0.2, 0.0, 0.0, kK),
                  {-0.2, -0.2, -0.2, -0.2});
}

TEST(SwU01, StrafeLeft) {
  expect_ws_close(wheel_linear_velocities(0.0, 0.2, 0.0, kK),
                  {-0.2, 0.2, 0.2, -0.2});
}

TEST(SwU01, StrafeRight) {
  expect_ws_close(wheel_linear_velocities(0.0, -0.2, 0.0, kK),
                  {0.2, -0.2, -0.2, 0.2});
}

TEST(SwU01, RotateLeftCcw) {
  const double k = kK * 0.5;
  expect_ws_close(wheel_linear_velocities(0.0, 0.0, 0.5, kK),
                  {-k, k, -k, k});
}

TEST(SwU01, RotateRightCw) {
  const double k = kK * 0.5;
  expect_ws_close(wheel_linear_velocities(0.0, 0.0, -0.5, kK),
                  {k, -k, k, -k});
}

TEST(SwU01, MixedCommandMatchesFormula) {
  const double vx = 0.1, vy = 0.05, wz = 0.2;
  const double k = kK * wz;
  expect_ws_close(wheel_linear_velocities(vx, vy, wz, kK),
                  {vx - vy - k, vx + vy + k, vx + vy - k, vx - vy + k});
}

// ------------------------------------------------------------- SW-U02

TEST(SwU02, PeakScaledToMaxPwm) {
  const auto out = wheel_pwm(1.0, 0.0, 0.0, 700.0, 255, 25);
  expect_pwm(out, 255, 255, 255, 255);
}

TEST(SwU02, ProportionalScalingWithin1Lsb) {
  // effective wz already includes k_geom: raw = (0.2, 0.8, 0.8, 0.2)
  const double vx = 0.5, vy = 0.0, wz = 0.3;
  const WheelSpeeds raw = wheel_linear_velocities(vx, vy, wz, 1.0);
  const double peak = std::max({std::fabs(raw.l1), std::fabs(raw.l2),
                                std::fabs(raw.r1), std::fabs(raw.r2)});
  EXPECT_GT(peak, 255.0 / 700.0);

  const auto out = wheel_pwm(vx, vy, wz, 700.0, 255, 25);
  EXPECT_EQ(std::max({out[0], out[1], out[2], out[3]}), 255);
  const double raw_arr[4] = {raw.l1, raw.l2, raw.r1, raw.r2};
  for (int i = 0; i < 4; ++i) {
    const double exact = raw_arr[i] * (255.0 / peak);
    EXPECT_LT(std::fabs(out[i] - exact), 1.0);
  }
}

TEST(SwU02, NoScalingWhenBelowMax) {
  const auto out = wheel_pwm(0.2, 0.0, 0.0, 700.0, 255, 25);
  expect_pwm(out, 140, 140, 140, 140);
}

// ------------------------------------------------------------- SW-U03

namespace {
std::array<int, 4> pwm_for(double vx) {
  return wheel_pwm(vx, 0.0, 0.0, 100.0, 255, 25);
}
}  // namespace

TEST(SwU03, ZeroStaysZero) {
  expect_pwm(pwm_for(0.0), 0, 0, 0, 0);
}

TEST(SwU03, BelowDeadbandMapsToDeadband) {
  expect_pwm(pwm_for(0.01), 25, 25, 25, 25);
  expect_pwm(pwm_for(0.24), 25, 25, 25, 25);
  expect_pwm(pwm_for(-0.01), -25, -25, -25, -25);
  expect_pwm(pwm_for(-0.24), -25, -25, -25, -25);
}

TEST(SwU03, DeadbandBoundaryKept) {
  expect_pwm(pwm_for(0.25), 25, 25, 25, 25);
  expect_pwm(pwm_for(-0.25), -25, -25, -25, -25);
}

TEST(SwU03, AboveDeadbandUnchanged) {
  expect_pwm(pwm_for(0.26), 26, 26, 26, 26);
  expect_pwm(pwm_for(-0.26), -26, -26, -26, -26);
}

// ------------------------------------------------------------- SW-U04

TEST(SwU04, BaseSignsAllPositive) {
  const auto out = wheel_pwm(0.2, 0.0, 0.0, 100.0, 255, 1);
  expect_pwm(out, 20, 20, 20, 20);
}

TEST(SwU04, EachWheelIndependent) {
  expect_pwm(wheel_pwm(0.2, 0.0, 0.0, 100.0, 255, 1, {-1, 1, 1, 1}), -20, 20,
             20, 20);
  expect_pwm(wheel_pwm(0.2, 0.0, 0.0, 100.0, 255, 1, {1, -1, 1, 1}), 20, -20,
             20, 20);
  expect_pwm(wheel_pwm(0.2, 0.0, 0.0, 100.0, 255, 1, {1, 1, -1, 1}), 20, 20,
             -20, 20);
  expect_pwm(wheel_pwm(0.2, 0.0, 0.0, 100.0, 255, 1, {1, 1, 1, -1}), 20, 20,
             20, -20);
}

TEST(SwU04, SignAppliedBeforeDeadband) {
  // -0.01 m/s -> pwm -1 -> sign -1 -> +1 -> deadband +25, not -25
  const auto out = wheel_pwm(-0.01, 0.0, 0.0, 100.0, 255, 25, {-1, 1, 1, 1});
  expect_pwm(out, 25, -25, -25, -25);
}

// -------------------------------------------------------- clamp_velocity

TEST(ClampVelocity, Limits) {
  const Cmd3 c = clamp_velocity(1.0, -1.0, 5.0, 0.3, 1.5);
  EXPECT_DOUBLE_EQ(c.vx, 0.3);
  EXPECT_DOUBLE_EQ(c.vy, -0.3);
  EXPECT_DOUBLE_EQ(c.wz, 1.5);
}

TEST(ClampVelocity, Passthrough) {
  const Cmd3 c = clamp_velocity(0.1, 0.05, 0.2, 0.3, 1.5);
  EXPECT_DOUBLE_EQ(c.vx, 0.1);
  EXPECT_DOUBLE_EQ(c.vy, 0.05);
  EXPECT_DOUBLE_EQ(c.wz, 0.2);
}

// ----------------------------------------------------------------- extras

TEST(RoundHalfEven, MatchesPythonRound) {
  EXPECT_DOUBLE_EQ(round_half_even(2.5), 2.0);
  EXPECT_DOUBLE_EQ(round_half_even(3.5), 4.0);
  EXPECT_DOUBLE_EQ(round_half_even(-2.5), -2.0);
  EXPECT_DOUBLE_EQ(round_half_even(17.57), 18.0);
  EXPECT_DOUBLE_EQ(round_half_even(122.43), 122.0);
}

TEST(MotorCommand, VendorCtrlMutoSemantics) {
  const MotorCmd fwd = motor_command(140);
  EXPECT_EQ(fwd.dir, 0);
  EXPECT_EQ(fwd.speed, 140);
  const MotorCmd rev = motor_command(-140);
  EXPECT_EQ(rev.dir, 1);
  EXPECT_EQ(rev.speed, 140);
  const MotorCmd zero = motor_command(0);
  EXPECT_EQ(zero.dir, 0);
  EXPECT_EQ(zero.speed, 0);
  const MotorCmd hi = motor_command(300);
  EXPECT_EQ(hi.dir, 0);
  EXPECT_EQ(hi.speed, 255);
  const MotorCmd lo = motor_command(-300);
  EXPECT_EQ(lo.dir, 1);
  EXPECT_EQ(lo.speed, 255);
}

}  // namespace
