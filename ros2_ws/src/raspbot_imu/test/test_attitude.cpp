/// 姿态解算测试（1:1 映射 Python 版 test/test_attitude.py，15 用例）。
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "raspbot_imu/attitude.hpp"
#include "raspbot_imu/quaternion.hpp"
#include "raspbot_imu/types.hpp"

using namespace raspbot_imu;

TEST(Attitude, QuatYawRoundtrip) {
  for (double yaw : {-3.0, -1.2, 0.0, 0.4, 2.5}) {
    EXPECT_LT(std::fabs(wrap_pi(quat_yaw(quat_from_yaw(yaw)) - yaw)), 1e-9);
  }
}

TEST(Attitude, QuatComposeYaw) {
  const Quat q = quat_multiply(quat_from_yaw(0.3), quat_from_yaw(0.2));
  EXPECT_LT(std::fabs(wrap_pi(quat_yaw(q) - 0.5)), 1e-9);
}

TEST(Attitude, QuatMatrixRotatesBodyXToHeading) {
  const double yaw = 1.2;
  const Mat33 rotation = quat_to_matrix(quat_from_yaw(yaw));
  const Vec3 x_rot = mat_vec(rotation, {1.0, 0.0, 0.0});
  EXPECT_NEAR(x_rot[0], std::cos(yaw), 1e-9);
  EXPECT_NEAR(x_rot[1], std::sin(yaw), 1e-9);
  EXPECT_NEAR(x_rot[2], 0.0, 1e-9);
  const Vec3 z_rot = mat_vec(rotation, {0.0, 0.0, 1.0});
  EXPECT_NEAR(z_rot[0], 0.0, 1e-9);
  EXPECT_NEAR(z_rot[1], 0.0, 1e-9);
  EXPECT_NEAR(z_rot[2], 1.0, 1e-9);
}

TEST(Attitude, WrapPi) {
  EXPECT_NEAR(wrap_pi(kPi + 0.1), -kPi + 0.1, 1e-9);
  EXPECT_NEAR(wrap_pi(-kPi - 0.1), kPi - 0.1, 1e-9);
}

TEST(Attitude, ImuSampleAcceptsLists) {
  const ImuSample sample(1.0, {0.0, 0.0, 1.0}, {0.0, 0.0, kGravity});
  EXPECT_EQ(sample.gyro.size(), 3u);
  EXPECT_NEAR(sample.accel[2], kGravity, 1e-12);
}

TEST(Attitude, BiasEstimateStatic) {
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.002);
  const Vec3 bias{0.010, -0.020, 0.030};
  std::vector<Vec3> samples(2000);
  for (Vec3& s : samples) {
    for (int i = 0; i < 3; ++i) {
      s[i] = bias[i] + noise(rng);
    }
  }
  const Vec3 estimated = estimate_gyro_bias(samples);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(estimated[i], bias[i], 2e-3);
  }
}

TEST(Attitude, BiasEstimateRejectsMotion) {
  std::mt19937 rng(8);
  std::normal_distribution<double> noise(0.0, 0.2);
  std::vector<Vec3> samples(500);
  for (Vec3& s : samples) {
    for (int i = 0; i < 3; ++i) {
      s[i] = noise(rng);
    }
  }
  try {
    estimate_gyro_bias(samples);
    FAIL() << "应当抛出 std::invalid_argument";
  } catch (const std::invalid_argument& exc) {
    EXPECT_NE(std::string(exc.what()).find("静止标定"), std::string::npos);
  }
}

TEST(Attitude, BiasEstimateRejectsTooFewSamples) {
  const std::vector<Vec3> samples(5, Vec3{0.0, 0.0, 0.0});
  try {
    estimate_gyro_bias(samples);
    FAIL() << "应当抛出 std::invalid_argument";
  } catch (const std::invalid_argument& exc) {
    EXPECT_NE(std::string(exc.what()).find("至少 10 帧"), std::string::npos);
  }
}

TEST(Attitude, YawIntegratesRate) {
  YawEstimator estimator;
  const double rate = kPi / 2.0;  // 90 °/s
  double yaw = 0.0;
  for (int i = 0; i < 200; ++i) {  // 2 s
    yaw = estimator.update({0.0, 0.0, rate}, {0.0, 0.0, kGravity}, 0.01);
  }
  EXPECT_LT(std::fabs(std::fabs(yaw) - kPi), 0.01);
}

TEST(Attitude, YawCompensatesKnownBias) {
  const Vec3 bias{0.0, 0.0, 0.02};
  YawEstimator estimator(bias);
  double yaw = 0.0;
  for (int i = 0; i < 100; ++i) {  // 1 s，真实角速度 0.5 rad/s
    yaw = estimator.update({0.0, 0.0, 0.5 + bias[2]},
                           {0.0, 0.0, kGravity}, 0.01);
  }
  EXPECT_NEAR(yaw, 0.5, 1e-9);
}

TEST(Attitude, YawTracksBiasWhenStationary) {
  YawEstimator estimator;
  for (int i = 0; i < 3000; ++i) {  // 30 s 静止，真实零偏 0.01 rad/s
    estimator.update({0.0, 0.0, 0.01}, {0.0, 0.0, kGravity}, 0.01);
  }
  EXPECT_GT(estimator.bias[2], 0.006);
  EXPECT_LT(estimator.bias[2], 0.011);
}

TEST(Attitude, YawDoesNotTrackBiasWhenMoving) {
  YawEstimator estimator;
  for (int i = 0; i < 3000; ++i) {
    estimator.update({0.0, 0.0, 0.5}, {0.0, 0.0, kGravity}, 0.01);
  }
  EXPECT_LT(std::fabs(estimator.bias[2]), 1e-6);
}

TEST(Attitude, MahonyLevelsFromRest) {
  MahonyAHRS ahrs;
  for (int i = 0; i < 1000; ++i) {  // 10 s
    ahrs.update({0.0, 0.0, 0.0}, {0.0, 0.0, kGravity}, 0.01);
  }
  const auto [roll, pitch] = quat_roll_pitch(ahrs.q);
  EXPECT_LT(std::fabs(roll), 0.01);
  EXPECT_LT(std::fabs(pitch), 0.01);
}

TEST(Attitude, MahonyRecoversRollFromGravity) {
  const double roll_ref = kPi / 180.0 * 30.0;
  // 机体绕 x 轴滚转 30° 时，加速度计读数 = R^T * [0,0,g]
  const Vec3 accel{0.0, kGravity * std::sin(roll_ref),
                   kGravity * std::cos(roll_ref)};
  MahonyAHRS ahrs(2.0, 0.0);
  for (int i = 0; i < 2000; ++i) {  // 20 s
    ahrs.update({0.0, 0.0, 0.0}, accel, 0.01);
  }
  const auto [roll, pitch] = quat_roll_pitch(ahrs.q);
  EXPECT_NEAR(roll, roll_ref, 0.02);
  EXPECT_LT(std::fabs(pitch), 0.02);
}

TEST(Attitude, MahonyGatesLinearAcceleration) {
  MahonyAHRS ahrs(2.0, 0.0);
  for (int i = 0; i < 1000; ++i) {
    ahrs.update({0.0, 0.0, 0.0}, {0.0, 0.0, kGravity}, 0.01);
  }
  for (int i = 0; i < 100; ++i) {  // 1 s 的 3 g 冲击，不应把姿态带偏
    ahrs.update({0.0, 0.0, 0.0}, {0.0, 0.0, 3.0 * kGravity}, 0.01);
  }
  const auto [roll, pitch] = quat_roll_pitch(ahrs.q);
  EXPECT_LT(std::fabs(roll), 0.02);
  EXPECT_LT(std::fabs(pitch), 0.02);
}
