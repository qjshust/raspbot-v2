// Copyright (c) 2026 RASPBOT-V2 WP-H
// 运动学/里程计纯函数用例（C++ 版新增，Python 版无对应测试）。
// 覆盖 fake_base 依赖的半隐式积分与 yaw→四元数。

#include <cmath>
#include <gtest/gtest.h>

#include "raspbot_mock/kinematics.hpp"

using raspbot_mock::Pose2D;
using raspbot_mock::Quat;
using raspbot_mock::integrate_pose;
using raspbot_mock::sanitize_dt;
using raspbot_mock::yaw_to_quat;

// -std=c++17 下 glibc 隐藏 M_PI
constexpr double kPi = 3.14159265358979323846;

TEST(KinematicsTest, SanitizeDtGuards)
{
  EXPECT_DOUBLE_EQ(sanitize_dt(-0.1), 0.0);
  EXPECT_DOUBLE_EQ(sanitize_dt(0.0), 0.0);
  EXPECT_DOUBLE_EQ(sanitize_dt(1.0001), 0.0);
  EXPECT_DOUBLE_EQ(sanitize_dt(0.5), 0.5);
  EXPECT_DOUBLE_EQ(sanitize_dt(1.0), 1.0);
}

TEST(KinematicsTest, IntegratePoseStraightLine)
{
  // vx=1.0 m/s 直行 1 s（10 步 × 0.1 s）
  Pose2D p;
  for (int i = 0; i < 10; ++i) {
    p = integrate_pose(p, 1.0, 0.0, 0.0, 0.1);
  }
  EXPECT_NEAR(p.x, 1.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(p.theta, 0.0, 1e-9);
}

TEST(KinematicsTest, IntegratePoseThetaBeforeTranslation)
{
  // 半隐式顺序：一步 dt=0.1、wz=1.0、vx=1.0 → theta=0.1，
  // x += cos(0.1)*0.1（而不是用旧 theta=0 平移 0.1）
  const Pose2D p = integrate_pose(Pose2D{}, 1.0, 0.0, 1.0, 0.1);
  EXPECT_NEAR(p.theta, 0.1, 1e-12);
  EXPECT_NEAR(p.x, std::cos(0.1) * 0.1, 1e-12);
  EXPECT_NEAR(p.y, std::sin(0.1) * 0.1, 1e-12);
}

TEST(KinematicsTest, YawToQuat)
{
  const Quat q0 = yaw_to_quat(0.0);
  EXPECT_DOUBLE_EQ(q0.x, 0.0);
  EXPECT_DOUBLE_EQ(q0.y, 0.0);
  EXPECT_DOUBLE_EQ(q0.z, 0.0);
  EXPECT_DOUBLE_EQ(q0.w, 1.0);

  const Quat q = yaw_to_quat(kPi / 2.0);
  EXPECT_NEAR(q.z, std::sin(kPi / 4.0), 1e-12);
  EXPECT_NEAR(q.w, std::cos(kPi / 4.0), 1e-12);
  // 单位四元数
  EXPECT_NEAR(q.z * q.z + q.w * q.w, 1.0, 1e-12);
}
