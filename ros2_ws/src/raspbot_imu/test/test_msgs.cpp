/// IMU 消息组装测试（1:1 映射 Python 版 test/test_msgs.py，6 用例）。
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "raspbot_imu/msgs.hpp"
#include "raspbot_imu/quaternion.hpp"
#include "raspbot_imu/types.hpp"

using namespace raspbot_imu;

namespace {

// 与 config/imu.yaml 一致（HWT101 实际部署值）
constexpr double kYawVar = 0.0003;
constexpr double kTiltVar = 1.0;
constexpr double kGyroVar = 0.0004;
constexpr double kAccelVar = 0.04;

ImuData make_dict(double yaw = 0.0) {
  const Quat q = quat_from_yaw(yaw);
  return build_imu_data(
      1750000000.123, "imu_link", {q[0], q[1], q[2], q[3]},
      {0.1, -0.2, 0.3}, {0.0, 0.0, kGravity}, kYawVar, kTiltVar, kGyroVar,
      kAccelVar);
}

}  // namespace

TEST(Msgs, FrameIdAndStamp) {
  const ImuData data = make_dict();
  EXPECT_EQ(data.frame_id, "imu_link");
  EXPECT_NEAR(data.stamp_sec, 1750000000.123, 1e-6);
}

TEST(Msgs, QuaternionNormIsUnitForDifferentYaws) {
  for (double yaw : {0.0, 0.4, -1.2, 2.9, kPi, -kPi + 0.01}) {
    const ImuData data = make_dict(yaw);
    const double w = data.orientation[0];
    const double x = data.orientation[1];
    const double y = data.orientation[2];
    const double z = data.orientation[3];
    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    EXPECT_LE(std::fabs(norm - 1.0), 1e-6) << "yaw=" << yaw;
  }
}

TEST(Msgs, OrientationPassesThrough) {
  const Quat q = quat_from_yaw(0.7);
  const ImuData data =
      build_imu_data(0.0, "imu_link", {q[0], q[1], q[2], q[3]},
                     {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(data.orientation[0], q[0]);
  EXPECT_DOUBLE_EQ(data.orientation[1], q[1]);
  EXPECT_DOUBLE_EQ(data.orientation[2], q[2]);
  EXPECT_DOUBLE_EQ(data.orientation[3], q[3]);
}

TEST(Msgs, CovarianceDiagonalsMatchYaml) {
  const ImuData data = make_dict();
  const std::array<double, 9> expected_orientation = {
      kTiltVar, 0.0, 0.0,
      0.0, kTiltVar, 0.0,
      0.0, 0.0, kYawVar,
  };
  const std::array<double, 9> expected_gyro = {
      kGyroVar, 0.0, 0.0,
      0.0, kGyroVar, 0.0,
      0.0, 0.0, kGyroVar,
  };
  const std::array<double, 9> expected_accel = {
      kAccelVar, 0.0, 0.0,
      0.0, kAccelVar, 0.0,
      0.0, 0.0, kAccelVar,
  };
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(data.orientation_covariance[i], expected_orientation[i]);
    EXPECT_DOUBLE_EQ(data.angular_velocity_covariance[i], expected_gyro[i]);
    EXPECT_DOUBLE_EQ(data.linear_acceleration_covariance[i],
                     expected_accel[i]);
  }
}

TEST(Msgs, GyroAccelPassthrough) {
  const ImuData data = make_dict();
  EXPECT_DOUBLE_EQ(data.angular_velocity[0], 0.1);
  EXPECT_DOUBLE_EQ(data.angular_velocity[1], -0.2);
  EXPECT_DOUBLE_EQ(data.angular_velocity[2], 0.3);
  EXPECT_DOUBLE_EQ(data.linear_acceleration[0], 0.0);
  EXPECT_DOUBLE_EQ(data.linear_acceleration[1], 0.0);
  EXPECT_DOUBLE_EQ(data.linear_acceleration[2], kGravity);
}

TEST(Msgs, RejectsWrongLengths) {
  try {
    build_imu_data(0.0, "imu_link", {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                   {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0);
    FAIL() << "quat 长度错误应当抛异常";
  } catch (const std::invalid_argument& exc) {
    EXPECT_NE(std::string(exc.what()).find("quat"), std::string::npos);
  }
  try {
    build_imu_data(0.0, "imu_link", {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0},
                   {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0);
    FAIL() << "gyro 长度错误应当抛异常";
  } catch (const std::invalid_argument& exc) {
    EXPECT_NE(std::string(exc.what()).find("gyro"), std::string::npos);
  }
}
