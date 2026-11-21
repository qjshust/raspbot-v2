/// 参数校验测试（1:1 映射 Python 版 test/test_config.py，8 用例）。
#include <gtest/gtest.h>

#include <string>

#include "raspbot_imu/config.hpp"

using namespace raspbot_imu;

TEST(Config, RejectsUnknownModeWithChoices) {
  try {
    validate_config("foo", "device", "witmotion");
    FAIL() << "应当抛出 std::invalid_argument";
  } catch (const std::invalid_argument& exc) {
    const std::string message = exc.what();
    EXPECT_NE(message.find("yaw"), std::string::npos);
    EXPECT_NE(message.find("mahony"), std::string::npos);
  }
}

TEST(Config, RejectsUnknownAngleSourceWithChoices) {
  try {
    validate_config("yaw", "bad", "witmotion");
    FAIL() << "应当抛出 std::invalid_argument";
  } catch (const std::invalid_argument& exc) {
    const std::string message = exc.what();
    EXPECT_NE(message.find("device"), std::string::npos);
    EXPECT_NE(message.find("gyro"), std::string::npos);
  }
}

TEST(Config, RejectsUnknownSourceWithChoices) {
  try {
    validate_config("yaw", "device", "unknown");
    FAIL() << "应当抛出 std::invalid_argument";
  } catch (const std::invalid_argument& exc) {
    const std::string message = exc.what();
    EXPECT_NE(message.find("mock"), std::string::npos);
    EXPECT_NE(message.find("witmotion"), std::string::npos);
    EXPECT_NE(message.find("icm42688"), std::string::npos);
  }
}

TEST(Config, ValidYawDeviceWitmotion) {
  const ImuConfig cfg = validate_config("yaw", "device", "witmotion");
  EXPECT_EQ(cfg.mode, "yaw");
  EXPECT_EQ(cfg.angle_source, "device");
  EXPECT_EQ(cfg.source, "witmotion");
}

TEST(Config, ValidYawGyroIcm42688) {
  const ImuConfig cfg = validate_config("yaw", "gyro", "icm42688");
  EXPECT_EQ(cfg.mode, "yaw");
  EXPECT_EQ(cfg.angle_source, "gyro");
  EXPECT_EQ(cfg.source, "icm42688");
}

TEST(Config, ValidMahonyGyroMock) {
  const ImuConfig cfg = validate_config("mahony", "gyro", "mock");
  EXPECT_EQ(cfg.mode, "mahony");
  EXPECT_EQ(cfg.angle_source, "gyro");
  EXPECT_EQ(cfg.source, "mock");
}

TEST(Config, NormalizesCase) {
  const ImuConfig cfg = validate_config("YAW", "Device", "WITMOTION");
  EXPECT_EQ(cfg.mode, "yaw");
  EXPECT_EQ(cfg.angle_source, "device");
  EXPECT_EQ(cfg.source, "witmotion");
}

TEST(Config, NormalizesWhitespace) {
  const ImuConfig cfg = validate_config(" mahony ", "gyro", "mock");
  EXPECT_EQ(cfg.mode, "mahony");
  EXPECT_EQ(cfg.angle_source, "gyro");
  EXPECT_EQ(cfg.source, "mock");
}
