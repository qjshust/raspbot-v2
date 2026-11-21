#pragma once

#include <string>

namespace raspbot_imu {

/// 规范化后的关键配置（对应 Python 版 config.validate_config 返回值）。
struct ImuConfig {
  std::string mode;         // yaw | mahony
  std::string angle_source; // device | gyro
  std::string source;       // mock | witmotion | icm42688
};

/// 校验 IMU 节点关键参数；非法值抛 std::invalid_argument（信息含全部可选值）。
/// 合法时返回小写、去首尾空白的三元组。
ImuConfig validate_config(const std::string& mode,
                          const std::string& angle_source,
                          const std::string& source);

}  // namespace raspbot_imu
