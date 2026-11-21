#pragma once

#include <array>
#include <string>
#include <vector>

namespace raspbot_imu {

/// 一帧组装好的 IMU 数据（对应 Python 版 msgs.build_imu_dict 的返回值）。
struct ImuData {
  double stamp_sec = 0.0;
  std::string frame_id;
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 0.0};  // w, x, y, z
  std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};  // rad/s
  std::array<double, 3> linear_acceleration{0.0, 0.0, 0.0};  // m/s^2
  // 三个 3x3 行主序对角协方差。
  std::array<double, 9> orientation_covariance{0.0};
  std::array<double, 9> angular_velocity_covariance{0.0};
  std::array<double, 9> linear_acceleration_covariance{0.0};
};

/// 组装一帧 IMU 数据（与 ROS 无关，可离线单测）。
///
/// 长度不对抛 std::invalid_argument（信息含字段名）。
ImuData build_imu_data(double now_sec, const std::string& frame_id,
                       const std::vector<double>& quat,
                       const std::vector<double>& gyro,
                       const std::vector<double>& accel,
                       double yaw_var, double tilt_var,
                       double gyro_var, double accel_var);

}  // namespace raspbot_imu
