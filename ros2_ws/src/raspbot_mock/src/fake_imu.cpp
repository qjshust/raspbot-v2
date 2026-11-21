// Copyright (c) 2026 RASPBOT-V2 WP-H
// fake_imu — 合成 IMU 节点（FR-H-04 / SW-I04），C++ 版。
//
// 行为与 legacy_python/raspbot_mock/fake_imu.py 一致：
//   100 Hz 发布 /imu/data（frame imu_link，SensorData QoS）；
//   yaw = yaw_rate_deg_s * t（默认 0：静止指向 0）；
//   四元数仅由 yaw 生成（z = sin(yaw/2), w = cos(yaw/2)）；
//   协方差默认与 raspbot_imu 一致（yaw 0.0025、tilt 1.0、gyro 0.0004、
//   accel 0.04，设计文档 §7.2）。

#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using sensor_msgs::msg::Imu;

// -std=c++17 下 glibc 隐藏 M_PI，用标准方式定义
constexpr double kPi = 3.14159265358979323846;

class FakeImu : public rclcpp::Node
{
public:
  FakeImu()
  : Node("fake_imu")
  {
    const double rate = declare_parameter<double>("rate", 100.0);
    const double noise = declare_parameter<double>("noise", 0.0);
    const double yaw_rate_deg_s = declare_parameter<double>("yaw_rate_deg_s", 0.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    yaw_var_ = declare_parameter<double>("yaw_variance", 0.0025);
    tilt_var_ = declare_parameter<double>("tilt_variance", 1.0);
    gyro_var_ = declare_parameter<double>("gyro_variance", 0.0004);
    accel_var_ = declare_parameter<double>("accel_variance", 0.04);

    wz_ = yaw_rate_deg_s * kPi / 180.0;
    gauss_ = std::normal_distribution<double>(0.0, std::max(noise > 0.0 ? noise : 1e-12, 1e-12));

    // 契约：/imu/data 100 Hz, SensorData QoS
    pub_ = create_publisher<Imu>("/imu/data", rclcpp::SensorDataQoS());

    start_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&FakeImu::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "fake_imu up: rate=%.1f Hz yaw_rate=%.2f deg/s noise=%.4f",
      rate, yaw_rate_deg_s, noise);
  }

private:
  void tick()
  {
    const rclcpp::Time t_now = now();
    const double t = (t_now - start_).seconds();
    const double yaw = wz_ * t + gauss_(rng_);

    Imu msg;
    msg.header.stamp = t_now;
    msg.header.frame_id = frame_id_;
    msg.orientation.z = std::sin(yaw / 2.0);
    msg.orientation.w = std::cos(yaw / 2.0);
    // 协方差对角线：x/y = roll/pitch（tilt），z = yaw
    msg.orientation_covariance[0] = tilt_var_;
    msg.orientation_covariance[7] = tilt_var_;
    msg.orientation_covariance[35] = yaw_var_;
    msg.angular_velocity.z = wz_;
    msg.angular_velocity_covariance[0] = gyro_var_;
    msg.angular_velocity_covariance[7] = gyro_var_;
    msg.angular_velocity_covariance[35] = gyro_var_;
    msg.linear_acceleration_covariance[0] = accel_var_;
    msg.linear_acceleration_covariance[7] = accel_var_;
    msg.linear_acceleration_covariance[35] = accel_var_;
    pub_->publish(msg);
  }

  std::string frame_id_;
  double wz_;
  double yaw_var_;
  double tilt_var_;
  double gyro_var_;
  double accel_var_;

  rclcpp::Publisher<Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_;

  std::mt19937 rng_{std::random_device{}()};
  std::normal_distribution<double> gauss_{0.0, 1e-12};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeImu>());
  rclcpp::shutdown();
  return 0;
}
