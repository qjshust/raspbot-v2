// Copyright (c) 2026 RASPBOT-V2 WP-H
// fake_scan — 合成 LaserScan 节点（FR-H-04 / SW-I04），C++ 版。
//
// 行为与 legacy_python/raspbot_mock/fake_scan.py 一致：
//   10 Hz 发布 /scan（frame laser，SensorData QoS）；车位于 (0,0)，
//   5x4 m 矩形房间（半宽 2.5 x 2.0），360° 720 点（0.5°），射线-矩形
//   求交（slab 法，见 raspbot_mock::geometry）；未命中按 range_max 报告；
//   命中值加 noise 高斯扰动并夹到 [range_min, range_max]。
// 注：0.05 m 栅格下 5x4 m 房间 = 100x80 格地图（07_nav2_sil 依赖）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "raspbot_mock/geometry.hpp"

using sensor_msgs::msg::LaserScan;

// -std=c++17 下 glibc 隐藏 M_PI，用标准方式定义
constexpr double kPi = 3.14159265358979323846;

class FakeScan : public rclcpp::Node
{
public:
  FakeScan()
  : Node("fake_scan")
  {
    rate_ = declare_parameter<double>("rate", 10.0);
    half_x_ = declare_parameter<double>("room_x", 5.0) / 2.0;
    half_y_ = declare_parameter<double>("room_y", 4.0) / 2.0;
    noise_ = declare_parameter<double>("noise", 0.002);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser");
    range_min_ = declare_parameter<double>("range_min", 0.2);
    range_max_ = declare_parameter<double>("range_max", 15.0);
    const int num_points = declare_parameter<int>("num_points", 720);

    gauss_ = std::normal_distribution<double>(0.0, std::max(noise_ > 0.0 ? noise_ : 1e-12, 1e-12));

    // 360° / 0.5° = 720 点
    const double increment = 2.0 * kPi / static_cast<double>(num_points);
    angles_.reserve(num_points);
    for (int i = 0; i < num_points; ++i) {
      angles_.push_back(-kPi + i * increment);
    }

    // 契约：/scan 10 Hz, SensorData QoS, frame_id=laser
    pub_ = create_publisher<LaserScan>("/scan", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&FakeScan::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "fake_scan up: rate=%.1f Hz room=%.1fx%.1f m %zu points noise=%.4f",
      rate_, half_x_ * 2.0, half_y_ * 2.0, angles_.size(), noise_);
  }

private:
  void tick()
  {
    LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.angle_min = angles_.front();
    msg.angle_max = angles_.back();
    msg.angle_increment = angles_.size() > 1 ?
      angles_[1] - angles_[0] : 0.0f;
    msg.time_increment = 0.0f;
    msg.scan_time = static_cast<float>(1.0 / rate_);
    msg.range_min = static_cast<float>(range_min_);
    msg.range_max = static_cast<float>(range_max_);

    msg.ranges.reserve(angles_.size());
    for (const double angle : angles_) {
      const auto dist = raspbot_mock::ray_rect_distance(angle, half_x_, half_y_);
      if (!dist) {
        msg.ranges.push_back(static_cast<float>(range_max_));  // 未命中
        continue;
      }
      const double value = *dist + gauss_(rng_);
      msg.ranges.push_back(static_cast<float>(
          std::clamp(value, range_min_, range_max_)));
    }
    pub_->publish(msg);
  }

  double rate_;
  double half_x_;
  double half_y_;
  double noise_;
  std::string frame_id_;
  double range_min_;
  double range_max_;
  std::vector<double> angles_;

  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mt19937 rng_{std::random_device{}()};
  std::normal_distribution<double> gauss_{0.0, 1e-12};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeScan>());
  rclcpp::shutdown();
  return 0;
}
