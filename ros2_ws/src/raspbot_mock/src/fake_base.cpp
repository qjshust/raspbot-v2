// Copyright (c) 2026 RASPBOT-V2 WP-H
// fake_base — mock 底盘节点（FR-H-04 / SW-I04），C++ 版。
//
// 行为与 legacy_python/raspbot_mock/fake_base.py 一致：
//   订阅 /cmd_vel_safe（Reliable, depth 10），30 Hz 发布 /odom_raw
//   （frame odom，child base_link），按指令速度做半隐式积分
//   （theta 先更新再更新 x/y，与 raspbot_base 完全一致）。
// 默认不发布 TF（odom->base_link 属 E，CONTRACTS §3）；仅独立测试时
// 经参数 publish_tf:=true 开启。

#include <chrono>
#include <algorithm>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "raspbot_mock/kinematics.hpp"

using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;

class FakeBase : public rclcpp::Node
{
public:
  FakeBase()
  : Node("fake_base")
  {
    rate_ = declare_parameter<double>("rate", 30.0);
    noise_ = declare_parameter<double>("noise", 0.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    gauss_ = std::normal_distribution<double>(0.0, std::max(noise_ > 0.0 ? noise_ : 1e-12, 1e-12));

    // 契约：/cmd_vel_safe 由 E 发布（Reliable, depth 10）；/odom_raw 同 QoS
    sub_cmd_ = create_subscription<Twist>(
      "/cmd_vel_safe", 10, std::bind(&FakeBase::on_cmd, this, _1));
    pub_odom_ = create_publisher<Odometry>("/odom_raw", 10);

    // TF 属 E，默认不发；仅独立测试时经参数开启
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&FakeBase::tick, this));
    last_tick_ = now();
    last_cmd_ = now();

    RCLCPP_INFO(
      get_logger(),
      "fake_base up: rate=%.1f Hz noise=%.4f publish_tf=%s (TF owned by E)",
      rate_, noise_, publish_tf_ ? "true" : "false");
  }

private:
  void on_cmd(const Twist::SharedPtr msg)
  {
    cmd_ = *msg;
    last_cmd_ = now();
  }

  void tick()
  {
    const rclcpp::Time t_now = now();
    const double dt = raspbot_mock::sanitize_dt((t_now - last_tick_).seconds());
    last_tick_ = t_now;

    // 与 raspbot_base 相同的超时停车语义（SW-F01）
    const double silent = (t_now - last_cmd_).seconds();
    const Twist & cmd = (silent <= cmd_timeout_) ? cmd_ : zero_cmd_;
    const double vx = cmd.linear.x;
    const double vy = cmd.linear.y;
    const double wz = cmd.angular.z;

    // 半隐式积分：先 theta，再 x/y（与 raspbot_base 一致）
    pose_ = raspbot_mock::integrate_pose(pose_, vx, vy, wz, dt);

    // 噪声只扰动发布位姿，不污染积分状态
    const double x = pose_.x + gauss_(rng_);
    const double y = pose_.y + gauss_(rng_);
    const double th = pose_.theta + gauss_(rng_);
    const raspbot_mock::Quat q = raspbot_mock::yaw_to_quat(th);

    Odometry odom;
    odom.header.stamp = t_now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.z = q.z;
    odom.pose.pose.orientation.w = q.w;
    // 开环航位推算：诚实的大协方差（与 raspbot_base 同口径）
    odom.pose.covariance[0] = 0.10;
    odom.pose.covariance[7] = 0.10;
    odom.pose.covariance[35] = 0.20;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;
    odom.twist.covariance[0] = 0.05;
    odom.twist.covariance[7] = 0.05;
    odom.twist.covariance[35] = 0.10;
    pub_odom_->publish(odom);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = odom.header.stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x;
      tf.transform.translation.y = y;
      tf.transform.rotation.z = q.z;
      tf.transform.rotation.w = q.w;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  double rate_;
  double noise_;
  double cmd_timeout_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;

  rclcpp::Subscription<Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  Twist cmd_;
  Twist zero_cmd_;
  raspbot_mock::Pose2D pose_;
  rclcpp::Time last_tick_;
  rclcpp::Time last_cmd_;

  std::mt19937 rng_{std::random_device{}()};
  std::normal_distribution<double> gauss_{0.0, 1e-12};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeBase>());
  rclcpp::shutdown();
  return 0;
}
