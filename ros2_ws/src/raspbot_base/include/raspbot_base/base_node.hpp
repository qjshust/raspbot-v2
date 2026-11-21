// ROS 2 base driver for the Yahboom RASPBOT-V2 mecanum chassis.
//
// Contract (CONTRACTS v1.2, identical to the legacy Python node):
//   subscribe : /cmd_vel_safe (parameter cmd_vel_topic, produced by WP-E),
//               /buzzer, /led, /servo_pan, /servo_tilt
//   publish   : /odom_raw  (30 Hz, Reliable depth 10)
//               /range     (5 Hz, SensorData)
//               /line_sensor (5 Hz, Reliable depth 10)
//               /diagnostics (1 Hz; base.launch.py remaps it to
//               /raspbot_base/diagnostics, WP-H merges into /diagnostics)
//   no TF     : odom->base_link is owned exclusively by the EKF (WP-E).
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "raspbot_base/i2c_bus.hpp"
#include "raspbot_base/kinematics.hpp"
#include "raspbot_base/odometry.hpp"
#include "raspbot_base/safety.hpp"

namespace raspbot_base {

class RaspbotBaseNode : public rclcpp::Node {
 public:
  explicit RaspbotBaseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~RaspbotBaseNode() override;

  /// Zero all four motors and switch the ultrasonic ranger off. Idempotent;
  /// called on every exit path (on_shutdown, destructor, main).
  void stop();

 private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);
  void on_buzzer(const std_msgs::msg::Bool::ConstSharedPtr& msg);
  void on_led(const std_msgs::msg::ColorRGBA::ConstSharedPtr& msg);
  void on_pan(const std_msgs::msg::Float32::ConstSharedPtr& msg);
  void on_tilt(const std_msgs::msg::Float32::ConstSharedPtr& msg);

  void control_loop();
  void sensor_loop();
  void publish_odom(const Cmd3& cmd, double now_sec);
  void publish_range(std::uint8_t hi, std::uint8_t lo);
  void publish_diagnostics();

  void note_sensor_error(double now_sec, const std::string& what);
  double now_sec() const;

  // parameters (defaults identical to config/params.yaml)
  double max_linear_ = 0.30;
  double max_angular_ = 1.50;
  double k_geom_ = 0.1245;
  double pwm_per_mps_ = 700.0;
  int max_pwm_ = 255;
  int deadband_ = 25;
  std::array<int, 4> wheel_signs_ = {1, 1, 1, 1};
  double control_rate_ = 30.0;
  double sensor_rate_ = 5.0;
  double cmd_timeout_ = 0.5;
  std::string cmd_vel_topic_ = "cmd_vel_safe";
  double cmd_resend_interval_ = 1.0;
  double diag_rate_ = 1.0;
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";

  I2cBus bus_;
  geometry_msgs::msg::Twist last_cmd_;
  CommandWatchdog watchdog_{0.5};
  CommandGate gate_;
  SensorErrorTracker errors_;
  Pose2D pose_{0.0, 0.0, 0.0};
  double last_integrate_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_line_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_buzzer_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr sub_led_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_pan_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_tilt_;
  rclcpp::TimerBase::SharedPtr timer_ctrl_;
  rclcpp::TimerBase::SharedPtr timer_sensor_;
  rclcpp::TimerBase::SharedPtr timer_diag_;
};

}  // namespace raspbot_base
