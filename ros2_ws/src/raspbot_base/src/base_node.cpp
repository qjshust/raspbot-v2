#include "raspbot_base/base_node.hpp"

#include <algorithm>
#include <csignal>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "raspbot_base/sensors.hpp"

namespace raspbot_base {

namespace {

constexpr std::uint8_t kRegMotor = 0x01;   // [motor_id, dir, speed]
constexpr std::uint8_t kRegServo = 0x02;   // [servo_id, angle]
constexpr std::uint8_t kRegBuzzer = 0x06;  // [state]
constexpr std::uint8_t kRegUltrasonic = 0x07;  // [state]
constexpr std::uint8_t kRegLedAll = 0x08;  // [R, G, B]
constexpr std::uint8_t kRegLine = 0x0A;    // read 1 byte
constexpr std::uint8_t kRegDistLo = 0x1A;  // read 1 byte
constexpr std::uint8_t kRegDistHi = 0x1B;  // read 1 byte

constexpr double kRangeMax = 4.0;
constexpr double kSensorWarnInterval = 5.0;  // s: WARN at most once (SW-G08)

volatile std::sig_atomic_t g_sigterm = 0;

extern "C" void handle_sigterm(int /*sig*/) {
  g_sigterm = 1;
}

}  // namespace

RaspbotBaseNode::RaspbotBaseNode(const rclcpp::NodeOptions& options)
    : Node("raspbot_base", options), bus_("/dev/i2c-1", 0x2B) {
  this->declare_parameter<double>("max_linear", 0.30);
  this->declare_parameter<double>("max_angular", 1.50);
  this->declare_parameter<double>("k_geom", 0.1245);
  this->declare_parameter<double>("pwm_per_mps", 700.0);
  this->declare_parameter<int64_t>("max_pwm", 255);
  this->declare_parameter<int64_t>("deadband_pwm", 25);
  this->declare_parameter<std::vector<int64_t>>("wheel_signs", {1, 1, 1, 1});
  this->declare_parameter<double>("control_rate", 30.0);
  this->declare_parameter<double>("sensor_rate", 5.0);
  this->declare_parameter<double>("cmd_timeout", 0.5);
  this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_safe");
  this->declare_parameter<double>("cmd_resend_interval", 1.0);
  this->declare_parameter<double>("diag_rate", 1.0);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");

  max_linear_ = this->get_parameter("max_linear").as_double();
  max_angular_ = this->get_parameter("max_angular").as_double();
  k_geom_ = this->get_parameter("k_geom").as_double();
  pwm_per_mps_ = this->get_parameter("pwm_per_mps").as_double();
  max_pwm_ = static_cast<int>(this->get_parameter("max_pwm").as_int());
  deadband_ = static_cast<int>(this->get_parameter("deadband_pwm").as_int());
  {
    const auto signs = this->get_parameter("wheel_signs").as_integer_array();
    if (signs.size() != 4) {
      throw std::invalid_argument(
          "wheel_signs must have exactly 4 entries (L1 L2 R1 R2)");
    }
    for (std::size_t i = 0; i < 4; ++i) {
      wheel_signs_[i] = static_cast<int>(signs[i]);
    }
  }
  control_rate_ = this->get_parameter("control_rate").as_double();
  sensor_rate_ = this->get_parameter("sensor_rate").as_double();
  cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  cmd_resend_interval_ = this->get_parameter("cmd_resend_interval").as_double();
  diag_rate_ = this->get_parameter("diag_rate").as_double();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();

  watchdog_ = CommandWatchdog(cmd_timeout_);

  // ultrasonic ranger on (mirrors Ctrl_Ulatist_Switch(1) at init)
  bus_.write_block(kRegUltrasonic, {1});

  // publishers: contract topics/QoS (CONTRACTS v1.2)
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "odom_raw", rclcpp::QoS(10));
  pub_range_ = this->create_publisher<sensor_msgs::msg::Range>(
      "range", rclcpp::SensorDataQoS());
  pub_line_ = this->create_publisher<std_msgs::msg::UInt8>("line_sensor",
                                                           rclcpp::QoS(10));
  // absolute name exactly like the Python diagnostic_updater default; the
  // remapping in base.launch.py ('diagnostics' -> 'raspbot_base/diagnostics')
  // applies to it, keeping WP-H the single publisher of /diagnostics.
  pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

  // subscriptions
  sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        this->on_cmd_vel(msg);
      });
  sub_buzzer_ = this->create_subscription<std_msgs::msg::Bool>(
      "buzzer", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        this->on_buzzer(msg);
      });
  sub_led_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
      "led", rclcpp::QoS(10),
      [this](const std_msgs::msg::ColorRGBA::ConstSharedPtr msg) {
        this->on_led(msg);
      });
  sub_pan_ = this->create_subscription<std_msgs::msg::Float32>(
      "servo_pan", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32::ConstSharedPtr msg) {
        this->on_pan(msg);
      });
  sub_tilt_ = this->create_subscription<std_msgs::msg::Float32>(
      "servo_tilt", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32::ConstSharedPtr msg) {
        this->on_tilt(msg);
      });

  // control 30 Hz, sensors 5 Hz, diagnostics diag_rate Hz
  timer_ctrl_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      [this]() { this->control_loop(); });
  timer_sensor_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / sensor_rate_),
      [this]() { this->sensor_loop(); });
  timer_diag_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / diag_rate_),
      [this]() { this->publish_diagnostics(); });

  watchdog_.feed(now_sec());

  // zero the wheels on context shutdown too (belt & braces for SIGINT)
  rclcpp::on_shutdown([this]() { this->stop(); });

  RCLCPP_INFO(this->get_logger(),
              "raspbot_base up: I2C 0x2B, cmd topic=%s, k_geom=%.4f m, "
              "%.0f pwm/(m/s)",
              cmd_vel_topic_.c_str(), k_geom_, pwm_per_mps_);
}

RaspbotBaseNode::~RaspbotBaseNode() {
  try {
    stop();
  } catch (...) {
  }
}

void RaspbotBaseNode::stop() {
  const std::uint8_t zero = 0;
  for (std::uint8_t i = 0; i < 4; ++i) {
    bus_.write_block(kRegMotor, {i, zero, zero});
  }
  bus_.write_block(kRegUltrasonic, {zero});
  if (rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "stop: wheels zeroed, ultrasonic off");
  }
}

double RaspbotBaseNode::now_sec() const {
  return this->get_clock()->now().seconds();
}

// ------------------------------------------------------------------- inputs

void RaspbotBaseNode::on_cmd_vel(
    const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
  last_cmd_ = *msg;
  watchdog_.feed(now_sec());
}

void RaspbotBaseNode::on_buzzer(const std_msgs::msg::Bool::ConstSharedPtr& msg) {
  bus_.write_block(kRegBuzzer,
                   {static_cast<std::uint8_t>(msg->data ? 1 : 0)});
}

void RaspbotBaseNode::on_led(
    const std_msgs::msg::ColorRGBA::ConstSharedPtr& msg) {
  // int(clamp_unit(v) * 255): truncation, exactly like the legacy Python node
  const auto r = static_cast<std::uint8_t>(clamp_unit(msg->r) * 255.0);
  const auto g = static_cast<std::uint8_t>(clamp_unit(msg->g) * 255.0);
  const auto b = static_cast<std::uint8_t>(clamp_unit(msg->b) * 255.0);
  bus_.write_block(kRegLedAll, {r, g, b});
}

void RaspbotBaseNode::on_pan(const std_msgs::msg::Float32::ConstSharedPtr& msg) {
  bus_.write_block(
      kRegServo,
      {1, static_cast<std::uint8_t>(static_cast<int>(clamp_servo_pan(msg->data)))});
}

void RaspbotBaseNode::on_tilt(
    const std_msgs::msg::Float32::ConstSharedPtr& msg) {
  bus_.write_block(
      kRegServo,
      {2, static_cast<std::uint8_t>(static_cast<int>(clamp_servo_tilt(msg->data)))});
}

// ---------------------------------------------------------------- control

void RaspbotBaseNode::control_loop() {
  const double now = now_sec();

  // FR-A-02: command silence means stop
  geometry_msgs::msg::Twist cmd;
  if (!watchdog_.stale(now)) {
    cmd = last_cmd_;
  }

  const Cmd3 c = clamp_velocity(cmd.linear.x, cmd.linear.y, cmd.angular.z,
                                max_linear_, max_angular_);
  const auto pwms = wheel_pwm(c.vx, c.vy, k_geom_ * c.wz, pwm_per_mps_,
                              max_pwm_, deadband_, wheel_signs_);

  // FR-A-06: re-send the four wheel PWMs at least every cmd_resend_interval
  // even when the command did not change (the expansion board forgets its
  // last state after a reset).
  if (gate_.should_send(pwms, now, cmd_resend_interval_)) {
    for (std::size_t i = 0; i < pwms.size(); ++i) {
      const MotorCmd m = motor_command(pwms[i]);
      if (!bus_.write_block(kRegMotor, {static_cast<std::uint8_t>(i), m.dir,
                                        m.speed})) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "motor write failed");
      }
    }
  }

  publish_odom(c, now);
}

void RaspbotBaseNode::publish_odom(const Cmd3& cmd, double now_sec) {
  const double dt = sanitize_dt(now_sec - last_integrate_);
  last_integrate_ = now_sec;
  pose_ = integrate_pose(pose_.x, pose_.y, pose_.theta, cmd.vx, cmd.vy, cmd.wz,
                         dt);

  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = this->now();
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = pose_.x;
  odom.pose.pose.position.y = pose_.y;
  const auto q = yaw_to_quat(pose_.theta);
  odom.pose.pose.orientation.z = q[2];
  odom.pose.pose.orientation.w = q[3];
  // open loop: report honest, large covariance (identical to legacy node)
  odom.pose.covariance[0] = 0.10;
  odom.pose.covariance[7] = 0.10;
  odom.pose.covariance[35] = 0.20;
  odom.twist.twist.linear.x = cmd.vx;
  odom.twist.twist.linear.y = cmd.vy;
  odom.twist.twist.angular.z = cmd.wz;
  odom.twist.covariance[0] = 0.05;
  odom.twist.covariance[7] = 0.05;
  odom.twist.covariance[35] = 0.10;
  pub_odom_->publish(odom);
}

// --------------------------------------------------------------- sensors

void RaspbotBaseNode::note_sensor_error(double now, const std::string& what) {
  if (errors_.note_error(now, kSensorWarnInterval)) {
    RCLCPP_WARN(this->get_logger(), "sensor read failed: %s", what.c_str());
  }
}

void RaspbotBaseNode::sensor_loop() {
  const double now = now_sec();
  std::uint8_t hi = 0;
  std::uint8_t lo = 0;
  std::uint8_t line = 0;
  const bool ok_hi = bus_.read_block(kRegDistHi, &hi, 1);
  const bool ok_lo = bus_.read_block(kRegDistLo, &lo, 1);
  const bool ok_line = bus_.read_block(kRegLine, &line, 1);

  if (!ok_hi) {
    note_sensor_error(now, "reg 0x1B returned no data");
  }
  if (!ok_lo) {
    note_sensor_error(now, "reg 0x1A returned no data");
  }
  if (!ok_line) {
    note_sensor_error(now, "reg 0x0A returned no data");
  }

  if (ok_hi && ok_lo) {
    publish_range(hi, lo);
  }
  if (ok_line) {
    std_msgs::msg::UInt8 msg;
    msg.data = line;
    pub_line_->publish(msg);
  }
}

void RaspbotBaseNode::publish_range(std::uint8_t hi, std::uint8_t lo) {
  auto rng = sensor_msgs::msg::Range();
  rng.header.stamp = this->now();
  rng.header.frame_id = "ultrasonic";
  rng.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  rng.field_of_view = 0.26f;
  rng.min_range = 0.02f;
  rng.max_range = static_cast<float>(kRangeMax);
  // 0xFFFF (out of range) and 0 (no echo) both report max_range.
  rng.range = static_cast<float>(decode_range(hi, lo, kRangeMax));
  pub_range_->publish(rng);
}

// ------------------------------------------------------------ diagnostics

void RaspbotBaseNode::publish_diagnostics() {
  auto arr = diagnostic_msgs::msg::DiagnosticArray();
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  st.name = "raspbot_base: Chassis";
  st.message = "chassis ok";
  st.hardware_id = "RASPBOT-V2 chassis 0x2B";

  const auto kv = [](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    return entry;
  };
  st.values.push_back(kv("heartbeat", "1"));
  st.values.push_back(kv("i2c_error_count", std::to_string(errors_.count())));

  const double age = watchdog_.has_feed()
                         ? std::max(0.0, now_sec() - watchdog_.last_feed())
                         : 0.0;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << age;
  st.values.push_back(kv("last_cmd_age_sec", oss.str()));

  arr.status.push_back(st);
  pub_diag_->publish(arr);
}

}  // namespace raspbot_base

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<raspbot_base::RaspbotBaseNode> node;
  try {
    node = std::make_shared<raspbot_base::RaspbotBaseNode>();
  } catch (const std::exception& e) {
    std::cerr << "raspbot_base: node init failed: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // SIGTERM: rclcpp's default handler only covers SIGINT; install our own
  // async-signal-safe flag so a SIGTERM exits the spin loop, stop() zeroes
  // the wheels, and the process returns 0 (SW-F02 / SW-G05). SIGINT is
  // handled by rclcpp: the context shuts down, rclcpp::ok() turns false and
  // the same cleanup path runs.
  std::signal(SIGTERM, &raspbot_base::handle_sigterm);

  while (rclcpp::ok() && raspbot_base::g_sigterm == 0) {
    executor.spin_once(std::chrono::milliseconds(50));
  }

  node->stop();
  executor.remove_node(node);
  node.reset();

  rclcpp::shutdown();
  return 0;
}
