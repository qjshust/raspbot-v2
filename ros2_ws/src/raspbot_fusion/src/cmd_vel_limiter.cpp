// ============================================================================
// cmd_vel_limiter 节点（C++ 重写版）—— /cmd_vel_safe 的唯一发布者（契约 CONTRACTS §2）
//
// 订阅 /cmd_vel（Reliable, depth 10）、/estop（Bool, Reliable, depth 1）、
// /imu/data（SensorData，仅取 header.stamp 算新鲜度）；
// 以 control_rate（默认 30 Hz）发布 /cmd_vel_safe（Twist, Reliable）。
//
// 与 Python 版语义 1:1：
//   - /estop 锁存（latched Bool，解除后从零平滑恢复）；
//   - IMU 数据超时（> imu_timeout）→ 线性输出钳制到 degraded_max_linear（0.15 m/s）；
//   - 输入超时（> input_timeout=0.4 s）→ 输出瞬时归零（绕过斜率限幅，
//     实测 SW-F01 判据 0.41 s）；
//   - 30 Hz 定时器 + 斜率限幅参数与 config/limiter.yaml 一致；
//   - 时间处理用 rclcpp::Time（对应原 rclpy.time.Time.from_msg 修复：
//     与节点时钟同 clock_type 构造，避免跨时钟类型相减）。
//
// 安全链路（P0，J-07/SW-F01 三级停车 ≤0.5 s）：
//   /cmd_vel 静默超过 input_timeout（0.4 s）→ 输出瞬时归零（不经斜率限幅），
//   30 Hz 持续发布零速 → 底盘 A 的 0.5 s 看门狗（cmd_timeout）作为最后兜底。
//   总链 ≈0.4 s + 两级传播各 ~33 ms ≈ 0.47 s ≤ 0.54 s 判据。
// ============================================================================

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

#include "raspbot_fusion/limiter.hpp"

namespace raspbot_fusion
{

class CmdVelLimiterNode : public rclcpp::Node
{
public:
  explicit CmdVelLimiterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("cmd_vel_limiter", options)
  {
    // ---- 参数（默认值 = config/limiter.yaml，launch 里以 yaml 传入覆盖）----
    control_rate_ = declare_parameter<double>("control_rate", 30.0);
    const double max_linear = declare_parameter<double>("max_linear", 0.30);
    const double max_angular = declare_parameter<double>("max_angular", 1.5);
    const double max_lin_accel = declare_parameter<double>("max_lin_accel", 0.6);
    const double max_ang_accel = declare_parameter<double>("max_ang_accel", 2.0);
    const double degraded_max_linear =
      declare_parameter<double>("degraded_max_linear", 0.15);
    const double imu_timeout = declare_parameter<double>("imu_timeout", 1.0);
    const double input_timeout = declare_parameter<double>("input_timeout", 0.4);
    warn_interval_ = declare_parameter<double>("warn_interval", 2.0);

    if (!(control_rate_ > 0.0) || !std::isfinite(control_rate_)) {
      // 非法参数拒绝启动（退出码非 0，SW-U15 精神）
      throw std::invalid_argument(
        "control_rate must be a positive number, got " + std::to_string(control_rate_));
    }

    limiter_ = std::make_unique<VelocityLimiter>(
      max_linear, max_angular, max_lin_accel, max_ang_accel,
      degraded_max_linear, imu_timeout, input_timeout);

    // ---- QoS（契约 CONTRACTS §2）----
    rclcpp::QoS reliable10(10);
    reliable10.reliability(rclcpp::ReliabilityPolicy::Reliable);
    rclcpp::QoS reliable1(1);
    reliable1.reliability(rclcpp::ReliabilityPolicy::Reliable);

    // ---- 订阅 / 发布 ----
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", reliable10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        target_ = Vel{msg->linear.x, msg->linear.y, msg->angular.z};
        last_cmd_t_ = SteadyClock::now();
      });
    sub_estop_ = create_subscription<std_msgs::msg::Bool>(
      "/estop", reliable1,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        const bool active = msg->data;
        estop_latched_ = active;
        limiter_->estop(active);
        if (active) {
          RCLCPP_WARN(get_logger(),
            "ESTOP 锁存：此后 /cmd_vel_safe 恒零，直到显式发布 /estop=false");
        } else {
          RCLCPP_INFO(get_logger(), "ESTOP 解除：恢复输出（从零平滑加速）");
        }
      });
    // /imu/data 仅取 header.stamp 计算新鲜度（SensorData QoS）。
    // 用 rclcpp::Time 且 clock_type 与节点时钟一致（对应原 Python 版
    // rclpy.time.Time.from_msg 修复）。
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        last_imu_stamp_ =
          rclcpp::Time(msg->header.stamp, this->now().get_clock_type());
        has_imu_ = true;
      });
    pub_safe_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe", reliable10);

    // ---- 内部状态 ----
    last_warn_t_ = SteadyClock::now();
    // 初始视为已知状态，避免开机（传感器/指令尚未到达）误告警
    was_imu_stale_ = true;
    was_input_stale_ = true;

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&CmdVelLimiterNode::step, this));

    RCLCPP_INFO(get_logger(),
      "cmd_vel_limiter 启动：%.0f Hz → /cmd_vel_safe, max_linear=%.2f m/s, "
      "max_angular=%.2f rad/s, degraded=%.2f m/s, imu_timeout=%.1f s, "
      "input_timeout=%.1f s",
      control_rate_, limiter_->max_linear, limiter_->max_angular,
      limiter_->degraded_max_linear, limiter_->imu_timeout,
      limiter_->input_timeout);
  }

  /// 任何退出路径补发零速（底盘 0.5 s 看门狗兜底之前先主动停车）。
  void publish_zero()
  {
    pub_safe_->publish(geometry_msgs::msg::Twist{});
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  void step()
  {
    const auto now_mono = SteadyClock::now();
    double dt = 0.0;
    if (last_step_t_.has_value()) {
      dt = std::chrono::duration<double>(now_mono - *last_step_t_).count();
    }
    last_step_t_ = now_mono;

    // IMU 新鲜度（从未收到 → NaN = 掉线）
    double imu_age = kNever;
    if (has_imu_) {
      imu_age = (this->now() - last_imu_stamp_).seconds();
      if (!(imu_age >= 0.0)) {
        imu_age = 0.0;  // 时钟跳变防护：未来时间戳按新鲜处理
      }
    }
    // 输入新鲜度（从未收到 → NaN = 超时）
    double input_age = kNever;
    if (last_cmd_t_.has_value()) {
      input_age = std::chrono::duration<double>(now_mono - *last_cmd_t_).count();
    }

    const Vel out = limiter_->step(
      target_.vx, target_.vy, target_.wz, dt, imu_age, input_age);

    geometry_msgs::msg::Twist msg;
    msg.linear.x = out.vx;
    msg.linear.y = out.vy;
    msg.linear.z = 0.0;
    msg.angular.z = out.wz;
    pub_safe_->publish(msg);

    warn(now_mono, imu_age, input_age);
  }

  /// 状态转换告警，限频 warn_interval 秒（要求 WARN ≤1/2 s）。
  void warn(SteadyClock::time_point now_mono, double imu_age, double input_age)
  {
    const bool stale_imu = limiter_->imu_stale(imu_age);
    const bool stale_in = limiter_->input_stale(input_age);
    const bool can_log =
      std::chrono::duration<double>(now_mono - last_warn_t_).count() >=
      warn_interval_;

    if (stale_imu && !was_imu_stale_ && can_log) {
      RCLCPP_WARN(get_logger(),
        "IMU 掉线（无数据或 age>%.1f s）：线性速度上限降至 %.2f m/s",
        limiter_->imu_timeout, limiter_->degraded_max_linear);
      last_warn_t_ = now_mono;
    }
    if (!stale_imu && was_imu_stale_ && can_log) {
      RCLCPP_INFO(get_logger(),
        "IMU 恢复：恢复满速上限 %.2f m/s", limiter_->max_linear);
      last_warn_t_ = now_mono;
    }
    if (stale_in && !was_input_stale_ && can_log) {
      RCLCPP_WARN(get_logger(),
        "输入指令超时（/cmd_vel 超过 %.1f s 未更新）：瞬时归零",
        limiter_->input_timeout);
      last_warn_t_ = now_mono;
    }
    if (!stale_in && was_input_stale_ && can_log) {
      RCLCPP_INFO(get_logger(), "输入指令恢复");
      last_warn_t_ = now_mono;
    }

    was_imu_stale_ = stale_imu;
    was_input_stale_ = stale_in;
  }

  std::unique_ptr<VelocityLimiter> limiter_;
  double control_rate_ = 30.0;
  double warn_interval_ = 2.0;

  Vel target_{0.0, 0.0, 0.0};                       // 最近一条 /cmd_vel
  std::optional<SteadyClock::time_point> last_cmd_t_;
  rclcpp::Time last_imu_stamp_{0, 0, RCL_SYSTEM_TIME};  // 占位，has_imu_ 控制有效
  bool has_imu_ = false;
  std::optional<SteadyClock::time_point> last_step_t_;
  SteadyClock::time_point last_warn_t_{};
  bool was_imu_stale_ = true;
  bool was_input_stale_ = true;
  bool estop_latched_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_safe_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace raspbot_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<raspbot_fusion::CmdVelLimiterNode> node;
  try {
    node = std::make_shared<raspbot_fusion::CmdVelLimiterNode>();
  } catch (const std::exception & e) {
    // 非法参数拒绝启动：退出码非 0（SW-U15 精神）
    RCLCPP_FATAL(rclcpp::get_logger("cmd_vel_limiter"), "启动失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  try {
    rclcpp::spin(node);
  } catch (...) {
    // 外部关闭（SIGINT / shutdown）——按优雅退出处理
  }

  try {
    node->publish_zero();  // 任何退出路径都补发零速（底盘 0.5 s 看门狗兜底）
  } catch (...) {
  }
  rclcpp::shutdown();
  return 0;
}
