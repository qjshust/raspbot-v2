// ============================================================================
// odom_relay 节点（C++ 重写版）—— /odom → /odometry/filtered 轻量转发（FR-E-05 P2）
//
// 不用 topic_tools/relay，保证零额外依赖。消息原样转发（保留 header.stamp、
// frame_id、child_frame_id 与协方差），发布节奏 30 Hz 跟随 ekf_node 输出。
// ============================================================================

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace raspbot_fusion
{

class OdomRelayNode : public rclcpp::Node
{
public:
  explicit OdomRelayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("odom_relay", options)
  {
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

    pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", qos);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        pub_->publish(*msg);  // 原样转发
      });

    RCLCPP_INFO(get_logger(), "odom_relay 启动：/odom → /odometry/filtered");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

}  // namespace raspbot_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<raspbot_fusion::OdomRelayNode> node;
  try {
    node = std::make_shared<raspbot_fusion::OdomRelayNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("odom_relay"), "启动失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  try {
    rclcpp::spin(node);
  } catch (...) {
    // 外部关闭（SIGINT / shutdown）——按优雅退出处理
  }

  rclcpp::shutdown();
  return 0;
}
