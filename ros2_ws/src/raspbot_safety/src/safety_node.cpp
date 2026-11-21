// Copyright (c) 2026 RASPBOT-V2 WP-H
// safety_node — 平台诊断聚合与安全监控（FR-H-03 / SYS-S05 / SYS-S04 可见性），
// C++ 版。行为与 legacy_python/raspbot_safety/safety_node.py 一致。
//
// 1 Hz 发布 /diagnostics（Reliable）：
//   1. 聚合：订阅各节点 remap 后的诊断话题
//      raspbot_base/diagnostics、twist_mux/diagnostics、
//      lidar_filter/diagnostics、ekf/diagnostics，
//      把子 status 合并进 /diagnostics（名称加话题前缀）。
//   2. 自身状态：heartbeat、CPU 负载（getloadavg）、CPU 使用率（/proc/stat
//      两次采样差值）、内存（/proc/meminfo）、SoC 温度（thermal_zone0）、
//      uptime。
//   3. 安全：/estop 状态；/range 最近 1 s 最小值 <0.35 m → WARN；
//      /imu/data(>1 s)、/odom_raw(>0.5 s)、/scan(>0.5 s) 失联 → WARN。
// DiagnosticStatus.level 为 uint8（对应 Python 版的 bytes 修复，C++ 原生）。
// 解析与阈值判断是纯函数（sysinfo / thresholds），离线可测。

#include <chrono>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"

#include "raspbot_safety/sysinfo.hpp"
#include "raspbot_safety/thresholds.hpp"

using namespace raspbot_safety;  // thresholds:: / sysinfo:: 纯函数命名空间

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::Range;
using std_msgs::msg::Bool;
using std::placeholders::_1;

namespace
{

constexpr char kTempPath[] = "/sys/class/thermal/thermal_zone0/temp";
constexpr char kProcStat[] = "/proc/stat";
constexpr char kProcMeminfo[] = "/proc/meminfo";
constexpr char kProcUptime[] = "/proc/uptime";

// Python '%.Nf' 等价格式化（与 legacy 消息文本一致）
std::string fixed(double v, int precision)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(precision) << v;
  return os.str();
}

std::optional<std::string> read_file(const char * path)
{
  std::ifstream in(path);
  if (!in) {
    return std::nullopt;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

}  // namespace

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode()
  : Node("safety_node")
  {
    rate_ = declare_parameter<double>("rate", 1.0);
    temp_warn_c_ = declare_parameter<double>("temp_warn_c", thresholds::TEMP_WARN_C);
    cpu_warn_pct_ = declare_parameter<double>("cpu_warn_pct", thresholds::CPU_WARN_PCT);
    mem_warn_pct_ = declare_parameter<double>("mem_warn_pct", thresholds::MEM_WARN_PCT);
    range_warn_m_ = declare_parameter<double>("range_warn_m", thresholds::RANGE_WARN_M);
    range_window_s_ = declare_parameter<double>("range_window_s", 1.0);
    imu_stale_s_ = declare_parameter<double>("imu_stale_s", thresholds::IMU_STALE_S);
    odom_stale_s_ = declare_parameter<double>("odom_stale_s", thresholds::ODOM_STALE_S);
    scan_stale_s_ = declare_parameter<double>("scan_stale_s", thresholds::SCAN_STALE_S);

    // 契约：/diagnostics 1 Hz, Reliable（唯一发布者）
    pub_diag_ = create_publisher<DiagnosticArray>("/diagnostics", 10);

    // 安全相关订阅（契约 QoS：/estop Reliable depth 1；传感器 SensorData）
    sub_estop_ = create_subscription<Bool>(
      "/estop", rclcpp::QoS(1).reliable(),
      std::bind(&SafetyNode::on_estop, this, _1));
    sub_range_ = create_subscription<Range>(
      "/range", rclcpp::SensorDataQoS(),
      std::bind(&SafetyNode::on_range, this, _1));
    sub_imu_ = create_subscription<Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&SafetyNode::on_imu, this, _1));
    sub_odom_ = create_subscription<Odometry>(
      "/odom_raw", 10, std::bind(&SafetyNode::on_odom, this, _1));
    sub_scan_ = create_subscription<LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&SafetyNode::on_scan, this, _1));

    // 聚合订阅：各节点 remap 后的诊断话题（标签带前导斜杠，与 Python 版
    // 动态发现时的绝对话题名一致，用于 /diagnostics 内条目名前缀）
    diag_sources_ = {
      {"/ekf/diagnostics", "ekf/diagnostics"},
      {"/lidar_filter/diagnostics", "lidar_filter/diagnostics"},
      {"/raspbot_base/diagnostics", "raspbot_base/diagnostics"},
      {"/twist_mux/diagnostics", "twist_mux/diagnostics"},
    };
    for (size_t i = 0; i < diag_sources_.size(); ++i) {
      diag_sources_[i].sub = create_subscription<DiagnosticArray>(
        diag_sources_[i].topic, 10,
        [this, i](const DiagnosticArray::SharedPtr msg) { on_diag(i, msg); });
    }

    // 传感器时效状态
    seen_["/imu/data"] = {0, rclcpp::Time()};
    seen_["/odom_raw"] = {0, rclcpp::Time()};
    seen_["/scan"] = {0, rclcpp::Time()};

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&SafetyNode::tick, this));

    RCLCPP_INFO(get_logger(), "safety_node up: rate=%.1f Hz, publishing /diagnostics", rate_);
  }

private:
  // ---------------------------------------------------------------- 订阅回调

  void on_estop(const Bool::SharedPtr msg)
  {
    estop_ = msg->data;
  }

  void on_range(const Range::SharedPtr msg)
  {
    const rclcpp::Time t_now = now();
    range_samples_.emplace_back(t_now, msg->range);
  }

  void mark(const std::string & topic)
  {
    seen_[topic].count += 1;
    seen_[topic].last = now();
  }

  void on_imu(const Imu::SharedPtr /*msg*/)
  {
    mark("/imu/data");
  }

  void on_odom(const Odometry::SharedPtr /*msg*/)
  {
    mark("/odom_raw");
  }

  void on_scan(const LaserScan::SharedPtr /*msg*/)
  {
    mark("/scan");
  }

  void on_diag(size_t index, const DiagnosticArray::SharedPtr msg)
  {
    diag_sources_[index].latest = *msg;
    diag_sources_[index].received = true;
  }

  // ---------------------------------------------------------------- 状态构造

  DiagnosticStatus make_status(
    const std::string & name, int level, const std::string & message,
    const std::string & hardware_id,
    const std::vector<std::pair<std::string, std::string>> & values = {})
  {
    DiagnosticStatus status;
    status.name = name;
    status.level = static_cast<uint8_t>(level);  // uint8，对应 Python bytes 修复
    status.message = message;
    status.hardware_id = hardware_id;
    for (const auto & kv : values) {
      KeyValue item;
      item.key = kv.first;
      item.value = kv.second;
      status.values.push_back(item);
    }
    return status;
  }

  DiagnosticStatus heartbeat_status()
  {
    ++heartbeat_;
    return make_status(
      "Raspbot Platform: Heartbeat", thresholds::OK,
      "heartbeat " + std::to_string(heartbeat_), "safety_node",
      {{"heartbeat", std::to_string(heartbeat_)}});
  }

  DiagnosticStatus cpu_status()
  {
    std::optional<double> usage;
    const auto text = read_file(kProcStat);
    const auto cur = text ? raspbot_safety::sysinfo::parse_cpu_stat(*text)
                          : std::nullopt;
    if (cur && prev_cpu_) {
      usage = raspbot_safety::sysinfo::cpu_percent(*prev_cpu_, *cur);
    }
    if (cur) {
      prev_cpu_ = cur;
    }

    double load[3] = {0.0, 0.0, 0.0};
    getloadavg(load, 3);

    const auto lv = thresholds::cpu_level(usage, cpu_warn_pct_);
    return make_status(
      "Raspbot Platform: CPU", lv.level, lv.message, "cpu",
      {{"usage_pct", usage ? fixed(*usage, 1) : "n/a"},
       {"load1", fixed(load[0], 2)},
       {"load5", fixed(load[1], 2)},
       {"load15", fixed(load[2], 2)}});
  }

  DiagnosticStatus mem_status()
  {
    const auto text = read_file(kProcMeminfo);
    const auto mem = text ? raspbot_safety::sysinfo::parse_meminfo(*text)
                          : std::map<std::string, long>{};
    const auto used = raspbot_safety::sysinfo::mem_percent(mem);
    const auto lv = thresholds::mem_level(used, mem_warn_pct_);
    const auto total_it = mem.find("memtotal");
    const auto avail_it = mem.find("memavailable");
    return make_status(
      "Raspbot Platform: Memory", lv.level, lv.message, "mem",
      {{"used_pct", used ? fixed(*used, 1) : "n/a"},
       {"memtotal_kb", total_it != mem.end() ? std::to_string(total_it->second) : "n/a"},
       {"memavailable_kb", avail_it != mem.end() ? std::to_string(avail_it->second) : "n/a"}});
  }

  DiagnosticStatus temp_status()
  {
    const auto text = read_file(kTempPath);
    const auto temp = text ? raspbot_safety::sysinfo::parse_temp_mc(*text)
                           : std::nullopt;
    const auto lv = thresholds::temp_level(temp, temp_warn_c_);
    return make_status(
      "Raspbot Platform: Temperature", lv.level, lv.message, "soc",
      {{"temp_c", temp ? fixed(*temp, 1) : "n/a"}});
  }

  DiagnosticStatus uptime_status()
  {
    const auto text = read_file(kProcUptime);
    const auto up = text ? raspbot_safety::sysinfo::parse_uptime(*text)
                         : std::nullopt;
    return make_status(
      "Raspbot Platform: Uptime", thresholds::OK,
      raspbot_safety::sysinfo::fmt_uptime(up), "system",
      {{"uptime_s", up ? fixed(*up, 2) : "n/a"}});
  }

  DiagnosticStatus estop_status()
  {
    const auto lv = thresholds::estop_level(estop_);
    return make_status(
      "Raspbot Safety: E-Stop", lv.level, lv.message, "estop",
      {{"estop", estop_ ? "True" : "False"}});
  }

  DiagnosticStatus range_status()
  {
    // 只保留最近 range_window_s 内的样本
    const rclcpp::Time cutoff = now() -
      rclcpp::Duration::from_seconds(range_window_s_);
    while (!range_samples_.empty() && range_samples_.front().first < cutoff) {
      range_samples_.pop_front();
    }
    std::optional<double> min_range;
    for (const auto & sample : range_samples_) {
      if (!min_range || sample.second < *min_range) {
        min_range = sample.second;
      }
    }
    const auto lv = thresholds::range_level(min_range, range_warn_m_);
    return make_status(
      "Raspbot Safety: Proximity", lv.level, lv.message, "ultrasonic",
      {{"min_range_m", min_range ? fixed(*min_range, 3) : "n/a"},
       {"window_s", fixed(range_window_s_, 1)}});
  }

  DiagnosticStatus sensor_status(
    const std::string & name, const std::string & hardware,
    const std::string & topic, double limit_s)
  {
    const SensorSeen & seen = seen_[topic];
    const auto lv = (seen.count == 0) ?
      thresholds::stale_level(0.0, limit_s, 0, true) :
      thresholds::stale_level((now() - seen.last).seconds(), limit_s,
        seen.count, false);
    return make_status(
      name, lv.level, lv.message, hardware,
      {{"topic", topic},
       {"stale_s", fixed(limit_s, 1)},
       {"msg_count", std::to_string(seen.count)}});
  }

  // ------------------------------------------------------------------- tick

  void tick()
  {
    DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status = {
      heartbeat_status(),
      cpu_status(),
      mem_status(),
      temp_status(),
      uptime_status(),
      estop_status(),
      range_status(),
      sensor_status("Raspbot Sensors: imu/data", "imu",
        "/imu/data", imu_stale_s_),
      sensor_status("Raspbot Sensors: odom_raw", "base",
        "/odom_raw", odom_stale_s_),
      sensor_status("Raspbot Sensors: scan", "lidar",
        "/scan", scan_stale_s_),
    };
    // 聚合子诊断：字段原样透传（level 已是 uint8），名称加话题前缀
    for (const auto & source : diag_sources_) {
      if (!source.received) {
        continue;
      }
      for (const auto & sub : source.latest.status) {
        DiagnosticStatus status;
        status.name = source.label + ": " + sub.name;
        status.level = sub.level;
        status.message = sub.message;
        status.hardware_id = sub.hardware_id;
        status.values = sub.values;
        arr.status.push_back(status);
      }
    }
    pub_diag_->publish(arr);
  }

  struct SensorSeen
  {
    long count = 0;
    rclcpp::Time last;
  };

  struct DiagSource
  {
    std::string label;    // 聚合条目名前缀（绝对话题名）
    std::string topic;    // 订阅名（相对）
    rclcpp::Subscription<DiagnosticArray>::SharedPtr sub;
    DiagnosticArray latest;
    bool received = false;
  };

  // 参数
  double rate_;
  double temp_warn_c_;
  double cpu_warn_pct_;
  double mem_warn_pct_;
  double range_warn_m_;
  double range_window_s_;
  double imu_stale_s_;
  double odom_stale_s_;
  double scan_stale_s_;

  // 发布/订阅
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Subscription<Bool>::SharedPtr sub_estop_;
  rclcpp::Subscription<Range>::SharedPtr sub_range_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 状态
  uint64_t heartbeat_ = 0;
  bool estop_ = false;
  std::deque<std::pair<rclcpp::Time, double>> range_samples_;
  std::map<std::string, SensorSeen> seen_;
  std::optional<std::vector<long>> prev_cpu_;
  std::vector<DiagSource> diag_sources_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
