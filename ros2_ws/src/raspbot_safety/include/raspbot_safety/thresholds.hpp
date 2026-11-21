// Copyright (c) 2026 RASPBOT-V2 WP-H
// thresholds — 安全与诊断阈值纯函数（离线可测）。
//
// 与 legacy_python/raspbot_safety/thresholds.py 口径一致：级别用整数常量
// （0=OK, 1=WARN, 2=ERROR）。safety_node 构造消息时把 level 赋给
// DiagnosticStatus.level（uint8，对应 Python 版的 bytes 修复）。

#ifndef RASPBOT_SAFETY__THRESHOLDS_HPP_
#define RASPBOT_SAFETY__THRESHOLDS_HPP_

#include <optional>
#include <string>

namespace raspbot_safety
{
namespace thresholds
{

constexpr int OK = 0;
constexpr int WARN = 1;
constexpr int ERROR = 2;

constexpr double TEMP_WARN_C = 80.0;   // 设计 §7.4：>80 °C 降频并告警
constexpr double CPU_WARN_PCT = 90.0;
constexpr double MEM_WARN_PCT = 90.0;
constexpr double RANGE_WARN_M = 0.35;  // 最近 1 s 内最小距离 <0.35 m 告警
constexpr double IMU_STALE_S = 1.0;    // 设计 §7.4：IMU 掉线判定
constexpr double ODOM_STALE_S = 0.5;
constexpr double SCAN_STALE_S = 0.5;   // 设计 §7.4：雷达掉线判定

struct LevelResult
{
  int level = OK;
  std::string message;
};

/// age_s > limit_s 视为失联（设计 §7.4 口径）。
bool is_stale(double age_s, double limit_s);

/// 传感器时效 → (level, message)。never=true 表示从未收到消息。
LevelResult stale_level(double age_s, double limit_s, long count, bool never);

/// SoC 温度 → (level, message)。nullopt 表示读取失败。
LevelResult temp_level(
  const std::optional<double> & temp_c, double warn_c = TEMP_WARN_C);

LevelResult cpu_level(
  const std::optional<double> & usage_pct, double warn_pct = CPU_WARN_PCT);

LevelResult mem_level(
  const std::optional<double> & used_pct, double warn_pct = MEM_WARN_PCT);

/// 最近窗口内最小距离 → (level, message)。nullopt 表示窗口内无数据。
LevelResult range_level(
  const std::optional<double> & min_range, double warn_m = RANGE_WARN_M);

/// 急停状态 → (level, message)。
LevelResult estop_level(bool active);

}  // namespace thresholds
}  // namespace raspbot_safety

#endif  // RASPBOT_SAFETY__THRESHOLDS_HPP_
