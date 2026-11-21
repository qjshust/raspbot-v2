// Copyright (c) 2026 RASPBOT-V2 WP-H

#include "raspbot_safety/thresholds.hpp"

#include <iomanip>
#include <sstream>

namespace raspbot_safety
{
namespace thresholds
{

namespace
{

// Python '%.Nf' 等价格式化（与 legacy 消息文本一致）
std::string fixed(double v, int precision)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(precision) << v;
  return os.str();
}

}  // namespace

bool is_stale(double age_s, double limit_s)
{
  return age_s > limit_s;
}

LevelResult stale_level(double age_s, double limit_s, long count, bool never)
{
  if (never) {
    return {WARN, "no data received yet"};
  }
  if (is_stale(age_s, limit_s)) {
    return {WARN,
      "stale: no data for " + fixed(age_s, 2) + " s (limit " +
      fixed(limit_s, 2) + " s), " + std::to_string(count) + " msgs"};
  }
  return {OK,
    "age " + fixed(age_s, 2) + " s (limit " + fixed(limit_s, 2) +
    " s), " + std::to_string(count) + " msgs"};
}

LevelResult temp_level(const std::optional<double> & temp_c, double warn_c)
{
  if (!temp_c) {
    return {WARN, "temperature unreadable"};
  }
  if (*temp_c >= warn_c) {
    return {WARN,
      "SoC " + fixed(*temp_c, 1) + " C >= " + fixed(warn_c, 1) +
      " C (throttle risk)"};
  }
  return {OK, "SoC " + fixed(*temp_c, 1) + " C"};
}

LevelResult cpu_level(const std::optional<double> & usage_pct, double warn_pct)
{
  if (!usage_pct) {
    return {WARN, "cpu usage unreadable"};
  }
  if (*usage_pct >= warn_pct) {
    return {WARN,
      "cpu " + fixed(*usage_pct, 1) + " % >= " + fixed(warn_pct, 0) + " %"};
  }
  return {OK, "cpu " + fixed(*usage_pct, 1) + " %"};
}

LevelResult mem_level(const std::optional<double> & used_pct, double warn_pct)
{
  if (!used_pct) {
    return {WARN, "memory unreadable"};
  }
  if (*used_pct >= warn_pct) {
    return {WARN,
      "mem " + fixed(*used_pct, 1) + " % >= " + fixed(warn_pct, 0) + " %"};
  }
  return {OK, "mem " + fixed(*used_pct, 1) + " %"};
}

LevelResult range_level(const std::optional<double> & min_range, double warn_m)
{
  if (!min_range) {
    return {OK, "no range data in window"};
  }
  if (*min_range < warn_m) {
    return {WARN,
      "min range " + fixed(*min_range, 3) + " m < " + fixed(warn_m, 2) + " m"};
  }
  return {OK, "min range " + fixed(*min_range, 3) + " m"};
}

LevelResult estop_level(bool active)
{
  if (active) {
    return {WARN, "ESTOP ENGAGED"};
  }
  return {OK, "released"};
}

}  // namespace thresholds
}  // namespace raspbot_safety
