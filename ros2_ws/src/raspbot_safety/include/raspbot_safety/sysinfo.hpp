// Copyright (c) 2026 RASPBOT-V2 WP-H
// sysinfo — /proc 与 /sys 文本解析纯函数（离线可测，不读真实文件）。
//
// 与 legacy_python/raspbot_safety/sysinfo.py 口径一致：函数只接受字符串
// 输入并返回纯数据；文件读取由 safety_node 完成并把原始文本传进来。

#ifndef RASPBOT_SAFETY__SYSINFO_HPP_
#define RASPBOT_SAFETY__SYSINFO_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace raspbot_safety
{
namespace sysinfo
{

struct LoadAvg
{
  bool valid = false;
  double load1 = 0.0;
  double load5 = 0.0;
  double load15 = 0.0;
};

/// 解析 /proc/loadavg → (load1, load5, load15)；失败 valid=false。
LoadAvg parse_loadavg(const std::string & text);

/// 解析 /proc/stat 的聚合 `cpu` 行 → 各计数字段列表；失败返回 nullopt。
/// 字段顺序：user nice system idle iowait irq softirq steal guest guest_nice。
std::optional<std::vector<long>> parse_cpu_stat(const std::string & text);

/// 两次 /proc/stat 采样 → CPU 使用率（%）；样本无效返回 nullopt。
std::optional<double> cpu_percent(
  const std::vector<long> & prev, const std::vector<long> & cur);

/// 解析 /proc/meminfo → {key: 值(kB)}，key 为小写（memtotal 等）。
std::map<std::string, long> parse_meminfo(const std::string & text);

/// meminfo map → 已用内存百分比；缺字段返回 nullopt。
std::optional<double> mem_percent(const std::map<std::string, long> & mem);

/// 解析 sysfs 温度文件（m°C 整数文本）→ °C 浮点；失败返回 nullopt。
std::optional<double> parse_temp_mc(const std::string & text);

/// 解析 /proc/uptime 首字段 → 秒；失败返回 nullopt。
std::optional<double> parse_uptime(const std::string & text);

/// 秒 → '1d 2h 3m 4s' 风格文本；nullopt → 'unknown'。
std::string fmt_uptime(const std::optional<double> & seconds);

}  // namespace sysinfo
}  // namespace raspbot_safety

#endif  // RASPBOT_SAFETY__SYSINFO_HPP_
