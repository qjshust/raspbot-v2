// Copyright (c) 2026 RASPBOT-V2 WP-H

#include "raspbot_safety/sysinfo.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace raspbot_safety
{
namespace sysinfo
{

namespace
{

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

}  // namespace

LoadAvg parse_loadavg(const std::string & text)
{
  LoadAvg out;
  std::istringstream iss(text);
  double a = 0.0, b = 0.0, c = 0.0;
  if (iss >> a >> b >> c) {
    out.valid = true;
    out.load1 = a;
    out.load5 = b;
    out.load15 = c;
  }
  return out;
}

std::optional<std::vector<long>> parse_cpu_stat(const std::string & text)
{
  if (text.empty()) {
    return std::nullopt;
  }
  std::istringstream lines(text);
  std::string line;
  while (std::getline(lines, line)) {
    std::istringstream parts(line);
    std::string token;
    if (!(parts >> token) || token != "cpu") {
      continue;  // 只取聚合 `cpu` 行（cpu0 等跳过）
    }
    std::vector<long> values;
    while (parts >> token) {
      try {
        size_t pos = 0;
        values.push_back(std::stol(token, &pos));
        if (pos != token.size()) {
          return std::nullopt;
        }
      } catch (const std::exception &) {
        return std::nullopt;
      }
    }
    return values;
  }
  return std::nullopt;
}

std::optional<double> cpu_percent(
  const std::vector<long> & prev, const std::vector<long> & cur)
{
  if (prev.empty() || cur.empty() || prev.size() != cur.size()) {
    return std::nullopt;
  }
  long total_prev = 0;
  long total_cur = 0;
  for (const long v : prev) {total_prev += v;}
  for (const long v : cur) {total_cur += v;}
  const long delta_total = total_cur - total_prev;
  if (delta_total <= 0) {
    return std::nullopt;
  }
  // idle(3) + iowait(4) 视为空闲
  const long idle_prev = prev[3] + (prev.size() > 4 ? prev[4] : 0L);
  const long idle_cur = cur[3] + (cur.size() > 4 ? cur[4] : 0L);
  const long delta_idle = idle_cur - idle_prev;
  return 100.0 * std::max(0.0, static_cast<double>(delta_total - delta_idle)) /
         static_cast<double>(delta_total);
}

std::map<std::string, long> parse_meminfo(const std::string & text)
{
  std::map<std::string, long> out;
  std::istringstream lines(text);
  std::string line;
  while (std::getline(lines, line)) {
    std::istringstream parts(line);
    std::string key;
    std::string value;
    if (!(parts >> key) || !(parts >> value)) {
      continue;
    }
    if (!key.empty() && key.back() == ':') {
      key.pop_back();
    }
    try {
      out[lower(key)] = std::stol(value);
    } catch (const std::exception &) {
      continue;
    }
  }
  return out;
}

std::optional<double> mem_percent(const std::map<std::string, long> & mem)
{
  const auto total_it = mem.find("memtotal");
  if (total_it == mem.end() || total_it->second == 0) {
    return std::nullopt;
  }
  const long total = total_it->second;
  long available = 0;
  const auto avail_it = mem.find("memavailable");
  if (avail_it != mem.end()) {
    available = avail_it->second;
  } else {
    // 老内核回退：free + buffers + cached
    const auto free_it = mem.find("memfree");
    const auto buf_it = mem.find("buffers");
    const auto cache_it = mem.find("cached");
    available = (free_it != mem.end() ? free_it->second : 0L) +
                (buf_it != mem.end() ? buf_it->second : 0L) +
                (cache_it != mem.end() ? cache_it->second : 0L);
  }
  return 100.0 * std::max(0.0, static_cast<double>(total - available)) /
         static_cast<double>(total);
}

std::optional<double> parse_temp_mc(const std::string & text)
{
  std::string s = text;
  // 去除首尾空白
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return std::nullopt;
  }
  const auto last = s.find_last_not_of(" \t\r\n");
  s = s.substr(first, last - first + 1);
  try {
    size_t pos = 0;
    const double v = std::stod(s, &pos);
    if (pos != s.size()) {
      return std::nullopt;
    }
    return v / 1000.0;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<double> parse_uptime(const std::string & text)
{
  std::istringstream iss(text);
  double v = 0.0;
  if (iss >> v) {
    return v;
  }
  return std::nullopt;
}

std::string fmt_uptime(const std::optional<double> & seconds)
{
  if (!seconds) {
    return "unknown";
  }
  long s = static_cast<long>(*seconds);
  const long days = s / 86400;
  s %= 86400;
  const long hours = s / 3600;
  s %= 3600;
  const long minutes = s / 60;
  s %= 60;

  std::ostringstream os;
  if (days) {
    os << days << "d ";
  }
  if (hours) {
    os << hours << "h ";
  }
  if (minutes) {
    os << minutes << "m ";
  }
  os << s << "s";
  return os.str();
}

}  // namespace sysinfo
}  // namespace raspbot_safety
