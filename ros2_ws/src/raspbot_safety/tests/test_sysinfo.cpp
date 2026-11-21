// Copyright (c) 2026 RASPBOT-V2 WP-H
// 1:1 映射 legacy Python 用例 tests/test_sysinfo.py（17 用例）。
// 纯解析函数：输入字符串，不读真实 /proc。

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "raspbot_safety/sysinfo.hpp"

using raspbot_safety::sysinfo::cpu_percent;
using raspbot_safety::sysinfo::fmt_uptime;
using raspbot_safety::sysinfo::mem_percent;
using raspbot_safety::sysinfo::parse_cpu_stat;
using raspbot_safety::sysinfo::parse_loadavg;
using raspbot_safety::sysinfo::parse_meminfo;
using raspbot_safety::sysinfo::parse_temp_mc;
using raspbot_safety::sysinfo::parse_uptime;

// ---------------------------------------------------------------- loadavg

// test_parse_loadavg_ok
TEST(SysinfoTest, ParseLoadavgOk)
{
  const auto v = parse_loadavg("0.15 0.20 0.25 1/234 5678");
  ASSERT_TRUE(v.valid);
  EXPECT_NEAR(v.load1, 0.15, 1e-12);
  EXPECT_NEAR(v.load5, 0.20, 1e-12);
  EXPECT_NEAR(v.load15, 0.25, 1e-12);
}

// test_parse_loadavg_garbage
TEST(SysinfoTest, ParseLoadavgGarbage)
{
  EXPECT_FALSE(parse_loadavg("").valid);
  EXPECT_FALSE(parse_loadavg("abc").valid);
}

// ---------------------------------------------------------------- cpu stat

constexpr char kCpuSample[] =
  "cpu  100 20 300 4000 500 10 5 0 0 0\n"
  "cpu0 50 10 150 2000 250 5 2 0 0 0\n"
  "intr 12345 0 0 0\n";

// test_parse_cpu_stat_takes_aggregate_line
TEST(SysinfoTest, ParseCpuStatTakesAggregateLine)
{
  const auto vals = parse_cpu_stat(kCpuSample);
  ASSERT_TRUE(vals.has_value());
  const std::vector<long> expected = {100, 20, 300, 4000, 500, 10, 5, 0, 0, 0};
  EXPECT_EQ(*vals, expected);
}

// test_parse_cpu_stat_empty
TEST(SysinfoTest, ParseCpuStatEmpty)
{
  EXPECT_FALSE(parse_cpu_stat("").has_value());
  EXPECT_FALSE(parse_cpu_stat("intr 1 2 3\n").has_value());
}

// test_cpu_percent_idle_only
TEST(SysinfoTest, CpuPercentIdleOnly)
{
  const std::vector<long> prev = {10, 0, 0, 100, 0, 0, 0, 0, 0, 0};
  const std::vector<long> cur = {10, 0, 0, 200, 0, 0, 0, 0, 0, 0};
  const auto v = cpu_percent(prev, cur);
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(*v, 0.0, 1e-9);
}

// test_cpu_percent_half_busy
TEST(SysinfoTest, CpuPercentHalfBusy)
{
  // 总计 +200，其中 idle +100 → 使用率 50%
  const std::vector<long> prev = {10, 0, 0, 100, 0, 0, 0, 0, 0, 0};
  const std::vector<long> cur = {110, 0, 0, 200, 0, 0, 0, 0, 0, 0};
  const auto v = cpu_percent(prev, cur);
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(*v, 50.0, 1e-9);
}

// test_cpu_percent_iowait_counts_as_idle
TEST(SysinfoTest, CpuPercentIowaitCountsAsIdle)
{
  const std::vector<long> prev = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  const std::vector<long> cur = {0, 0, 0, 0, 100, 0, 0, 0, 0, 0};
  const auto v = cpu_percent(prev, cur);
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(*v, 0.0, 1e-9);
}

// test_cpu_percent_invalid
TEST(SysinfoTest, CpuPercentInvalid)
{
  const std::vector<long> a = {1, 2, 3, 4};
  const std::vector<long> b = {1, 2, 3};
  EXPECT_FALSE(cpu_percent(a, b).has_value());          // 长度不一致
  EXPECT_FALSE(cpu_percent(a, a).has_value());          // 时间没有前进
  EXPECT_FALSE(cpu_percent({}, b).has_value());         // 空样本
}

// ---------------------------------------------------------------- meminfo

constexpr char kMemSample[] =
  "MemTotal:        8123456 kB\n"
  "MemFree:         1234567 kB\n"
  "MemAvailable:    3456789 kB\n"
  "Buffers:           12345 kB\n"
  "Cached:           234567 kB\n";

// test_parse_meminfo
TEST(SysinfoTest, ParseMeminfo)
{
  const auto mem = parse_meminfo(kMemSample);
  EXPECT_EQ(mem.at("memtotal"), 8123456L);
  EXPECT_EQ(mem.at("memavailable"), 3456789L);
}

// test_mem_percent
TEST(SysinfoTest, MemPercent)
{
  const auto mem = parse_meminfo(kMemSample);
  const auto v = mem_percent(mem);
  ASSERT_TRUE(v.has_value());
  const double expected = 100.0 * (8123456.0 - 3456789.0) / 8123456.0;
  EXPECT_NEAR(*v, expected, 1e-9);
}

// test_mem_percent_fallback_without_memavailable
TEST(SysinfoTest, MemPercentFallbackWithoutMemavailable)
{
  const std::map<std::string, long> mem = {
    {"memtotal", 1000}, {"memfree", 600}, {"buffers", 100}, {"cached", 100}};
  const auto v = mem_percent(mem);
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(*v, 20.0, 1e-9);
}

// test_mem_percent_empty
TEST(SysinfoTest, MemPercentEmpty)
{
  EXPECT_FALSE(mem_percent({}).has_value());
}

// ---------------------------------------------------------------- temp

// test_parse_temp_mc
TEST(SysinfoTest, ParseTempMc)
{
  const auto v1 = parse_temp_mc("45123\n");
  ASSERT_TRUE(v1.has_value());
  EXPECT_NEAR(*v1, 45.123, 1e-9);
  const auto v2 = parse_temp_mc("  45000  \n");
  ASSERT_TRUE(v2.has_value());
  EXPECT_NEAR(*v2, 45.0, 1e-9);
}

// test_parse_temp_mc_garbage
TEST(SysinfoTest, ParseTempMcGarbage)
{
  EXPECT_FALSE(parse_temp_mc("").has_value());
  EXPECT_FALSE(parse_temp_mc("abc").has_value());
}

// ---------------------------------------------------------------- uptime

// test_parse_uptime
TEST(SysinfoTest, ParseUptime)
{
  const auto v = parse_uptime("12345.67 54321.00\n");
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(*v, 12345.67, 1e-9);
}

// test_parse_uptime_garbage
TEST(SysinfoTest, ParseUptimeGarbage)
{
  EXPECT_FALSE(parse_uptime("").has_value());
}

// test_fmt_uptime
TEST(SysinfoTest, FmtUptime)
{
  EXPECT_EQ(fmt_uptime(0.0), "0s");
  EXPECT_EQ(fmt_uptime(65.0), "1m 5s");
  EXPECT_EQ(fmt_uptime(3725.0), "1h 2m 5s");
  EXPECT_EQ(fmt_uptime(90065.0), "1d 1h 1m 5s");
  EXPECT_EQ(fmt_uptime(std::nullopt), "unknown");
}
