// Copyright (c) 2026 RASPBOT-V2 WP-H
// 1:1 映射 legacy Python 用例 tests/test_thresholds.py（8 用例）。

#include <gtest/gtest.h>

#include <optional>

#include "raspbot_safety/thresholds.hpp"

using raspbot_safety::thresholds::OK;
using raspbot_safety::thresholds::WARN;
using raspbot_safety::thresholds::cpu_level;
using raspbot_safety::thresholds::estop_level;
using raspbot_safety::thresholds::is_stale;
using raspbot_safety::thresholds::mem_level;
using raspbot_safety::thresholds::range_level;
using raspbot_safety::thresholds::stale_level;
using raspbot_safety::thresholds::temp_level;

// ---------------------------------------------------------------- stale

// test_is_stale_boundary
TEST(ThresholdsTest, IsStaleBoundary)
{
  EXPECT_FALSE(is_stale(1.0, 1.0));    // 恰好等于阈值：不判失联
  EXPECT_TRUE(is_stale(1.001, 1.0));
  EXPECT_FALSE(is_stale(0.499, 0.5));
  EXPECT_TRUE(is_stale(0.5 + 1e-9, 0.5));
}

// test_stale_level_never_received
TEST(ThresholdsTest, StaleLevelNeverReceived)
{
  const auto r = stale_level(0.0, 1.0, 0, true);
  EXPECT_EQ(r.level, WARN);
  EXPECT_NE(r.message.find("no data"), std::string::npos);
}

// test_stale_level_fresh_and_stale
TEST(ThresholdsTest, StaleLevelFreshAndStale)
{
  EXPECT_EQ(stale_level(0.4, 1.0, 42, false).level, OK);
  EXPECT_EQ(stale_level(1.2, 1.0, 42, false).level, WARN);
}

// ---------------------------------------------------------------- temp

// test_temp_level
TEST(ThresholdsTest, TempLevel)
{
  const auto ok = temp_level(45.1, 80.0);
  EXPECT_EQ(ok.level, OK);
  EXPECT_EQ(ok.message, "SoC 45.1 C");

  const auto warn = temp_level(80.0, 80.0);   // 设计 §7.4：>80 告警
  EXPECT_EQ(warn.level, WARN);
  EXPECT_NE(warn.message.find("80.0"), std::string::npos);

  EXPECT_EQ(temp_level(85.0, 80.0).level, WARN);
  EXPECT_EQ(temp_level(std::nullopt).level, WARN);
}

// ---------------------------------------------------------------- cpu/mem

// test_cpu_level
TEST(ThresholdsTest, CpuLevel)
{
  EXPECT_EQ(cpu_level(42.0, 90.0).level, OK);
  EXPECT_EQ(cpu_level(90.0, 90.0).level, WARN);
  EXPECT_EQ(cpu_level(std::nullopt).level, WARN);
}

// test_mem_level
TEST(ThresholdsTest, MemLevel)
{
  EXPECT_EQ(mem_level(42.0, 90.0).level, OK);
  EXPECT_EQ(mem_level(95.0, 90.0).level, WARN);
  EXPECT_EQ(mem_level(std::nullopt).level, WARN);
}

// ---------------------------------------------------------------- range

// test_range_level
TEST(ThresholdsTest, RangeLevel)
{
  EXPECT_EQ(range_level(std::nullopt).level, OK);          // 无数据不告警（mock 模式）
  EXPECT_EQ(range_level(0.349, 0.35).level, WARN);          // <0.35 m
  EXPECT_EQ(range_level(0.35, 0.35).level, OK);             // 边界
  EXPECT_EQ(range_level(1.5, 0.35).level, OK);
}

// ---------------------------------------------------------------- estop

// test_estop_level
TEST(ThresholdsTest, EstopLevel)
{
  EXPECT_EQ(estop_level(false).level, OK);
  const auto r = estop_level(true);
  EXPECT_EQ(r.level, WARN);
  EXPECT_NE(r.message.find("ESTOP"), std::string::npos);
}
