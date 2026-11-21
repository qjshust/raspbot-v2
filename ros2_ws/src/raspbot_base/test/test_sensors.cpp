// 1:1 mapping of legacy test/test_sensors.py (SW-U08, SW-U09, 6 cases).
#include <gtest/gtest.h>

#include "raspbot_base/sensors.hpp"

using raspbot_base::decode_line;
using raspbot_base::decode_range;

namespace {

void expect_line(const std::array<int, 4>& actual, int e0, int e1, int e2,
                 int e3) {
  EXPECT_EQ(actual[0], e0);
  EXPECT_EQ(actual[1], e1);
  EXPECT_EQ(actual[2], e2);
  EXPECT_EQ(actual[3], e3);
}

// ------------------------------------------------------------- SW-U08

TEST(SwU08, BasicConversion) {
  // 0x0258 = 600 mm -> 0.6 m
  EXPECT_DOUBLE_EQ(decode_range(0x02, 0x58), 0.6);
  EXPECT_DOUBLE_EQ(decode_range(0x00, 0x64), 0.1);   // 100 mm
  EXPECT_DOUBLE_EQ(decode_range(0x03, 0xE8), 1.0);   // 1000 mm
}

TEST(SwU08, OutOfRangeMapsToMaxRange) {
  EXPECT_DOUBLE_EQ(decode_range(0xFF, 0xFF), 4.0);  // 0xFFFF: over range
  EXPECT_DOUBLE_EQ(decode_range(0x00, 0x00), 4.0);  // 0: no echo
  EXPECT_DOUBLE_EQ(decode_range(0x0F, 0xA0), 4.0);  // 4000 mm > 4.0 m
}

TEST(SwU08, NeverNegative) {
  for (const int hi : {0x00, 0xFF, 0x02, -1}) {
    for (const int lo : {0x00, 0xFF, 0x58, -1}) {
      EXPECT_GE(decode_range(hi, lo), 0.0) << "hi=" << hi << " lo=" << lo;
    }
  }
}

TEST(SwU08, CustomMaxRange) {
  EXPECT_DOUBLE_EQ(decode_range(0x02, 0x58, 2.0), 0.6);
  EXPECT_DOUBLE_EQ(decode_range(0xFF, 0xFF, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(decode_range(0x00, 0x00, 2.0), 2.0);
}

// ------------------------------------------------------------- SW-U09

TEST(SwU09, KnownBytes) {
  expect_line(decode_line(0x0F), 1, 1, 1, 1);
  expect_line(decode_line(0x0A), 1, 0, 1, 0);
  expect_line(decode_line(0x05), 0, 1, 0, 1);
  expect_line(decode_line(0x08), 1, 0, 0, 0);
  expect_line(decode_line(0x04), 0, 1, 0, 0);
  expect_line(decode_line(0x02), 0, 0, 1, 0);
  expect_line(decode_line(0x01), 0, 0, 0, 1);
  expect_line(decode_line(0x00), 0, 0, 0, 0);
}

TEST(SwU09, AllBytesPolarityPreserved) {
  for (int b = 0; b < 256; ++b) {
    const auto bits = decode_line(b);
    EXPECT_EQ(bits[0], (b >> 3) & 1);
    EXPECT_EQ(bits[1], (b >> 2) & 1);
    EXPECT_EQ(bits[2], (b >> 1) & 1);
    EXPECT_EQ(bits[3], b & 1);
  }
}

}  // namespace
