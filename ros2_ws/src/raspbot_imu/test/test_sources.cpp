/// 数据源测试（1:1 映射 Python 版 test/test_sources.py，7 用例）。
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "raspbot_imu/mock_source.hpp"
#include "raspbot_imu/sources.hpp"
#include "raspbot_imu/types.hpp"
#include "raspbot_imu/witmotion.hpp"

using namespace raspbot_imu;

namespace {

/// 构造 11 字节维特帧（小端 int16 三元组 + 2 字节填充 + 校验和）。
std::vector<std::uint8_t> wm_frame(std::uint8_t kind, std::int16_t x = 0,
                                   std::int16_t y = 0, std::int16_t z = 0) {
  std::vector<std::uint8_t> body{0x55, kind};
  const std::int16_t values[3] = {x, y, z};
  for (std::int16_t v : values) {
    body.push_back(static_cast<std::uint8_t>(v & 0xFF));
    body.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
  }
  body.push_back(0x00);
  body.push_back(0x00);
  std::uint8_t sum = 0;
  for (std::size_t i = 0; i < 10; ++i) {
    sum = static_cast<std::uint8_t>(sum + body[i]);
  }
  body.push_back(sum);
  return body;
}

}  // namespace

TEST(Sources, ParserAccelAndGyro) {
  WitMotionParser parser(16.0, 2000.0);
  auto frame = wm_frame(WitMotionParser::kAccel, 0, 0, 16384);
  EXPECT_TRUE(parser.feed(frame));
  EXPECT_TRUE(parser.seen_accel);
  EXPECT_NEAR(parser.accel[2], 8.0 * kGravity, 8.0 * kGravity * 1e-6);

  frame = wm_frame(WitMotionParser::kGyro, 0, 0, 16384);
  EXPECT_TRUE(parser.feed(frame));
  EXPECT_NEAR(parser.gyro[2], 1000.0 * kPi / 180.0,
              1000.0 * kPi / 180.0 * 1e-6);
}

TEST(Sources, ParserResyncsAfterGarbage) {
  WitMotionParser parser;
  std::vector<std::uint8_t> data{0xde, 0xad, 0xbe, 0xef};
  const auto frame = wm_frame(WitMotionParser::kGyro, 3277, 0, 0);
  data.insert(data.end(), frame.begin(), frame.end());
  EXPECT_TRUE(parser.feed(data));
  EXPECT_NEAR(parser.gyro[0], 200.0 * kPi / 180.0,
              200.0 * kPi / 180.0 * 1e-3);
}

TEST(Sources, ParserHandlesSplitFrame) {
  WitMotionParser parser;
  const auto frame = wm_frame(WitMotionParser::kAccel, 0, 0, 8192);
  const std::vector<std::uint8_t> first(frame.begin(), frame.begin() + 6);
  const std::vector<std::uint8_t> second(frame.begin() + 6, frame.end());
  EXPECT_FALSE(parser.feed(first));
  EXPECT_TRUE(parser.feed(second));
  EXPECT_NEAR(parser.accel[2], 4.0 * kGravity, 4.0 * kGravity * 1e-6);
}

TEST(Sources, ParserRejectsBadChecksum) {
  WitMotionParser parser;
  auto frame = wm_frame(WitMotionParser::kAccel, 0, 0, 16384);
  frame.back() = static_cast<std::uint8_t>((frame.back() + 1) & 0xFF);
  EXPECT_FALSE(parser.feed(frame));
  EXPECT_FALSE(parser.seen_accel);
}

TEST(Sources, MockSourceRateAndMotion) {
  MockSource source(100.0);
  std::vector<ImuSample> samples;
  for (int i = 0; i < 100; ++i) {
    samples.push_back(*source.read());
  }
  EXPECT_NEAR(samples[0].t, 0.0, 1e-12);
  EXPECT_NEAR(samples[99].t, 0.99, 1e-12);
  EXPECT_NEAR(source.yaw_rate(2.5), kPi / 180.0 * 45.0, 1e-12);
  EXPECT_DOUBLE_EQ(source.yaw_rate(0.5), 0.0);
  EXPECT_NEAR(source.yaw_rate(8.0), -kPi / 180.0 * 60.0, 1e-12);
}

TEST(Sources, MockSourceBiasAndNoise) {
  MockSource source(100.0, {0.010, -0.005, 0.020}, 0.0, 0.0, 0);
  ImuSample sample;
  for (int i = 0; i < 200; ++i) {  // 前 2 s 静止，只有零偏
    sample = *source.read();
  }
  EXPECT_NEAR(sample.gyro[0], 0.010, 1e-12);
  EXPECT_NEAR(sample.gyro[1], -0.005, 1e-12);
  EXPECT_NEAR(sample.gyro[2], 0.020, 1e-12);
  EXPECT_NEAR(sample.accel[0], 0.0, 1e-12);
  EXPECT_NEAR(sample.accel[1], 0.0, 1e-12);
  EXPECT_NEAR(sample.accel[2], kGravity, 1e-12);
}

TEST(Sources, MakeSourceFactory) {
  const SourceOptions opts;
  const auto mock = make_source("mock", opts);
  EXPECT_NE(dynamic_cast<MockSource*>(mock.get()), nullptr);
  try {
    make_source("does-not-exist", opts);
    FAIL() << "应当抛出 std::invalid_argument";
  } catch (const std::invalid_argument& exc) {
    EXPECT_NE(std::string(exc.what()).find("未知数据源"), std::string::npos);
  }
}
