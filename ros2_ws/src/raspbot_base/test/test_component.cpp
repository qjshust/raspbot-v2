// Mapping of legacy test/test_component.py (SW-G06..G08, 3 cases) to the
// logic layer, as agreed: the node-level orchestration (rclpy + fake SMBus)
// is not re-implemented here; each assertion is reproduced against the pure
// logic pieces the node calls (wheel_pwm/motor_command pipeline, the resend
// gate, the sensor error tracker + decoders).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "raspbot_base/kinematics.hpp"
#include "raspbot_base/safety.hpp"
#include "raspbot_base/sensors.hpp"

using raspbot_base::Cmd3;
using raspbot_base::CommandGate;
using raspbot_base::MotorCmd;
using raspbot_base::SensorErrorTracker;
using raspbot_base::clamp_velocity;
using raspbot_base::decode_line;
using raspbot_base::decode_range;
using raspbot_base::motor_command;
using raspbot_base::wheel_pwm;

namespace {

constexpr double kK = 0.1245;

void expect_pwm(const std::array<int, 4>& actual, int e0, int e1, int e2,
                int e3) {
  EXPECT_EQ(actual[0], e0);
  EXPECT_EQ(actual[1], e1);
  EXPECT_EQ(actual[2], e2);
  EXPECT_EQ(actual[3], e3);
}

// ------------------------------------------------------------- SW-G06

TEST(SwG06, ForwardCmdDrivesExpectedPwm) {
  // legacy: publish vx=0.2 on /cmd_vel_safe -> FakeSMBus saw [i,0,140] x4
  const Cmd3 c = clamp_velocity(0.2, 0.0, 0.0, 0.30, 1.50);
  EXPECT_DOUBLE_EQ(c.vx, 0.2);
  const auto pwms = wheel_pwm(c.vx, c.vy, kK * c.wz, 700.0, 255, 25);
  expect_pwm(pwms, 140, 140, 140, 140);
  for (const int pwm : pwms) {
    const MotorCmd m = motor_command(pwm);
    EXPECT_EQ(m.dir, 0);
    EXPECT_EQ(m.speed, 140);
  }
}

TEST(SwG06, ReverseCmdDrivesExpectedPwm) {
  const Cmd3 c = clamp_velocity(-0.2, 0.0, 0.0, 0.30, 1.50);
  EXPECT_DOUBLE_EQ(c.vx, -0.2);
  const auto pwms = wheel_pwm(c.vx, c.vy, kK * c.wz, 700.0, 255, 25);
  expect_pwm(pwms, -140, -140, -140, -140);
  for (const int pwm : pwms) {
    const MotorCmd m = motor_command(pwm);
    EXPECT_EQ(m.dir, 1);
    EXPECT_EQ(m.speed, 140);
  }
}

TEST(SwG06, MixedCmdPipeline) {
  // vx=0.1, vy=0.05, wz=0.2（k_geom 折算 k=0.0249）
  // 正确式：l1=0.0251→18→25(死区), l2=0.1251→88, r1=0.1749→122, r2=0.0749→52
  const Cmd3 c = clamp_velocity(0.1, 0.05, 0.2, 0.30, 1.50);
  EXPECT_DOUBLE_EQ(c.vx, 0.1);
  EXPECT_DOUBLE_EQ(c.vy, 0.05);
  EXPECT_DOUBLE_EQ(c.wz, 0.2);
  const auto pwms = wheel_pwm(c.vx, c.vy, kK * c.wz, 700.0, 255, 25);
  expect_pwm(pwms, 25, 88, 122, 52);
}

// ------------------------------------------------------------- SW-G07

TEST(SwG07, ResendGateSemantics) {
  CommandGate gate;
  const std::array<int, 4> pwms = {140, 140, 140, 140};

  EXPECT_TRUE(gate.should_send(pwms, 100.0, 1.0));  // first send
  EXPECT_FALSE(gate.should_send(pwms, 100.5, 1.0));  // unchanged, in window
  EXPECT_TRUE(gate.should_send(pwms, 101.1, 1.0));   // interval elapsed
  EXPECT_TRUE(gate.should_send({120, 120, 120, 120}, 101.2, 1.0));  // changed

  // legacy: constant command for ~3.2 s -> >= 3 sends, gaps <= 1.5 s
  // (criterion: re-send at least every ~1 s, FR-A-06)
  CommandGate g2;
  const std::array<int, 4> p2 = {140, 140, 140, 140};
  std::vector<double> send_times;
  double t = 0.0;
  for (int i = 0; i < 96; ++i) {  // 96 ticks x 1/30 s = 3.2 s
    if (g2.should_send(p2, t, 1.0)) {
      send_times.push_back(t);
    }
    t += 1.0 / 30.0;
  }
  ASSERT_GE(send_times.size(), 3u);
  double max_gap = 0.0;
  for (std::size_t i = 1; i < send_times.size(); ++i) {
    max_gap = std::max(max_gap, send_times[i] - send_times[i - 1]);
  }
  EXPECT_LE(max_gap, 1.5);
}

// ------------------------------------------------------------- SW-G08

TEST(SwG08, ReadFailuresCountedAndWarnRateLimited) {
  // legacy: first 10 reads raise IOError -> i2c_error_count >= 10 (exact
  // equality was racy there; the deterministic fake here yields exactly 10)
  // and the WARN is rate-limited to <= 1 per 5 s.
  SensorErrorTracker tracker;
  double now = 0.2;
  for (int i = 0; i < 10; ++i) {
    tracker.note_error(now, 5.0);
    now += 0.0667;  // three failures per 0.2 s sensor cycle
  }
  EXPECT_EQ(tracker.count(), 10);
  EXPECT_LE(tracker.warn_count(), 2);

  // still inside the rate-limit window: count grows, no new WARN
  EXPECT_FALSE(tracker.note_error(now, 5.0));
  EXPECT_EQ(tracker.count(), 11);
  EXPECT_LE(tracker.warn_count(), 2);

  // after 5 s the WARN may fire again
  EXPECT_TRUE(tracker.note_error(now + 5.1, 5.0));
  EXPECT_LE(tracker.warn_count(), 3);
}

TEST(SwG08, SensorsRecoverAfterFailures) {
  // legacy: fake bus fails the first 10 reads, then returns a healthy board
  // (0x1B=0x02, 0x1A=0x58 -> 0.6 m; 0x0A=0x0F). The 5 Hz loop keeps running
  // (node never exits) and /range + /line_sensor recover. Reproduced at the
  // logic level with a deterministic 3 s x 5 Hz simulation.
  int fail_next = 10;
  int total_reads = 0;
  const auto read_reg = [&](int reg) -> std::pair<bool, int> {
    ++total_reads;
    if (fail_next > 0) {
      --fail_next;
      return {false, 0};  // injected IOError
    }
    if (reg == 0x1B) return {true, 0x02};
    if (reg == 0x1A) return {true, 0x58};
    if (reg == 0x0A) return {true, 0x0F};
    return {true, 0};
  };

  SensorErrorTracker tracker;
  double range = -1.0;
  std::array<int, 4> line_bits = {0, 0, 0, 0};

  for (int cycle = 1; cycle <= 15; ++cycle) {  // 15 cycles x 0.2 s = 3 s
    const double t = 0.2 * cycle;
    const auto hi = read_reg(0x1B);
    const auto lo = read_reg(0x1A);
    const auto line = read_reg(0x0A);
    if (!hi.first) tracker.note_error(t, 5.0);
    if (!lo.first) tracker.note_error(t, 5.0);
    if (!line.first) tracker.note_error(t, 5.0);
    if (hi.first && lo.first) {
      range = decode_range(hi.second, lo.second);
    }
    if (line.first) {
      line_bits = decode_line(line.second);
    }
  }

  EXPECT_EQ(total_reads, 45);      // 5 Hz loop never stopped
  EXPECT_EQ(tracker.count(), 10);  // the 10 injected failures were counted
  EXPECT_LE(tracker.warn_count(), 2);
  EXPECT_GT(range, 0.0);  // /range recovered
  EXPECT_LE(range, 4.0);
  EXPECT_DOUBLE_EQ(range, 0.6);  // 0x0258 = 600 mm
  EXPECT_EQ(line_bits[0], 1);    // /line_sensor recovered (0x0F)
  EXPECT_EQ(line_bits[1], 1);
  EXPECT_EQ(line_bits[2], 1);
  EXPECT_EQ(line_bits[3], 1);
}

}  // namespace
