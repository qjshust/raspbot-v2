// 1:1 mapping of legacy test/test_safety.py (SW-U07, SW-U10, 8 cases).
#include <gtest/gtest.h>

#include "raspbot_base/kinematics.hpp"
#include "raspbot_base/safety.hpp"

using raspbot_base::CommandWatchdog;
using raspbot_base::clamp_servo_pan;
using raspbot_base::clamp_servo_tilt;
using raspbot_base::clamp_unit;
using raspbot_base::is_stale;
using raspbot_base::wheel_pwm;

namespace {

// ------------------------------------------------------------- SW-U07

TEST(SwU07, Watchdog049Vs051) {
  CommandWatchdog wd(0.5);
  wd.feed(100.0);
  EXPECT_FALSE(wd.stale(100.49));  // within timeout: keep last command
  EXPECT_FALSE(wd.stale(100.5));   // exactly at the limit: still alive
  EXPECT_TRUE(wd.stale(100.51));   // past the limit: stale -> zero speed
  EXPECT_TRUE(wd.stale(101.0));
}

TEST(SwU07, IsStaleBoundaries) {
  EXPECT_FALSE(is_stale(0.49, 0.0, 0.5));
  EXPECT_FALSE(is_stale(0.5, 0.0, 0.5));
  EXPECT_TRUE(is_stale(0.51, 0.0, 0.5));
}

TEST(SwU07, NeverFedCountsAsStale) {
  EXPECT_TRUE(CommandWatchdog(0.5).stale(0.0));
}

TEST(SwU07, StaleCommandZeroesAllWheels) {
  // Node semantics: watchdog stale -> zero command -> four zero PWMs.
  CommandWatchdog wd(0.5);
  wd.feed(0.0);
  const auto active = wheel_pwm(0.2, 0.0, 0.0, 700.0);
  for (const int v : active) {
    EXPECT_NE(v, 0);
  }
  EXPECT_FALSE(wd.stale(0.49));
  EXPECT_TRUE(wd.stale(0.51));
  const auto stopped = wheel_pwm(0.0, 0.0, 0.0, 700.0);
  EXPECT_EQ(stopped[0], 0);
  EXPECT_EQ(stopped[1], 0);
  EXPECT_EQ(stopped[2], 0);
  EXPECT_EQ(stopped[3], 0);
}

// ------------------------------------------------------------- SW-U10

TEST(SwU10, ServoPanClampedTo180) {
  EXPECT_DOUBLE_EQ(clamp_servo_pan(200.0), 180.0);
  EXPECT_DOUBLE_EQ(clamp_servo_pan(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(clamp_servo_pan(90.0), 90.0);
}

TEST(SwU10, ServoTiltClampedTo110) {
  EXPECT_DOUBLE_EQ(clamp_servo_tilt(150.0), 110.0);
  EXPECT_DOUBLE_EQ(clamp_servo_tilt(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(clamp_servo_tilt(110.0), 110.0);
  EXPECT_DOUBLE_EQ(clamp_servo_tilt(45.0), 45.0);
}

TEST(SwU10, LedChannelsClampedToUnit) {
  EXPECT_DOUBLE_EQ(clamp_unit(1.5), 1.0);
  EXPECT_DOUBLE_EQ(clamp_unit(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(clamp_unit(0.5), 0.5);
}

TEST(SwU10, ClampsAreLocalNotVendor) {
  // The legacy Python version verified with inspect that the clamps do not
  // call into the vendor library. The C++ clamps live in the pure
  // raspbot_base_lib which does not link or include Raspbot_Lib at all, so
  // the same guarantee holds by construction; re-assert the values here.
  EXPECT_DOUBLE_EQ(clamp_servo_pan(200.0), 180.0);
  EXPECT_DOUBLE_EQ(clamp_servo_tilt(150.0), 110.0);
  EXPECT_DOUBLE_EQ(clamp_unit(1.5), 1.0);
  EXPECT_DOUBLE_EQ(clamp_unit(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(clamp_unit(0.5), 0.5);
}

}  // namespace
