// Pure safety helpers: actuator clamps, the command watchdog, and the logic
// extracted from the node for the component-level gtest mapping (SW-G07 /
// SW-G08). No ROS types - testable offline with synthetic timestamps.
//
// Mirrors raspbot_base/safety.py from the legacy Python implementation. The
// clamps deliberately do NOT depend on the vendor library limits.
#pragma once

#include <array>

namespace raspbot_base {

/// Clamp a value into [0, 1] (LED colour channel).
double clamp_unit(double value);

/// Clamp the pan servo (id 1) angle to [0, 180] degrees.
double clamp_servo_pan(double value);

/// Clamp the tilt servo (id 2) angle to [0, 110] degrees (mechanical limit;
/// must not rely on the vendor library clamp).
double clamp_servo_tilt(double value);

/// True when strictly more than timeout_sec elapsed since last_sec.
bool is_stale(double now_sec, double last_sec, double timeout_sec);

/// Command-age watchdog driven by pure time inputs. A watchdog that was never
/// fed counts as stale (fail-safe).
class CommandWatchdog {
 public:
  explicit CommandWatchdog(double timeout_sec = 0.5) : timeout_sec_(timeout_sec) {}

  void feed(double t) {
    fed_ = true;
    last_feed_ = t;
  }

  bool stale(double t) const {
    if (!fed_) {
      return true;
    }
    return is_stale(t, last_feed_, timeout_sec_);
  }

  bool has_feed() const { return fed_; }
  double last_feed() const { return last_feed_; }

 private:
  double timeout_sec_;
  bool fed_ = false;
  double last_feed_ = 0.0;
};

/// Resend policy of the control loop (FR-A-06 / SW-G07), extracted from the
/// node for offline testing: send when the PWM set changed or when more than
/// ``resend_interval`` seconds elapsed since the last send.
class CommandGate {
 public:
  bool should_send(const std::array<int, 4>& pwms, double now_sec,
                   double resend_interval) {
    if (!has_last_ || pwms != last_pwms_) {
      last_pwms_ = pwms;
      has_last_ = true;
      last_send_ = now_sec;
      return true;
    }
    if (now_sec - last_send_ > resend_interval) {
      last_send_ = now_sec;
      return true;
    }
    return false;
  }

 private:
  std::array<int, 4> last_pwms_{0, 0, 0, 0};
  bool has_last_ = false;
  double last_send_ = 0.0;
};

/// Sensor error accounting of the node (SW-G08), extracted for offline
/// testing: counts every error; ``note_error`` returns true exactly when a
/// WARN should be emitted under a rate limit of ``warn_interval`` seconds.
class SensorErrorTracker {
 public:
  bool note_error(double now_sec, double warn_interval) {
    ++count_;
    if (last_warn_ < 0.0 || now_sec - last_warn_ >= warn_interval) {
      ++warn_count_;
      last_warn_ = now_sec;
      return true;
    }
    return false;
  }

  int count() const { return count_; }
  int warn_count() const { return warn_count_; }

 private:
  int count_ = 0;
  int warn_count_ = 0;
  double last_warn_ = -1.0;
};

}  // namespace raspbot_base
