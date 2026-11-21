#include "raspbot_base/safety.hpp"

#include <algorithm>

namespace raspbot_base {

double clamp_unit(double value) {
  return std::max(0.0, std::min(1.0, value));
}

double clamp_servo_pan(double value) {
  return std::max(0.0, std::min(180.0, value));
}

double clamp_servo_tilt(double value) {
  return std::max(0.0, std::min(110.0, value));
}

bool is_stale(double now_sec, double last_sec, double timeout_sec) {
  return (now_sec - last_sec) > timeout_sec;
}

}  // namespace raspbot_base
