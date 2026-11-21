#include "raspbot_base/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace raspbot_base {

WheelSpeeds wheel_linear_velocities(double vx, double vy, double wz,
                                    double k_geom) {
  const double k = k_geom;
  return {vx - vy - k * wz, vx + vy + k * wz, vx + vy - k * wz, vx - vy + k * wz};
}

double round_half_even(double value) {
  // std::nearbyint honours the current rounding mode; the process default is
  // round-to-nearest-even, which is exactly Python's round().
  return std::nearbyint(value);
}

std::array<int, 4> wheel_pwm(double vx, double vy, double wz,
                             double pwm_per_mps, int max_pwm, int deadband,
                             std::array<int, 4> wheel_signs) {
  const WheelSpeeds raw = wheel_linear_velocities(vx, vy, wz, 1.0);

  std::array<double, 4> pwm = {raw.l1 * pwm_per_mps, raw.l2 * pwm_per_mps,
                               raw.r1 * pwm_per_mps, raw.r2 * pwm_per_mps};
  double peak = 0.0;
  for (const double v : pwm) {
    peak = std::max(peak, std::fabs(v));
  }
  if (peak > max_pwm) {
    const double scale = max_pwm / peak;
    for (double& v : pwm) {
      v *= scale;
    }
  }

  std::array<int, 4> out = {0, 0, 0, 0};
  for (std::size_t i = 0; i < 4; ++i) {
    int value = static_cast<int>(round_half_even(pwm[i])) * wheel_signs[i];
    if (value != 0 && std::abs(value) < deadband) {
      value = (value > 0) ? deadband : -deadband;
    }
    out[i] = std::clamp(value, -255, 255);
  }
  return out;
}

Cmd3 clamp_velocity(double vx, double vy, double wz, double max_linear,
                    double max_angular) {
  const auto clamp1 = [](double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  };
  return {clamp1(vx, -max_linear, max_linear),
          clamp1(vy, -max_linear, max_linear),
          clamp1(wz, -max_angular, max_angular)};
}

MotorCmd motor_command(int pwm) {
  const int s = std::clamp(pwm, -255, 255);
  if (s < 0) {
    return {1, static_cast<std::uint8_t>(-s)};
  }
  return {0, static_cast<std::uint8_t>(s)};
}

}  // namespace raspbot_base
