// Pure mecanum kinematics for the RASPBOT-V2 chassis.
//
// No ROS types, no hardware: this header must stay includable and testable
// offline (gtest). It mirrors raspbot_base/kinematics.py from the legacy
// Python implementation bit-for-bit in semantics.
//
// ROS convention (CONTRACTS v1.2 §2): +x forward, +y left, +wz
// counter-clockwise (top view, right-hand rule). Wheel order matches the
// vendor library: L1, L2, R1, R2.
#pragma once

#include <array>
#include <cstdint>

namespace raspbot_base {

struct WheelSpeeds {
  double l1;  // wheel L1 linear velocity (m/s)
  double l2;
  double r1;
  double r2;
};

struct Cmd3 {
  double vx;  // m/s
  double vy;  // m/s
  double wz;  // rad/s
};

/// Signed PWM (-255..255) mapped to the vendor register pair: dir
/// (0=forward, 1=reverse) and speed 0..255. Mirrors Raspbot_Lib.Ctrl_Muto.
struct MotorCmd {
  std::uint8_t dir;
  std::uint8_t speed;
};

/// Mecanum inverse kinematics -> per-wheel linear velocities (m/s).
///
/// Formula identical to the legacy Python implementation:
///   L1 = vx - vy - k*wz
///   L2 = vx + vy + k*wz
///   R1 = vx + vy - k*wz
///   R2 = vx - vy + k*wz
WheelSpeeds wheel_linear_velocities(double vx, double vy, double wz,
                                    double k_geom);

/// Python round(): round-half-to-even (banker's rounding). C++ std::round is
/// half-away-from-zero, so we implement the Python semantic explicitly via
/// std::nearbyint under the default FE_TONEAREST rounding mode.
double round_half_even(double value);

/// Map a velocity command to four PWM ints in [-255, 255].
///
/// ``wz`` is the *effective* rotation term, i.e. the angular velocity already
/// multiplied by k_geom (see wheel_linear_velocities). Pipeline order is
/// preserved from the legacy Python code:
///   1. multiply each wheel linear velocity by pwm_per_mps;
///   2. if the absolute peak exceeds max_pwm, scale all four so the peak
///      equals max_pwm (proportional saturation);
///   3. round each value (banker's rounding, as Python int(round(v))) and
///      multiply by the per-wheel wheel_signs entry;
///   4. deadband: a non-zero value with |v| < deadband becomes +/-deadband;
///   5. clamp each result to [-255, 255].
std::array<int, 4> wheel_pwm(double vx, double vy, double wz,
                             double pwm_per_mps, int max_pwm = 255,
                             int deadband = 25,
                             std::array<int, 4> wheel_signs = {1, 1, 1, 1});

/// Clamp a (vx, vy, wz) command to the configured limits.
Cmd3 clamp_velocity(double vx, double vy, double wz, double max_linear,
                    double max_angular);

/// Signed PWM -> vendor motor register pair (Raspbot_Lib.Ctrl_Muto semantics:
/// negative speed selects direction 1 and abs(speed), clamped to 255).
MotorCmd motor_command(int pwm);

}  // namespace raspbot_base
