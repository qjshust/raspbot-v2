"""SW-U01..U04: mecanum inverse kinematics, PWM saturation, deadband, signs.

Pure functions only - no ROS, no hardware.
"""

import math

from raspbot_base.kinematics import (clamp_velocity,
                                     wheel_linear_velocities, wheel_pwm)

K = 0.1245  # k_geom from config/params.yaml


def _assert_close(actual, expected, tol=1e-9):
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        assert abs(a - e) < tol, '%r !~ %r' % (actual, expected)


# ---------------------------------------------------------------- SW-U01

def test_sw_u01_forward():
    _assert_close(wheel_linear_velocities(0.2, 0.0, 0.0, K),
                  (0.2, 0.2, 0.2, 0.2))


def test_sw_u01_backward():
    _assert_close(wheel_linear_velocities(-0.2, 0.0, 0.0, K),
                  (-0.2, -0.2, -0.2, -0.2))


def test_sw_u01_strafe_left():
    # +vy is left: L2/R1 turn forward, L1/R2 backward
    _assert_close(wheel_linear_velocities(0.0, 0.2, 0.0, K),
                  (-0.2, 0.2, 0.2, -0.2))


def test_sw_u01_strafe_right():
    _assert_close(wheel_linear_velocities(0.0, -0.2, 0.0, K),
                  (0.2, -0.2, -0.2, 0.2))


def test_sw_u01_rotate_left_ccw():
    k = K * 0.5
    _assert_close(wheel_linear_velocities(0.0, 0.0, 0.5, K),
                  (-k, k, -k, k))


def test_sw_u01_rotate_right_cw():
    k = K * 0.5
    _assert_close(wheel_linear_velocities(0.0, 0.0, -0.5, K),
                  (k, -k, k, -k))


def test_sw_u01_mixed_command_matches_formula():
    vx, vy, wz = 0.1, 0.05, 0.2
    k = K * wz
    _assert_close(wheel_linear_velocities(vx, vy, wz, K),
                  (vx - vy - k, vx + vy + k, vx + vy - k, vx - vy + k))


# ---------------------------------------------------------------- SW-U02

def test_sw_u02_peak_scaled_to_max_pwm():
    out = wheel_pwm(1.0, 0.0, 0.0, pwm_per_mps=700.0, max_pwm=255,
                    deadband=25)
    assert out == (255, 255, 255, 255)


def test_sw_u02_proportional_scaling_within_1_lsb():
    # effective wz already includes k_geom: raw = (0.2, 0.8, 0.8, 0.2)
    vx, vy, wz = 0.5, 0.0, 0.3
    raw = wheel_linear_velocities(vx, vy, wz, 1.0)
    peak = max(abs(v) for v in raw)
    assert peak > 255.0 / 700.0  # pre-scaled peak exceeds max_pwm

    out = wheel_pwm(vx, vy, wz, pwm_per_mps=700.0, max_pwm=255, deadband=25)
    assert max(out) == 255
    for value, r in zip(out, raw):
        exact = r * (255.0 / peak)
        assert abs(value - exact) < 1.0, (value, exact)


def test_sw_u02_no_scaling_when_below_max():
    out = wheel_pwm(0.2, 0.0, 0.0, pwm_per_mps=700.0, max_pwm=255,
                    deadband=25)
    assert out == (140, 140, 140, 140)


# ---------------------------------------------------------------- SW-U03

def _pwm_for(vx):
    return wheel_pwm(vx, 0.0, 0.0, pwm_per_mps=100.0, max_pwm=255,
                     deadband=25)


def test_sw_u03_zero_stays_zero():
    assert _pwm_for(0.0) == (0, 0, 0, 0)


def test_sw_u03_below_deadband_maps_to_deadband():
    assert _pwm_for(0.01) == (25, 25, 25, 25)
    assert _pwm_for(0.24) == (25, 25, 25, 25)
    assert _pwm_for(-0.01) == (-25, -25, -25, -25)
    assert _pwm_for(-0.24) == (-25, -25, -25, -25)


def test_sw_u03_deadband_boundary_kept():
    assert _pwm_for(0.25) == (25, 25, 25, 25)
    assert _pwm_for(-0.25) == (-25, -25, -25, -25)


def test_sw_u03_above_deadband_unchanged():
    assert _pwm_for(0.26) == (26, 26, 26, 26)
    assert _pwm_for(-0.26) == (-26, -26, -26, -26)


# ---------------------------------------------------------------- SW-U04

def test_sw_u04_base_signs_all_positive():
    out = wheel_pwm(0.2, 0.0, 0.0, pwm_per_mps=100.0, deadband=1)
    assert out == (20, 20, 20, 20)


def test_sw_u04_each_wheel_independent():
    cases = [((-1, 1, 1, 1), (-20, 20, 20, 20)),
             ((1, -1, 1, 1), (20, -20, 20, 20)),
             ((1, 1, -1, 1), (20, 20, -20, 20)),
             ((1, 1, 1, -1), (20, 20, 20, -20))]
    for signs, expected in cases:
        out = wheel_pwm(0.2, 0.0, 0.0, pwm_per_mps=100.0, deadband=1,
                        wheel_signs=signs)
        assert out == expected


def test_sw_u04_sign_applied_before_deadband():
    # -0.01 m/s -> pwm -1 -> sign -1 -> +1 -> deadband +25, not -25
    out = wheel_pwm(-0.01, 0.0, 0.0, pwm_per_mps=100.0,
                    wheel_signs=(-1, 1, 1, 1))
    assert out == (25, -25, -25, -25)


# ------------------------------------------------------------- clamp_velocity

def test_clamp_velocity_limits():
    assert clamp_velocity(1.0, -1.0, 5.0, 0.3, 1.5) == (0.3, -0.3, 1.5)


def test_clamp_velocity_passthrough():
    assert clamp_velocity(0.1, 0.05, 0.2, 0.3, 1.5) == (0.1, 0.05, 0.2)
