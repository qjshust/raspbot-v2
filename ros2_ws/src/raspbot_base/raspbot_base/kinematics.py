"""Pure mecanum kinematics for the RASPBOT-V2 chassis.

This module must stay importable on a plain Windows box: no rclpy, no
nav_msgs, no smbus. Everything here is plain float/int arithmetic so it can be
unit-tested offline (SW-U01..U04).

ROS convention (CONTRACTS v1.2 §2): +x forward, +y left, +wz
counter-clockwise (top view, right-hand rule). Wheel order matches the
vendor library: L1, L2, R1, R2.
"""

WHEEL_ORDER = ('L1', 'L2', 'R1', 'R2')


def wheel_linear_velocities(vx, vy, wz, k_geom):
    """Mecanum inverse kinematics -> per-wheel linear velocities (m/s).

    Formula is identical to the original in-tree implementation:

        L1 = vx - vy - k*wz
        L2 = vx + vy + k*wz
        R1 = vx + vy - k*wz
        R2 = vx - vy + k*wz
    """
    k = k_geom
    return (vx - vy - k * wz,
            vx + vy + k * wz,
            vx + vy - k * wz,
            vx - vy + k * wz)


def _clamp(value, low, high):
    return max(low, min(high, value))


def wheel_pwm(vx, vy, wz, *, pwm_per_mps, max_pwm=255, deadband=25,
              wheel_signs=(1, 1, 1, 1)):
    """Map a velocity command to four PWM ints in [-255, 255].

    ``wz`` is the *effective* rotation term, i.e. the angular velocity already
    multiplied by k_geom (see :func:`wheel_linear_velocities`).

    Pipeline (order preserved from the original in-tree code):

    1. multiply each wheel linear velocity by ``pwm_per_mps``;
    2. if the absolute peak exceeds ``max_pwm``, scale all four so the peak
       equals ``max_pwm`` (proportional saturation, SW-U02);
    3. round each value (``int(round(v))``, banker's rounding as before) and
       multiply by the per-wheel ``wheel_signs`` entry (SW-U04);
    4. deadband: a non-zero value with |v| < ``deadband`` becomes
       +/-``deadband`` (SW-U03);
    5. clamp each result to [-255, 255].

    Returns a tuple of four ints in wheel order (L1, L2, R1, R2).
    """
    raw = wheel_linear_velocities(vx, vy, wz, 1.0)

    pwm = [v * pwm_per_mps for v in raw]
    peak = max(abs(v) for v in pwm)
    if peak > max_pwm:
        scale = max_pwm / peak
        pwm = [v * scale for v in pwm]

    out = []
    for value, sign in zip(pwm, wheel_signs):
        value = int(round(value)) * sign
        if value != 0 and abs(value) < deadband:
            value = deadband if value > 0 else -deadband
        out.append(int(_clamp(value, -255, 255)))
    return tuple(out)


def clamp_velocity(vx, vy, wz, max_linear, max_angular):
    """Clamp a (vx, vy, wz) command to the configured limits.

    Returns a (vx, vy, wz) tuple.
    """
    return (_clamp(vx, -max_linear, max_linear),
            _clamp(vy, -max_linear, max_linear),
            _clamp(wz, -max_angular, max_angular))
