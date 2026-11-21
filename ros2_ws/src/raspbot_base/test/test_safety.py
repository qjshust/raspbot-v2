"""SW-U07, SW-U10: command watchdog and actuator clamps.

Pure functions only - no ROS, no hardware.
"""

from raspbot_base.kinematics import wheel_pwm
from raspbot_base.safety import (CommandWatchdog, clamp_servo_pan,
                                 clamp_servo_tilt, clamp_unit, is_stale)

# ---------------------------------------------------------------- SW-U07

def test_sw_u07_watchdog_0_49_vs_0_51():
    wd = CommandWatchdog(0.5)
    wd.feed(100.0)
    assert not wd.stale(100.49)   # within timeout: keep last command
    assert not wd.stale(100.5)    # exactly at the limit: still alive
    assert wd.stale(100.51)       # past the limit: stale -> zero speed
    assert wd.stale(101.0)


def test_sw_u07_is_stale_boundaries():
    assert not is_stale(0.49, 0.0, 0.5)
    assert not is_stale(0.5, 0.0, 0.5)
    assert is_stale(0.51, 0.0, 0.5)


def test_sw_u07_never_fed_counts_as_stale():
    assert CommandWatchdog(0.5).stale(0.0)


def test_sw_u07_stale_command_zeroes_all_wheels():
    # Node semantics: watchdog stale -> zero command -> four zero PWMs.
    wd = CommandWatchdog(0.5)
    wd.feed(0.0)
    active = wheel_pwm(0.2, 0.0, 0.0, pwm_per_mps=700.0)
    assert all(v != 0 for v in active)
    assert not wd.stale(0.49)
    assert wd.stale(0.51)
    stopped = wheel_pwm(0.0, 0.0, 0.0, pwm_per_mps=700.0)
    assert stopped == (0, 0, 0, 0)


# ---------------------------------------------------------------- SW-U10

def test_sw_u10_servo_pan_clamped_to_180():
    assert clamp_servo_pan(200.0) == 180.0
    assert clamp_servo_pan(-1.0) == 0.0
    assert clamp_servo_pan(90.0) == 90.0


def test_sw_u10_servo_tilt_clamped_to_110():
    assert clamp_servo_tilt(150.0) == 110.0
    assert clamp_servo_tilt(-1.0) == 0.0
    assert clamp_servo_tilt(110.0) == 110.0
    assert clamp_servo_tilt(45.0) == 45.0


def test_sw_u10_led_channels_clamped_to_unit():
    assert clamp_unit(1.5) == 1.0
    assert clamp_unit(-1.0) == 0.0
    assert clamp_unit(0.5) == 0.5


def test_sw_u10_clamps_are_local_not_vendor():
    # The vendor library re-clamps internally; OUR layer must clamp first so
    # the behaviour is testable offline (SW-U10 criterion).
    import inspect
    for fn in (clamp_servo_pan, clamp_servo_tilt, clamp_unit):
        assert 'Raspbot' not in inspect.getsource(fn)
