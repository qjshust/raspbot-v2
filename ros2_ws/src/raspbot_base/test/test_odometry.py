"""SW-U05, SW-U06: pose integration and time-step protection.

Pure functions only - no ROS, no hardware.
"""

import math

from raspbot_base.odometry import integrate_pose, sanitize_dt, yaw_to_quat

# ---------------------------------------------------------------- SW-U05

def test_sw_u05_straight_exact():
    # 0.2 m/s for 10 s, single step: x must be exactly 2.0
    x, y, theta = integrate_pose(0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 10.0)
    assert x == 2.0
    assert y == 0.0
    assert theta == 0.0


def test_sw_u05_straight_multistep():
    x, y, theta = 0.0, 0.0, 0.0
    for _ in range(100):
        x, y, theta = integrate_pose(x, y, theta, 0.2, 0.0, 0.0, 0.1)
    assert abs(x - 2.0) < 1e-9
    assert abs(y - 0.0) < 1e-9
    assert abs(theta - 0.0) < 1e-9


def test_sw_u05_diagonal():
    x, y, theta = integrate_pose(0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 2.0)
    assert abs(x - 0.4) < 1e-9
    assert abs(y - 0.4) < 1e-9
    assert theta == 0.0


def test_sw_u05_rotate_in_place():
    x, y, theta = integrate_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 2.0)
    assert x == 0.0 and y == 0.0
    assert abs(theta - 1.0) < 1e-9


def test_sw_u05_strafe_and_turn_right_hand_rule():
    # +vy strafes left, +wz yaws CCW (top view): the heading rotates left so
    # the leftward velocity gains a -x (backward) component.
    x, y, theta = integrate_pose(0.0, 0.0, 0.0, 0.0, 0.2, 0.5, 0.5)
    assert abs(theta - 0.25) < 1e-9
    expected_x = (0.0 * math.cos(0.25) - 0.2 * math.sin(0.25)) * 0.5
    expected_y = (0.0 * math.sin(0.25) + 0.2 * math.cos(0.25)) * 0.5
    assert abs(x - expected_x) < 1e-9
    assert abs(y - expected_y) < 1e-9
    assert x < 0.0 and y > 0.0


def test_sw_u05_multistep_matches_reference():
    # 400 x 5 ms steps; the reference recomputes the same discrete map
    vx, vy, wz, dt = 0.1, 0.05, 0.3, 0.005
    xr = yr = thr = 0.0
    for _ in range(400):
        thr = thr + wz * dt
        xr = xr + (vx * math.cos(thr) - vy * math.sin(thr)) * dt
        yr = yr + (vx * math.sin(thr) + vy * math.cos(thr)) * dt
    x, y, th = 0.0, 0.0, 0.0
    for _ in range(400):
        x, y, th = integrate_pose(x, y, th, vx, vy, wz, dt)
    assert abs(x - xr) < 1e-9
    assert abs(y - yr) < 1e-9
    assert abs(th - thr) < 1e-9


def test_yaw_to_quat_identity_and_quarter_turn():
    assert yaw_to_quat(0.0) == (0.0, 0.0, 0.0, 1.0)
    qx, qy, qz, qw = yaw_to_quat(math.pi / 2.0)
    assert abs(qx) < 1e-9 and abs(qy) < 1e-9
    assert abs(qz - math.sqrt(0.5)) < 1e-9
    assert abs(qw - math.sqrt(0.5)) < 1e-9


# ---------------------------------------------------------------- SW-U06

def test_sw_u06_sanitize_dt():
    assert sanitize_dt(0.0) == 0.0
    assert sanitize_dt(-0.001) == 0.0
    assert sanitize_dt(-10.0) == 0.0
    assert sanitize_dt(1.5) == 0.0
    assert sanitize_dt(100.0) == 0.0
    assert sanitize_dt(1.0) == 1.0
    assert sanitize_dt(0.5) == 0.5


def test_sw_u06_no_pose_jump_on_bad_dt():
    pose = (1.0, 2.0, 0.5)
    for bad in (0.0, -0.1, 1.5, 100.0):
        x, y, th = integrate_pose(pose[0], pose[1], pose[2],
                                  0.2, 0.1, 0.3, sanitize_dt(bad))
        assert (x, y, th) == pose
