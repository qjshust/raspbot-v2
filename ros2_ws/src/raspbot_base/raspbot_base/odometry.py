"""Pure dead-reckoning helpers for /odom_raw.

No ROS types, no hardware: importable and testable on a plain Windows box
(SW-U05, SW-U06). The chassis has no wheel encoders, so the node integrates the
*commanded* velocity - open loop, drift included.
"""

import math


def sanitize_dt(dt):
    """Guard the integration time step.

    ``dt <= 0`` (clock jump backwards) or ``dt > 1.0`` s (long stall, e.g.
    after a suspend) returns ``0.0`` so the pose cannot jump. Otherwise the
    original value is returned unchanged (SW-U06).
    """
    if dt <= 0.0 or dt > 1.0:
        return 0.0
    return dt


def integrate_pose(x, y, theta, vx, vy, wz, dt):
    """Semi-implicit Euler step; order identical to the in-tree code.

    First ``theta += wz*dt``, then x/y are updated with the *new* theta:

        x += (vx*cos(theta) - vy*sin(theta)) * dt
        y += (vx*sin(theta) + vy*cos(theta)) * dt

    Returns the (x, y, theta) tuple.
    """
    theta = theta + wz * dt
    x = x + (vx * math.cos(theta) - vy * math.sin(theta)) * dt
    y = y + (vx * math.sin(theta) + vy * math.cos(theta)) * dt
    return (x, y, theta)


def yaw_to_quat(theta):
    """Planar yaw -> unit quaternion (x, y, z, w) with x = y = 0."""
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))
