"""Pure safety helpers: actuator clamps and the command watchdog.

No ROS imports so the logic is testable offline with synthetic timestamps
(SW-U07, SW-U10). The clamps deliberately do NOT depend on the vendor library
limits: the library silently re-clamps inside Raspbot_Lib, which hides
configuration errors from us (known defect #2).
"""


def clamp_unit(v):
    """Clamp a value into [0, 1] (LED colour channel)."""
    return max(0.0, min(1.0, v))


def clamp_servo_pan(v):
    """Clamp the pan servo (id 1) angle to [0, 180] degrees."""
    return max(0.0, min(180.0, v))


def clamp_servo_tilt(v):
    """Clamp the tilt servo (id 2) angle to [0, 110] degrees.

    Servo 2 is mechanically limited to ~110 degrees; commanding 180 fights the
    mount. This clamp exists in OUR layer so the behaviour is testable and
    does not rely on the vendor library (SW-U10).
    """
    return max(0.0, min(110.0, v))


def is_stale(now_sec, last_sec, timeout_sec):
    """True when strictly more than ``timeout_sec`` elapsed since ``last_sec``."""
    return (now_sec - last_sec) > timeout_sec


class CommandWatchdog:
    """Command-age watchdog driven by pure time inputs.

    Feed it with the timestamp of every received command; ``stale(t)`` is True
    once more than ``timeout_sec`` has passed since the last feed. Time is
    passed in as a float (seconds), so the 0.49 s / 0.51 s boundary is
    testable without any clock (SW-U07). A watchdog that was never fed counts
    as stale (fail-safe).
    """

    def __init__(self, timeout_sec):
        self.timeout_sec = float(timeout_sec)
        self.last_feed = None

    def feed(self, t):
        self.last_feed = t

    def stale(self, t):
        if self.last_feed is None:
            return True
        return is_stale(t, self.last_feed, self.timeout_sec)
