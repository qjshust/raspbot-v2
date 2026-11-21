"""Pure decoders for the expansion-board sensors.

Importable on a plain Windows box (no rclpy / smbus) so the decoders can be
unit-tested offline (SW-U08, SW-U09).
"""


def decode_range(hi_byte, lo_byte, max_range=4.0):
    """Ultrasonic register pair -> range in metres.

    ``raw = (hi << 8) | lo`` is the distance in millimetres (register 0x1B is
    the high byte, 0x1A the low byte). ``raw == 0xFFFF`` (out of range) and
    ``raw == 0`` (no echo) both map to ``max_range``; otherwise the value is
    ``min(raw / 1000.0, max_range)``. Never returns a negative value (SW-U08).
    """
    raw = ((int(hi_byte) << 8) | int(lo_byte)) & 0xFFFF
    if raw == 0 or raw == 0xFFFF:
        return max_range
    return max(0.0, min(raw / 1000.0, max_range))


def decode_line(raw_byte):
    """Line sensor byte -> (x1, x2, x3, x4).

    Bit order and polarity are exactly the vendor convention (register 0x0A):
    ``x1 = (b >> 3) & 1, x2 = (b >> 2) & 1, x3 = (b >> 1) & 1, x4 = b & 1``.
    Polarity is NOT flipped (SW-U09).
    """
    b = int(raw_byte)
    return ((b >> 3) & 1, (b >> 2) & 1, (b >> 1) & 1, b & 1)
