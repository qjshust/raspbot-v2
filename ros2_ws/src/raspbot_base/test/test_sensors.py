"""SW-U08, SW-U09: ultrasonic range and line-sensor decoding.

Pure functions only - no ROS, no hardware.
"""

from raspbot_base.sensors import decode_line, decode_range

# ---------------------------------------------------------------- SW-U08

def test_sw_u08_basic_conversion():
    # 0x0258 = 600 mm -> 0.6 m
    assert decode_range(0x02, 0x58) == 0.6
    assert decode_range(0x00, 0x64) == 0.1     # 100 mm
    assert decode_range(0x03, 0xE8) == 1.0     # 1000 mm


def test_sw_u08_out_of_range_maps_to_max_range():
    assert decode_range(0xFF, 0xFF) == 4.0     # 0xFFFF: over range
    assert decode_range(0x00, 0x00) == 4.0     # 0: no echo
    assert decode_range(0x0F, 0xA0) == 4.0     # 4000 mm > 4.0 m


def test_sw_u08_never_negative():
    for hi in (0x00, 0xFF, 0x02, -1):
        for lo in (0x00, 0xFF, 0x58, -1):
            assert decode_range(hi, lo) >= 0.0, (hi, lo)


def test_sw_u08_custom_max_range():
    assert decode_range(0x02, 0x58, max_range=2.0) == 0.6
    assert decode_range(0xFF, 0xFF, max_range=2.0) == 2.0
    assert decode_range(0x00, 0x00, max_range=2.0) == 2.0


# ---------------------------------------------------------------- SW-U09

def test_sw_u09_known_bytes():
    assert decode_line(0x0F) == (1, 1, 1, 1)
    assert decode_line(0x0A) == (1, 0, 1, 0)
    assert decode_line(0x05) == (0, 1, 0, 1)
    assert decode_line(0x08) == (1, 0, 0, 0)
    assert decode_line(0x04) == (0, 1, 0, 0)
    assert decode_line(0x02) == (0, 0, 1, 0)
    assert decode_line(0x01) == (0, 0, 0, 1)
    assert decode_line(0x00) == (0, 0, 0, 0)


def test_sw_u09_all_bytes_polarity_preserved():
    for b in range(256):
        assert decode_line(b) == ((b >> 3) & 1, (b >> 2) & 1,
                                  (b >> 1) & 1, b & 1)
