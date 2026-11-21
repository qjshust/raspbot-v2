// Pure decoders for the expansion-board sensors (no ROS types).
//
// Mirrors raspbot_base/sensors.py from the legacy Python implementation.
#pragma once

#include <array>
#include <cstdint>

namespace raspbot_base {

/// Ultrasonic register pair -> range in metres.
///
/// raw = (hi << 8) | lo is the distance in millimetres (register 0x1B is the
/// high byte, 0x1A the low byte). raw == 0xFFFF (out of range) and raw == 0
/// (no echo) both map to max_range; otherwise min(raw / 1000.0, max_range).
/// Never returns a negative value.
double decode_range(int hi_byte, int lo_byte, double max_range = 4.0);

/// Line sensor byte -> (x1, x2, x3, x4).
///
/// Bit order and polarity are exactly the vendor convention (register 0x0A):
/// x1 = (b >> 3) & 1, x2 = (b >> 2) & 1, x3 = (b >> 1) & 1, x4 = b & 1.
/// Polarity is NOT flipped.
std::array<int, 4> decode_line(int raw_byte);

}  // namespace raspbot_base
