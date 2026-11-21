#include "raspbot_base/sensors.hpp"

#include <algorithm>
#include <cstdint>

namespace raspbot_base {

double decode_range(int hi_byte, int lo_byte, double max_range) {
  const std::uint32_t raw =
      ((static_cast<std::uint32_t>(hi_byte) << 8) |
       static_cast<std::uint32_t>(lo_byte)) &
      0xFFFFu;
  if (raw == 0 || raw == 0xFFFF) {
    return max_range;
  }
  return std::max(0.0, std::min(raw / 1000.0, max_range));
}

std::array<int, 4> decode_line(int raw_byte) {
  const int b = raw_byte;
  return {(b >> 3) & 1, (b >> 2) & 1, (b >> 1) & 1, b & 1};
}

}  // namespace raspbot_base
