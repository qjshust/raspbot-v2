// Minimal I2C master for the RASPBOT-V2 expansion board (0x2B on /dev/i2c-1).
//
// Uses the kernel's native I2C device interface (<linux/i2c-dev.h>, ioctl
// I2C_SLAVE) - no libi2c / smbus dependency, nothing to apt install. Unlike
// the vendor Python library, errors are NOT silently swallowed: every
// transaction reports success/failure so the node can count them.
#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

namespace raspbot_base {

class I2cBus {
 public:
  /// Opens ``path`` and selects ``addr``; throws std::runtime_error on
  /// failure (mirrors the legacy Python node, where smbus.SMBus(1) raised).
  I2cBus(const char* path, std::uint8_t addr);
  ~I2cBus();

  I2cBus(const I2cBus&) = delete;
  I2cBus& operator=(const I2cBus&) = delete;

  /// Write [reg, data...] in a single transfer; false on I/O error.
  bool write_block(std::uint8_t reg, std::initializer_list<std::uint8_t> data);

  /// Write the register byte, then read ``len`` bytes into ``out``.
  /// false on I/O error (including short reads).
  bool read_block(std::uint8_t reg, std::uint8_t* out, std::size_t len);

 private:
  int fd_ = -1;
  std::uint8_t addr_ = 0;
};

}  // namespace raspbot_base
