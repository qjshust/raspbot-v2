#include "raspbot_base/i2c_bus.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace raspbot_base {

I2cBus::I2cBus(const char* path, std::uint8_t addr) : addr_(addr) {
  fd_ = ::open(path, O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error(std::string("cannot open ") + path + ": " +
                             std::strerror(errno));
  }
  if (::ioctl(fd_, I2C_SLAVE, addr_) < 0) {
    const std::string msg =
        std::string("cannot select I2C slave 0x") +
        std::to_string(static_cast<int>(addr_)) + " on " + path + ": " +
        std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error(msg);
  }
}

I2cBus::~I2cBus() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool I2cBus::write_block(std::uint8_t reg,
                         std::initializer_list<std::uint8_t> data) {
  std::vector<std::uint8_t> buf;
  buf.reserve(data.size() + 1);
  buf.push_back(reg);
  buf.insert(buf.end(), data.begin(), data.end());
  const ssize_t n = ::write(fd_, buf.data(), buf.size());
  return n == static_cast<ssize_t>(buf.size());
}

bool I2cBus::read_block(std::uint8_t reg, std::uint8_t* out, std::size_t len) {
  if (::write(fd_, &reg, 1) != 1) {
    return false;
  }
  const ssize_t n = ::read(fd_, out, len);
  return n == static_cast<ssize_t>(len);
}

}  // namespace raspbot_base
