#include "raspbot_imu/config.hpp"

#include <cctype>
#include <stdexcept>

namespace raspbot_imu {
namespace {

std::string normalize(const std::string& s) {
  std::size_t begin = 0;
  std::size_t end = s.size();
  while (begin < end &&
         std::isspace(static_cast<unsigned char>(s[begin])) != 0) {
    ++begin;
  }
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(s[end - 1])) != 0) {
    --end;
  }
  std::string out = s.substr(begin, end - begin);
  for (char& c : out) {
    c = static_cast<char>(
        std::tolower(static_cast<unsigned char>(c)));
  }
  return out;
}

}  // namespace

ImuConfig validate_config(const std::string& mode,
                          const std::string& angle_source,
                          const std::string& source) {
  const std::string m = normalize(mode);
  if (m != "yaw" && m != "mahony") {
    throw std::invalid_argument("mode 非法：可选值 yaw / mahony，收到 '" +
                                mode + "'");
  }

  const std::string a = normalize(angle_source);
  if (a != "device" && a != "gyro") {
    throw std::invalid_argument(
        "angle_source 非法：可选值 device / gyro，收到 '" + angle_source +
        "'");
  }

  const std::string s = normalize(source);
  if (s != "mock" && s != "witmotion" && s != "icm42688") {
    throw std::invalid_argument(
        "source 非法：可选值 mock / witmotion / icm42688，收到 '" + source +
        "'");
  }

  return {m, a, s};
}

}  // namespace raspbot_imu
