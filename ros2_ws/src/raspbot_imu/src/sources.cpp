#include "raspbot_imu/sources.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "raspbot_imu/icm42688.hpp"
#include "raspbot_imu/mock_source.hpp"
#include "raspbot_imu/witmotion.hpp"

namespace raspbot_imu {
namespace {

double monotonic_seconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string lower_trim(std::string s) {
  std::size_t begin = 0;
  std::size_t end = s.size();
  while (begin < end && (s[begin] == ' ' || s[begin] == '\t')) {
    ++begin;
  }
  while (end > begin && (s[end - 1] == ' ' || s[end - 1] == '\t')) {
    --end;
  }
  s = s.substr(begin, end - begin);
  for (char& c : s) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  return s;
}

speed_t baud_to_speed(int baud) {
  switch (baud) {
    case 50: return B50;
    case 75: return B75;
    case 110: return B110;
    case 134: return B134;
    case 150: return B150;
    case 200: return B200;
    case 300: return B300;
    case 600: return B600;
    case 1200: return B1200;
    case 1800: return B1800;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    default:
      throw std::invalid_argument("不支持的波特率 " + std::to_string(baud));
  }
}

int serial_open(const std::string& port, int baud) {
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    throw std::runtime_error("打不开串口 " + port + "：" +
                             std::strerror(errno));
  }
  struct termios tio {};
  if (::tcgetattr(fd, &tio) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("tcgetattr 失败 " + port + "：" + err);
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB | CRTSCTS);
  tio.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
  tio.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ECHOE | ISIG);
  tio.c_oflag &= ~static_cast<tcflag_t>(OPOST);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  const speed_t speed = baud_to_speed(baud);
  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);
  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("tcsetattr 失败 " + port + "：" + err);
  }
  return fd;
}

}  // namespace

// --- WitMotion 串口源（termios，对应 Python 版 WitMotionSource） ---

class WitMotionSource final : public ImuSource {
 public:
  WitMotionSource(const std::string& port, int baud,
                  double accel_fs_g = 16.0, double gyro_fs_dps = 2000.0)
      : port_(port), baud_(baud), parser_(accel_fs_g, gyro_fs_dps) {
    fd_ = serial_open(port_, baud_);
  }

  ~WitMotionSource() override { close(); }

  bool has_accel() const override { return parser_.seen_accel; }

  std::optional<ImuSample> read() override {
    if (fd_ < 0) {
      // 断线后按 1 s 节流尝试重开；仍打不开则抛异常（节点计数并限频告警），
      // 重插成功即自动恢复发布。
      const auto now = std::chrono::steady_clock::now();
      if (now - last_open_attempt_ < std::chrono::seconds(1)) {
        throw std::runtime_error("串口未打开（" + port_ + "），等待重连");
      }
      last_open_attempt_ = now;
      fd_ = serial_open(port_, baud_);  // 失败会抛出，计入节点错误计数
    }

    std::uint8_t buf[256];
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return std::nullopt;  // 暂无数据
      }
      const std::string err = std::strerror(errno);
      close();
      throw std::runtime_error("串口读取失败：" + err);
    }
    if (n == 0) {
      return std::nullopt;
    }
    if (!parser_.feed(buf, static_cast<std::size_t>(n))) {
      return std::nullopt;
    }
    const Vec3 accel = parser_.seen_accel ? parser_.accel : Vec3{0.0, 0.0, 0.0};
    std::optional<double> yaw;
    if (parser_.seen_angle) {
      yaw = parser_.angle_deg[2];
    }
    return ImuSample(monotonic_seconds(), parser_.gyro, accel, yaw);
  }

  void close() override {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
  std::string port_;
  int baud_;
  int fd_ = -1;
  WitMotionParser parser_;
  std::chrono::steady_clock::time_point last_open_attempt_{};
};

// --- ICM-42688-P SPI 设备（spidev ioctl，对应 Python 版 icm42688.ICM42688） ---

namespace {

constexpr std::uint8_t kRegDeviceConfig = 0x11;
constexpr std::uint8_t kRegIntConfig = 0x14;
constexpr std::uint8_t kRegTempData = 0x1D;
constexpr std::uint8_t kRegIntStatus = 0x2D;
constexpr std::uint8_t kRegPwrMgmt0 = 0x4E;
constexpr std::uint8_t kRegGyroConfig0 = 0x4F;
constexpr std::uint8_t kRegAccelConfig0 = 0x50;
constexpr std::uint8_t kRegGyroAccelConfig0 = 0x52;
constexpr std::uint8_t kRegIntConfig1 = 0x64;
constexpr std::uint8_t kRegIntSource0 = 0x65;
constexpr std::uint8_t kRegWhoAmI = 0x75;
constexpr std::uint8_t kWhoAmI42688 = 0x47;
constexpr std::uint8_t kPwrMgmt0GyroLnAccelLn = 0x0F;

}  // namespace

class Icm42688Device {
 public:
  Icm42688Device(int bus, int device, int speed_hz, double gyro_fs_dps,
                 double accel_fs_g, double odr_hz)
      : bus_(bus),
        device_(device),
        speed_hz_(speed_hz),
        gyro_fs_dps_(gyro_fs_dps),
        accel_fs_g_(accel_fs_g),
        odr_hz_(odr_hz) {}

  ~Icm42688Device() { close(); }

  void open() {
    const std::string path =
        "/dev/spidev" + std::to_string(bus_) + "." + std::to_string(device_);
    fd_ = ::open(path.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error("打不开 SPI 设备 " + path + "：" +
                               std::strerror(errno));
    }
    std::uint8_t mode = 3;
    if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) {
      throw std::runtime_error("SPI 设置模式失败：" +
                               std::string(std::strerror(errno)));
    }
    std::uint32_t speed = static_cast<std::uint32_t>(speed_hz_);
    if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
      throw std::runtime_error("SPI 设置速率失败：" +
                               std::string(std::strerror(errno)));
    }
    reset();
    configure();
  }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  ImuSample read() {
    std::vector<std::uint8_t> buf = read_burst(kRegTempData, 14);
    auto result = parse_burst(buf.data(), buf.size(), gyro_fs_dps_, accel_fs_g_);
    const std::int16_t temp =
        static_cast<std::int16_t>((buf[0] << 8) | buf[1]);
    last_temperature_c = static_cast<double>(temp) / 132.48 + 25.0;
    return ImuSample(monotonic_seconds(), result.first, result.second);
  }

  std::optional<double> last_temperature_c;

 private:
  void transfer(const std::vector<std::uint8_t>& tx,
                std::vector<std::uint8_t>& rx) {
    struct spi_ioc_transfer tr {};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    tr.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    tr.len = static_cast<std::uint32_t>(tx.size());
    tr.speed_hz = static_cast<std::uint32_t>(speed_hz_);
    tr.bits_per_word = 8;
    if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
      throw std::runtime_error("SPI 传输失败：" +
                               std::string(std::strerror(errno)));
    }
  }

  void write_reg(std::uint8_t reg, std::uint8_t value) {
    std::vector<std::uint8_t> tx{static_cast<std::uint8_t>(reg & 0x7F),
                                 value};
    std::vector<std::uint8_t> rx(2, 0);
    transfer(tx, rx);
  }

  std::uint8_t read_reg(std::uint8_t reg) {
    std::vector<std::uint8_t> tx{
        static_cast<std::uint8_t>(0x80 | (reg & 0x7F)), 0x00};
    std::vector<std::uint8_t> rx(2, 0);
    transfer(tx, rx);
    return rx[1];
  }

  std::vector<std::uint8_t> read_burst(std::uint8_t reg, std::size_t length) {
    std::vector<std::uint8_t> tx(length + 1, 0x00);
    tx[0] = static_cast<std::uint8_t>(0x80 | (reg & 0x7F));
    std::vector<std::uint8_t> rx(length + 1, 0x00);
    transfer(tx, rx);
    return std::vector<std::uint8_t>(rx.begin() + 1, rx.end());
  }

  void reset() {
    write_reg(kRegDeviceConfig, 0x01);
    ::usleep(100000);  // 软复位后等待电源稳定
  }

  void configure() {
    if (read_reg(kRegWhoAmI) != kWhoAmI42688) {
      throw std::runtime_error(
          "WHO_AM_I 不匹配，未检测到 ICM-42688-P（检查 SPI 接线/片选/供电）");
    }
    write_reg(kRegPwrMgmt0, kPwrMgmt0GyroLnAccelLn);
    ::usleep(80000);  // 陀螺启动 60 ms + 加速度 20 ms

    const int gyro_code = nearest_code(kGyroFsCodes, gyro_fs_dps_, "gyro_fs_dps");
    const int accel_code = nearest_code(kAccelFsCodes, accel_fs_g_, "accel_fs_g");
    const int odr_code = nearest_code(kOdrCodes, odr_hz_, "odr_hz");

    write_reg(kRegGyroConfig0,
              static_cast<std::uint8_t>((gyro_code << 5) | odr_code));
    write_reg(kRegAccelConfig0,
              static_cast<std::uint8_t>((accel_code << 5) | odr_code));
    write_reg(kRegGyroAccelConfig0, 0x00);  // 滤波器带宽 = ODR/2
    write_reg(kRegIntConfig, 0x03);         // 推挽、高有效
    write_reg(kRegIntSource0, 0x08);        // UI 数据就绪 → INT1
    write_reg(kRegIntConfig1, 0x00);
    ::usleep(50000);
  }

  int bus_;
  int device_;
  int speed_hz_;
  double gyro_fs_dps_;
  double accel_fs_g_;
  double odr_hz_;
  int fd_ = -1;
};

/// 把 ICM42688 驱动包装成 ImuSource；任何异常 → nullopt（对应 Python _SpiSource）。
class SpiSource final : public ImuSource {
 public:
  explicit SpiSource(std::unique_ptr<Icm42688Device> driver)
      : driver_(std::move(driver)) {}

  std::optional<ImuSample> read() override {
    try {
      return driver_->read();
    } catch (...) {
      return std::nullopt;  // 单帧无效时跳过，等待下一帧
    }
  }

  void close() override { driver_->close(); }

 private:
  std::unique_ptr<Icm42688Device> driver_;
};

// --- 工厂 ---

std::unique_ptr<ImuSource> make_source(const std::string& kind_in,
                                       const SourceOptions& opts) {
  const std::string kind = lower_trim(kind_in);
  if (kind == "mock") {
    return std::make_unique<MockSource>(opts.rate_hz);
  }
  if (kind == "witmotion") {
    return std::make_unique<WitMotionSource>(opts.port, opts.baud);
  }
  if (kind == "icm42688") {
    auto driver = std::make_unique<Icm42688Device>(
        opts.spi_bus, opts.spi_device, opts.spi_speed_hz, opts.gyro_fs_dps,
        opts.accel_fs_g, opts.odr_hz);
    driver->open();
    return std::make_unique<SpiSource>(std::move(driver));
  }
  throw std::invalid_argument("未知数据源 '" + kind_in +
                              "'，可选：mock / witmotion / icm42688");
}

}  // namespace raspbot_imu
