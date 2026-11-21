#pragma once

#include <memory>
#include <string>

#include "raspbot_imu/types.hpp"

namespace raspbot_imu {

/// IMU 数据源统一接口（对应 Python 版 sources.ImuSource）。
class ImuSource {
 public:
  virtual ~ImuSource() = default;

  /// 非阻塞读取一帧；暂无数据返回 std::nullopt，异常时抛 std::exception。
  virtual std::optional<ImuSample> read() = 0;

  /// 是否提供加速度计数据。
  virtual bool has_accel() const { return true; }

  /// 释放底层资源。
  virtual void close() {}
};

/// 数据源构造参数（与 Python 版节点参数同名同默认值）。
struct SourceOptions {
  double rate_hz = 100.0;
  std::string port = "/dev/raspbot_imu";
  int baud = 115200;
  int spi_bus = 0;
  int spi_device = 0;
  int spi_speed_hz = 2000000;
  double gyro_fs_dps = 500.0;
  double accel_fs_g = 4.0;
  double odr_hz = 1000.0;
};

/// 按名称构造数据源：mock / witmotion / icm42688；
/// 未知名称抛 std::invalid_argument（信息含可选值）。
std::unique_ptr<ImuSource> make_source(const std::string& kind,
                                       const SourceOptions& opts);

}  // namespace raspbot_imu
