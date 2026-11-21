/// RASPBOT-V2 IMU 节点（C++ 重写，行为对齐 Python 版 legacy_python/raspbot_imu/node.py）。
///
/// 发布 ``/imu/data``（sensor_msgs/Imu，SensorData QoS、depth 10），默认 100 Hz，
/// frame_id ``imu_link``。数据源：mock / witmotion（termios 串口）/ icm42688（SPI）。
/// 参数名与默认值与 Python 版一致，见 config/imu.yaml。

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "raspbot_imu/attitude.hpp"
#include "raspbot_imu/config.hpp"
#include "raspbot_imu/msgs.hpp"
#include "raspbot_imu/quaternion.hpp"
#include "raspbot_imu/sources.hpp"

namespace raspbot_imu {

class ImuNode : public rclcpp::Node {
 public:
  explicit ImuNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("imu_node", options) {
    // --- 参数（同名同默认值与 Python 版一致） ---
    declare_parameter<std::string>("source", "icm42688");
    declare_parameter<double>("rate_hz", 100.0);
    declare_parameter<std::string>("mode", "yaw");
    declare_parameter<std::string>("frame_id", "imu_link");
    declare_parameter<double>("calibrate_seconds", 5.0);
    declare_parameter<double>("yaw_offset_deg", 0.0);
    declare_parameter<std::string>("angle_source", "gyro");
    declare_parameter<std::string>("serial_port", "/dev/raspbot_imu");
    declare_parameter<int64_t>("baud", 115200);
    declare_parameter<int64_t>("spi_bus", 0);
    declare_parameter<int64_t>("spi_device", 0);
    declare_parameter<int64_t>("spi_speed_hz", 2000000);
    declare_parameter<double>("gyro_fs_dps", 500.0);
    declare_parameter<double>("accel_fs_g", 4.0);
    declare_parameter<double>("odr_hz", 1000.0);
    declare_parameter<double>("mahony_kp", 0.8);
    declare_parameter<double>("mahony_ki", 0.02);
    declare_parameter<double>("orientation_yaw_variance", 0.0025);
    declare_parameter<double>("orientation_tilt_variance", 1.0);
    declare_parameter<double>("angular_velocity_variance", 0.0004);
    declare_parameter<double>("linear_acceleration_variance", 0.04);

    rate_hz_ = get_parameter("rate_hz").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    yaw_offset_ = get_parameter("yaw_offset_deg").as_double() * kPi / 180.0;
    calibrate_seconds_ = get_parameter("calibrate_seconds").as_double();
    tilt_var_ = get_parameter("orientation_tilt_variance").as_double();
    yaw_var_ = get_parameter("orientation_yaw_variance").as_double();
    gyro_var_ = get_parameter("angular_velocity_variance").as_double();
    accel_var_ = get_parameter("linear_acceleration_variance").as_double();

    // 读取异常统计与限频告警状态；必须先于 calibrate 使用
    last_error_report_ = std::chrono::steady_clock::now();
    last_report_ = std::chrono::steady_clock::now();

    // 参数校验（SW-U15 / SW-G04）：非法值抛异常、信息含可选值
    const ImuConfig cfg =
        validate_config(get_parameter("mode").as_string(),
                        get_parameter("angle_source").as_string(),
                        get_parameter("source").as_string());
    mode_ = cfg.mode;
    angle_source_ = cfg.angle_source;
    source_kind_ = cfg.source;

    if (angle_source_ == "device" && mode_ == "mahony") {
      RCLCPP_WARN(get_logger(),
                  "mode=mahony 会忽略器件角度（只用角速度+加速度），"
                  "要使用 HWT101 的解算角度请设 mode=yaw");
    }

    source_ = make_source(source_kind_, make_options());

    const std::optional<Vec3> bias = calibrate(calibrate_seconds_);

    if (mode_ == "mahony") {
      mahony_ = std::make_unique<MahonyAHRS>(
          get_parameter("mahony_kp").as_double(),
          get_parameter("mahony_ki").as_double());
      if (bias) {
        mahony_->bias = *bias;
      }
    } else if (angle_source_ == "gyro") {
      yaw_est_ = std::make_unique<YawEstimator>(bias);
    } else {
      // 器件角度：不需要我们自己积分，也不需要陀螺零偏
      RCLCPP_INFO(get_logger(),
                  "航向直接取自 HWT101 的解算角度（angle_source=device）");
    }

    // SensorData QoS + depth 10（契约 §2：SensorData；J-13：depth 10）
    pub_ = create_publisher<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS().keep_last(10));

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_),
                               [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "IMU 节点启动：source=%s mode=%s rate=%.0f Hz frame=%s",
                source_kind_.c_str(), mode_.c_str(), rate_hz_,
                frame_id_.c_str());
  }

  ~ImuNode() override {
    if (source_) {
      try {
        source_->close();
      } catch (...) {
        // 关闭失败不应影响退出
      }
    }
  }

 private:
  SourceOptions make_options() const {
    SourceOptions opts;
    opts.rate_hz = rate_hz_;
    opts.port = get_parameter("serial_port").as_string();
    opts.baud = static_cast<int>(get_parameter("baud").as_int());
    opts.spi_bus = static_cast<int>(get_parameter("spi_bus").as_int());
    opts.spi_device = static_cast<int>(get_parameter("spi_device").as_int());
    opts.spi_speed_hz =
        static_cast<int>(get_parameter("spi_speed_hz").as_int());
    opts.gyro_fs_dps = get_parameter("gyro_fs_dps").as_double();
    opts.accel_fs_g = get_parameter("accel_fs_g").as_double();
    opts.odr_hz = get_parameter("odr_hz").as_double();
    return opts;
  }

  /// 静止采样估计陀螺零偏；失败返回 nullopt 并继续用 0 零偏。
  std::optional<Vec3> calibrate(double seconds) {
    if (seconds <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "已跳过零偏标定（calibrate_seconds<=0），航向可能漂移");
      return std::nullopt;
    }
    RCLCPP_INFO(get_logger(), "零偏标定：保持车辆静止 %.1f s ...", seconds);
    std::vector<Vec3> samples;
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      try {
        const auto sample = source_->read();
        if (!sample) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        samples.push_back(sample->gyro);
      } catch (const std::exception& exc) {
        handle_source_error("标定读取", exc);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
    try {
      const Vec3 bias = estimate_gyro_bias(samples);
      RCLCPP_INFO(get_logger(),
                  "零偏估计 (rad/s)：x=%+.5f y=%+.5f z=%+.5f （%zu 帧）",
                  bias[0], bias[1], bias[2], samples.size());
      return bias;
    } catch (const std::invalid_argument& exc) {
      RCLCPP_WARN(get_logger(), "%s，本次使用 0 零偏", exc.what());
      return std::nullopt;
    }
  }

  /// 数据源异常统一处理：计数 + 限频 WARN（≤1 次/5 s），不发假数据。
  void handle_source_error(const char* where, const std::exception& exc) {
    errors_++;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_error_report_ >= std::chrono::seconds(5)) {
      last_error_report_ = now;
      RCLCPP_WARN(get_logger(),
                  "IMU 数据源异常（%s）：%s；累计 %zu 次。"
                  "停止发布 /imu/data，源恢复后自动继续",
                  where, exc.what(), errors_);
    }
  }

  void tick() {
    ImuSample sample;
    try {
      const auto read = source_->read();
      if (!read) {
        misses_++;
        report();
        return;
      }
      sample = *read;
    } catch (const std::exception& exc) {
      // D-05：异常不退出、不发假值；恢复后自动继续
      last_t_.reset();
      handle_source_error("读取", exc);
      return;
    }

    try {
      double dt =
          last_t_ ? sample.t - *last_t_ : 1.0 / rate_hz_;
      if (dt <= 1e-6 || dt > 0.2) {
        RCLCPP_WARN(get_logger(),
                    "采样间隔异常 dt=%.4f s，按标称周期处理", dt);
        dt = 1.0 / rate_hz_;
      }

      Quat quat;
      if (mode_ == "mahony" && mahony_) {
        quat = mahony_->update(sample.gyro, sample.accel, dt);
      } else if (angle_source_ == "device" && sample.yaw_deg) {
        // 器件已经做过卡尔曼融合，直接用它的航向，不做二次积分
        quat = quat_from_yaw(*sample.yaw_deg * kPi / 180.0);
      } else {
        if (!yaw_est_) {
          yaw_est_ = std::make_unique<YawEstimator>();
        }
        quat = quat_from_yaw(
            yaw_est_->update(sample.gyro, sample.accel, dt));
      }
      if (yaw_offset_ != 0.0) {
        quat = quat_multiply(quat_from_yaw(yaw_offset_), quat);
      }

      const auto now = get_clock()->now();
      const ImuData data = build_imu_data(
          static_cast<double>(now.nanoseconds()) * 1e-9, frame_id_,
          {quat[0], quat[1], quat[2], quat[3]},
          {sample.gyro[0], sample.gyro[1], sample.gyro[2]},
          {sample.accel[0], sample.accel[1], sample.accel[2]},
          yaw_var_, tilt_var_, gyro_var_, accel_var_);

      sensor_msgs::msg::Imu msg;
      msg.header.stamp = now;
      msg.header.frame_id = data.frame_id;
      msg.orientation.w = data.orientation[0];
      msg.orientation.x = data.orientation[1];
      msg.orientation.y = data.orientation[2];
      msg.orientation.z = data.orientation[3];
      msg.angular_velocity.x = data.angular_velocity[0];
      msg.angular_velocity.y = data.angular_velocity[1];
      msg.angular_velocity.z = data.angular_velocity[2];
      msg.linear_acceleration.x = data.linear_acceleration[0];
      msg.linear_acceleration.y = data.linear_acceleration[1];
      msg.linear_acceleration.z = data.linear_acceleration[2];
      msg.orientation_covariance = data.orientation_covariance;
      msg.angular_velocity_covariance = data.angular_velocity_covariance;
      msg.linear_acceleration_covariance = data.linear_acceleration_covariance;
      pub_->publish(msg);
    } catch (const std::exception& exc) {
      last_t_.reset();
      handle_source_error("处理/发布", exc);
      return;
    }

    last_t_ = sample.t;
    frames_++;
    report();
  }

  void report() {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration<double>(now - last_report_).count();
    if (elapsed < 5.0) {
      return;
    }
    const double measured = elapsed > 0.0 ? frames_ / elapsed : 0.0;
    RCLCPP_INFO(get_logger(),
                "发布 %.1f Hz（目标 %.0f Hz），空读 %zu 次，读取异常累计 %zu 次",
                measured, rate_hz_, misses_, errors_);
    frames_ = 0;
    misses_ = 0;
    last_report_ = now;
  }

  double rate_hz_ = 100.0;
  std::string frame_id_ = "imu_link";
  double yaw_offset_ = 0.0;
  double calibrate_seconds_ = 5.0;
  double tilt_var_ = 1.0;
  double yaw_var_ = 0.0025;
  double gyro_var_ = 0.0004;
  double accel_var_ = 0.04;
  std::string mode_ = "yaw";
  std::string angle_source_ = "gyro";
  std::string source_kind_ = "icm42688";

  std::unique_ptr<ImuSource> source_;
  std::unique_ptr<MahonyAHRS> mahony_;
  std::unique_ptr<YawEstimator> yaw_est_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<double> last_t_;
  std::size_t misses_ = 0;
  std::size_t frames_ = 0;
  std::size_t errors_ = 0;
  std::chrono::steady_clock::time_point last_report_{};
  std::chrono::steady_clock::time_point last_error_report_{};
};

}  // namespace raspbot_imu

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<raspbot_imu::ImuNode>();
    rclcpp::spin(node);
  } catch (const std::exception& exc) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_node"), "节点启动失败：%s",
                 exc.what());
    rc = 1;
  }
  // SIGINT/SIGTERM 由 rclcpp 信号处理接管：spin 正常返回 → 退出码 0
  // （对应 Python 版捕获 ExternalShutdownException / KeyboardInterrupt）。
  rclcpp::shutdown();
  return rc;
}
