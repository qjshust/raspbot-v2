#include "raspbot_imu/msgs.hpp"

#include <stdexcept>
#include <string>

namespace raspbot_imu {
namespace {

std::array<double, 3> as_vec3(const std::vector<double>& v, const char* name) {
  if (v.size() != 3) {
    throw std::invalid_argument(std::string(name) + " 应为 3 元向量，收到 " +
                                std::to_string(v.size()) + " 元");
  }
  return {v[0], v[1], v[2]};
}

}  // namespace

ImuData build_imu_data(double now_sec, const std::string& frame_id,
                       const std::vector<double>& quat,
                       const std::vector<double>& gyro,
                       const std::vector<double>& accel,
                       double yaw_var, double tilt_var,
                       double gyro_var, double accel_var) {
  if (quat.size() != 4) {
    throw std::invalid_argument("quat 应为 4 元四元数 [w,x,y,z]，收到 " +
                                std::to_string(quat.size()) + " 元");
  }

  ImuData d;
  d.stamp_sec = now_sec;
  d.frame_id = frame_id;
  d.orientation = {quat[0], quat[1], quat[2], quat[3]};
  d.angular_velocity = as_vec3(gyro, "gyro");
  d.linear_acceleration = as_vec3(accel, "accel");

  const double t = tilt_var;
  const double y = yaw_var;
  const double g = gyro_var;
  const double a = accel_var;
  d.orientation_covariance = {
      t, 0.0, 0.0,
      0.0, t, 0.0,
      0.0, 0.0, y,
  };
  d.angular_velocity_covariance = {
      g, 0.0, 0.0,
      0.0, g, 0.0,
      0.0, 0.0, g,
  };
  d.linear_acceleration_covariance = {
      a, 0.0, 0.0,
      0.0, a, 0.0,
      0.0, 0.0, a,
  };
  return d;
}

}  // namespace raspbot_imu
