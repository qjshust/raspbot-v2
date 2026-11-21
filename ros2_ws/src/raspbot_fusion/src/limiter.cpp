// VelocityLimiter 实现 —— 纯标准库，无 ROS 依赖（离线可测）。

#include "raspbot_fusion/limiter.hpp"

#include <utility>

namespace raspbot_fusion
{

namespace
{

/// 参数校验：必须为正的有限数（NaN/inf/≤0 拒绝），对应 Python 版构造函数的
/// ValueError。
double validate_param(const char * name, double value)
{
  if (!(value > 0.0) || !std::isfinite(value)) {
    throw std::invalid_argument(
      std::string(name) + " must be a positive finite number, got " +
      std::to_string(value));
  }
  return value;
}

/// dt 保护：≤0 或 >0.5（含 NaN/inf）一律按 0 处理；dt=0.5 恰好合法。
/// 对应 Python _sanitize_dt：not (0.0 < dt <= 0.5) → 0。
double sanitize_dt(double dt)
{
  if (!(dt > 0.0 && dt <= 0.5)) {  // NaN/inf 在此判 false → 归 0
    return 0.0;
  }
  return dt;
}

/// 数值防护：NaN/inf 按 0 处理（安全方向：宁停勿乱）。对应 _finite_or_zero。
double finite_or_zero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

/// 对应 Python _clamp(value, low, high) = max(low, min(high, value))。
double clamp_value(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

}  // namespace

VelocityLimiter::VelocityLimiter(
  double max_linear, double max_angular, double max_lin_accel,
  double max_ang_accel, double degraded_max_linear, double imu_timeout,
  double input_timeout)
: max_linear(validate_param("max_linear", max_linear)),
  max_angular(validate_param("max_angular", max_angular)),
  max_lin_accel(validate_param("max_lin_accel", max_lin_accel)),
  max_ang_accel(validate_param("max_ang_accel", max_ang_accel)),
  degraded_max_linear(validate_param("degraded_max_linear", degraded_max_linear)),
  imu_timeout(validate_param("imu_timeout", imu_timeout)),
  input_timeout(validate_param("input_timeout", input_timeout))
{
}

bool VelocityLimiter::imu_stale(double imu_age_sec) const
{
  // Python: not (imu_age_sec <= imu_timeout)；NaN → true（= None → True）
  return !(imu_age_sec <= imu_timeout);
}

bool VelocityLimiter::input_stale(double input_age_sec) const
{
  // Python: not (input_age_sec <= input_timeout)；NaN → true（= None → True）
  return !(input_age_sec <= input_timeout);
}

void VelocityLimiter::estop(bool active)
{
  estop_latched_ = active;
  if (estop_latched_) {
    prev_ = Vel{0.0, 0.0, 0.0};  // 解除后从零平滑起步，无突跳
  }
}

Vel VelocityLimiter::step(
  double target_vx, double target_vy, double target_wz, double dt,
  double imu_age_sec, double input_age_sec)
{
  dt = sanitize_dt(dt);

  // 0) estop 锁存：恒零，优先级最高
  if (estop_latched_) {
    prev_ = Vel{0.0, 0.0, 0.0};
    return prev_;
  }

  // 1) 输入超时：安全归零必须瞬时（J-07/SW-F01：从最后有效指令起算 ≤0.5 s，
  //    0.4 s 超时触发后不允许再花时间减速），不受斜率限幅约束；清零内部状态，
  //    恢复指令后从零平滑加速。
  if (input_stale(input_age_sec)) {
    prev_ = Vel{0.0, 0.0, 0.0};
    return prev_;
  }

  const double tvx = finite_or_zero(target_vx);
  const double tvy = finite_or_zero(target_vy);
  const double twz = finite_or_zero(target_wz);

  // 2) 一阶斜率限幅（逐轴）
  const double max_dv = max_lin_accel * dt;
  const double max_dw = max_ang_accel * dt;
  Vel out;
  out.vx = prev_.vx + clamp_value(tvx - prev_.vx, -max_dv, max_dv);
  out.vy = prev_.vy + clamp_value(tvy - prev_.vy, -max_dv, max_dv);
  out.wz = prev_.wz + clamp_value(twz - prev_.wz, -max_dw, max_dw);

  // 3) 幅值夹紧（IMU 掉线 → 线性上限降为 degraded_max_linear）
  const double vmax = imu_stale(imu_age_sec) ? degraded_max_linear : max_linear;
  out.vx = clamp_value(out.vx, -vmax, vmax);
  out.vy = clamp_value(out.vy, -vmax, vmax);
  out.wz = clamp_value(out.wz, -max_angular, max_angular);

  prev_ = out;
  return out;
}

}  // namespace raspbot_fusion
