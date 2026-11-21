// ============================================================================
// VelocityLimiter —— /cmd_vel → /cmd_vel_safe 的纯逻辑限速器（C++ 重写版）
//
// 本类是原 Python 版 raspbot_fusion/limiter.py 的 1:1 语义复刻，供
// cmd_vel_limiter 节点与 gtest 离线测试共用。不依赖任何 ROS 头文件。
//
// 语义（与 Python 版逐条一致）：
//   1. 一阶斜率限幅：输出变化率 ≤ max_lin_accel / max_ang_accel
//      （FR-E-04：阶跃无突跳）；
//   2. 幅值夹紧：|vx|,|vy| ≤ max_linear（IMU 掉线时 ≤ degraded_max_linear），
//      |wz| ≤ max_angular；
//   3. estop 锁存：true 后输出恒零，直到显式 false（CONTRACTS §2 锁存语义）；
//   4. 输入超时看门狗（P0）：指令超过 input_timeout 未更新 → 输出瞬时归零。
//      防止 limiter 30 Hz 持续转发最后一条非零指令、使底盘 0.5 s 看门狗永不
//      触发（J-07/SW-F01：三级停车从最后有效指令起算 ≤0.5 s，input_timeout
//      =0.4 s 留出 mux→limiter→base 两级各约 33 ms 传播时间，总链 ≈0.47 s）；
//   5. dt 异常保护：dt ≤ 0 或 > 0.5（含 NaN）按 0 处理，状态不跳变、不抛异常
//      （SW-U02 精神）。
//
// 处理顺序：estop 锁存（恒零，最高优先级）→ 输入超时（瞬时归零）→ 斜率限幅
// → 幅值夹紧。安全归零（estop / 输入超时）为瞬时跳变，不受斜率限幅约束；斜率
// 限幅只作用于正常指令目标的变化（含超时/急停解除后的从零恢复加速）。
//
// 与 Python 版的唯一接口差异（类型映射，语义等价）：
//   - Python 的 None（imu_age / input_age 未收到数据）→ C++ 用 NaN 表示，
//     imu_stale(NaN) / input_stale(NaN) 均返回 true（与 None 一致）；
//   - Python 的非数值字符串（如 'x'）在 C++ 强类型接口下不存在，NaN 已覆盖
//     其全部行为路径（构造函数校验 / dt 保护 / 新鲜度判定）。
// ============================================================================

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace raspbot_fusion
{

/// 速度三元组（vx, vy, wz），单位 m/s 与 rad/s。
struct Vel
{
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
};

/// NaN：表示「从未收到数据」（对应 Python 版 imu_age/input_age 的 None）。
constexpr double kNever = std::numeric_limits<double>::quiet_NaN();

class VelocityLimiter
{
public:
  /// 全部参数与 Python 版默认值一致；任一参数非正数或非有限（NaN/inf）抛
  /// std::invalid_argument（对应 Python 的 ValueError）。
  VelocityLimiter(
    double max_linear = 0.30,
    double max_angular = 1.5,
    double max_lin_accel = 0.6,
    double max_ang_accel = 2.0,
    double degraded_max_linear = 0.15,
    double imu_timeout = 1.0,
    double input_timeout = 0.4);

  // 参数（只读，对应 Python 公开属性）
  const double max_linear;
  const double max_angular;
  const double max_lin_accel;
  const double max_ang_accel;
  const double degraded_max_linear;
  const double imu_timeout;
  const double input_timeout;

  bool estop_latched() const { return estop_latched_; }

  /// IMU 是否视为掉线：age > imu_timeout（NaN = 从未收到，视为掉线）。
  bool imu_stale(double imu_age_sec) const;

  /// 指令是否视为超时：age > input_timeout（NaN = 从未收到，视为超时）。
  bool input_stale(double input_age_sec) const;

  /// true：锁存（此后输出恒零，直到显式 false）；false：解除。
  /// 锁存时清零内部状态，解除后从零平滑起步。
  void estop(bool active);

  /// 执行一拍限幅，返回 (vx, vy, wz)。
  /// dt ≤ 0 或 > 0.5（含 NaN）按 0 处理。
  /// imu_age_sec / input_age_sec 传 kNever(NaN) 表示从未收到数据。
  Vel step(
    double target_vx, double target_vy, double target_wz, double dt,
    double imu_age_sec, double input_age_sec = kNever);

private:
  bool estop_latched_ = false;
  Vel prev_{0.0, 0.0, 0.0};  // 上一拍输出（限幅内部状态）
};

}  // namespace raspbot_fusion
