// Copyright (c) 2026 RASPBOT-V2 WP-H
// kinematics: 纯运动学/里程计函数，无 ROS 依赖，离线可测。
//
// 兼容接口：与 raspbot_base 的纯函数口径一一对应
// （legacy_python/raspbot_base/kinematics.py 与 odometry.py）。
// A 交付 raspbot_base C++ kinematics 库后，本模块由 lead 统一替换为
// #include <raspbot_base/...> 的调用——函数签名保持不变。

#ifndef RASPBOT_MOCK__KINEMATICS_HPP_
#define RASPBOT_MOCK__KINEMATICS_HPP_

namespace raspbot_mock
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct Quat
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

/// dt <= 0（时钟回跳）或 dt > 1.0 s（长停顿）→ 0.0，防止位姿跳变；否则原值。
double sanitize_dt(double dt);

/// 半隐式欧拉一步：先 theta += wz*dt，再用新 theta 更新 x/y
/// （与 raspbot_base 的 integrate_pose 顺序完全一致）。
Pose2D integrate_pose(const Pose2D & pose, double vx, double vy, double wz, double dt);

/// 平面 yaw → 单位四元数（x = y = 0）。
Quat yaw_to_quat(double theta);

}  // namespace raspbot_mock

#endif  // RASPBOT_MOCK__KINEMATICS_HPP_
