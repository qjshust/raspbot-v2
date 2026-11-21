#pragma once

#include <utility>

#include "raspbot_imu/types.hpp"

namespace raspbot_imu {

/// 归一化四元数；零四元数退化为单位旋转。
Quat quat_normalize(const Quat& q);

/// 四元数乘法 a ⊗ b（先施加 b 再施加 a）。
Quat quat_multiply(const Quat& a, const Quat& b);

/// body -> world 旋转矩阵（行主序 3x3）。
Mat33 quat_to_matrix(const Quat& q);

/// 矩阵乘向量 R @ v。
Vec3 mat_vec(const Mat33& m, const Vec3& v);

/// 转置矩阵乘向量 R^T @ v。
Vec3 mat_t_vec(const Mat33& m, const Vec3& v);

/// 绕世界 Z 轴旋转 yaw 的四元数。
Quat quat_from_yaw(double yaw);

/// 提取偏航角，范围 (-pi, pi]。
double quat_yaw(const Quat& q);

/// 提取横滚、俯仰角（ZYX 欧拉序），roll ∈ (-pi, pi]，pitch ∈ [-pi/2, pi/2]。
std::pair<double, double> quat_roll_pitch(const Quat& q);

/// 把角度折回 (-pi, pi]。
double wrap_pi(double angle);

}  // namespace raspbot_imu
