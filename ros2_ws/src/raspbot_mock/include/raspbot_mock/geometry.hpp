// Copyright (c) 2026 RASPBOT-V2 WP-H
// geometry: 纯几何函数（射线-矩形求交），无 ROS 依赖，离线可测。
//
// 与 legacy_python/raspbot_mock/geometry.py 口径一致。

#ifndef RASPBOT_MOCK__GEOMETRY_HPP_
#define RASPBOT_MOCK__GEOMETRY_HPP_

#include <optional>

namespace raspbot_mock
{

/// 原点沿 angle 方向的射线与 [-half_x, half_x] x [-half_y, half_y] 矩形的
/// 交点距离（slab 方法）；无交点返回 std::nullopt。
/// 车位于 (0,0)（矩形内部）时每个方向都会命中。
std::optional<double> ray_rect_distance(double angle, double half_x, double half_y);

}  // namespace raspbot_mock

#endif  // RASPBOT_MOCK__GEOMETRY_HPP_
