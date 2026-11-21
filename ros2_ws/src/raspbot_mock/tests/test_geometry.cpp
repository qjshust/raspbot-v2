// Copyright (c) 2026 RASPBOT-V2 WP-H
// 1:1 映射 legacy Python 用例 tests/test_geometry.py（5 用例）。
// 矩形以原点为中心，射线从原点出发 → 必然命中；验证距离解析式与对称性。

#include <cmath>
#include <gtest/gtest.h>

#include "raspbot_mock/geometry.hpp"

using raspbot_mock::ray_rect_distance;

namespace
{
constexpr double HX = 2.5;  // 5 x 4 m 房间的半宽
constexpr double HY = 2.0;
// -std=c++17 下 glibc 隐藏 M_PI
constexpr double kPi = 3.14159265358979323846;
}  // namespace

// test_axis_hits
TEST(GeometryTest, AxisHits)
{
  EXPECT_NEAR(ray_rect_distance(0.0, HX, HY).value(), 2.5, 1e-9);
  EXPECT_NEAR(ray_rect_distance(kPi, HX, HY).value(), 2.5, 1e-9);
  EXPECT_NEAR(ray_rect_distance(kPi / 2.0, HX, HY).value(), 2.0, 1e-9);
  EXPECT_NEAR(ray_rect_distance(-kPi / 2.0, HX, HY).value(), 2.0, 1e-9);
}

// test_first_quadrant_is_min_of_two_walls
TEST(GeometryTest, FirstQuadrantIsMinOfTwoWalls)
{
  // 第一象限：x 墙距离 HX/cos(theta)，y 墙距离 HY/sin(theta)，取先撞到的
  for (int deg = 1; deg < 90; ++deg) {
    const double theta = deg * kPi / 180.0;
    const double expected = std::min(HX / std::cos(theta), HY / std::sin(theta));
    EXPECT_NEAR(ray_rect_distance(theta, HX, HY).value(), expected, 1e-9);
  }
}

// test_opposite_rays_symmetric
TEST(GeometryTest, OppositeRaysSymmetric)
{
  for (int i = 0; i < 720; ++i) {
    const double angle = -kPi + i * (2.0 * kPi / 720.0);
    const double d1 = ray_rect_distance(angle, HX, HY).value();
    const double d2 = ray_rect_distance(angle + kPi, HX, HY).value();
    EXPECT_NEAR(d1, d2, 1e-9);
  }
}

// test_all_720_rays_hit_within_bounds
TEST(GeometryTest, All720RaysHitWithinBounds)
{
  for (int i = 0; i < 720; ++i) {
    const double angle = -kPi + i * (2.0 * kPi / 720.0);
    const auto d = ray_rect_distance(angle, HX, HY);
    ASSERT_TRUE(d.has_value());
    EXPECT_GT(*d, 0.0);
    EXPECT_LE(*d, std::hypot(HX, HY));
  }
}

// test_corner_consistency
TEST(GeometryTest, CornerConsistency)
{
  // 45° 方向两墙距离相等（4 m 房间对角方向）
  const double half = 2.0;
  const double d = ray_rect_distance(kPi / 4.0, half, half).value();
  EXPECT_NEAR(d, half / std::cos(kPi / 4.0), 1e-9);
  EXPECT_NEAR(d, half / std::sin(kPi / 4.0), 1e-9);
}
