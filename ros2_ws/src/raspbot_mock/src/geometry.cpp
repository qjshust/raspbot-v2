// Copyright (c) 2026 RASPBOT-V2 WP-H

#include "raspbot_mock/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace raspbot_mock
{

std::optional<double> ray_rect_distance(double angle, double half_x, double half_y)
{
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double tmin = -std::numeric_limits<double>::infinity();
  double tmax = std::numeric_limits<double>::infinity();

  struct Slab
  {
    double origin;
    double direction;
    double half;
  };
  const Slab slabs[2] = {{0.0, dx, half_x}, {0.0, dy, half_y}};
  for (const Slab & s : slabs) {
    if (std::abs(s.direction) < 1e-12) {
      // 与轴平行：原点必须在 slab 内，否则整条射线无交点
      if (s.origin < -s.half || s.origin > s.half) {
        return std::nullopt;
      }
    } else {
      double t1 = (-s.half - s.origin) / s.direction;
      double t2 = (s.half - s.origin) / s.direction;
      if (t1 > t2) {
        std::swap(t1, t2);
      }
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
    }
  }
  if (tmax < tmin || tmax < 0.0) {
    return std::nullopt;
  }
  if (tmin < 0.0) {
    tmin = tmax;  // 原点在矩形内部且沿反方向先穿出
  }
  return tmin;
}

}  // namespace raspbot_mock
