"""raspbot_mock.geometry 的离线单测（无需 ROS）。

矩形以原点为中心，射线从原点出发 → 必然命中；这里验证距离解析式与对称性。
"""

import math

import pytest

from raspbot_mock.geometry import ray_rect_distance

HX, HY = 2.5, 2.0  # 5 x 4 m 房间的半宽


def test_axis_hits():
    assert ray_rect_distance(0.0, HX, HY) == pytest.approx(2.5)
    assert ray_rect_distance(math.pi, HX, HY) == pytest.approx(2.5)
    assert ray_rect_distance(math.pi / 2.0, HX, HY) == pytest.approx(2.0)
    assert ray_rect_distance(-math.pi / 2.0, HX, HY) == pytest.approx(2.0)


def test_first_quadrant_is_min_of_two_walls():
    # 第一象限：x 墙距离 HX/cos(theta)，y 墙距离 HY/sin(theta)，取先撞到的
    for deg in range(1, 90):
        theta = math.radians(deg)
        expected = min(HX / math.cos(theta), HY / math.sin(theta))
        assert ray_rect_distance(theta, HX, HY) == pytest.approx(expected)


def test_opposite_rays_symmetric():
    for i in range(720):
        angle = -math.pi + i * (2.0 * math.pi / 720.0)
        assert ray_rect_distance(angle, HX, HY) == pytest.approx(
            ray_rect_distance(angle + math.pi, HX, HY))


def test_all_720_rays_hit_within_bounds():
    for i in range(720):
        angle = -math.pi + i * (2.0 * math.pi / 720.0)
        d = ray_rect_distance(angle, HX, HY)
        assert d is not None
        assert 0.0 < d <= math.hypot(HX, HY)


def test_corner_consistency():
    # 45° 方向两墙距离相等（4 m 房间对角方向）
    half = 2.0
    d = ray_rect_distance(math.pi / 4.0, half, half)
    assert d == pytest.approx(half / math.cos(math.pi / 4.0))
    assert d == pytest.approx(half / math.sin(math.pi / 4.0))
