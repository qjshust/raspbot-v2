"""raspbot_mock geometry — 纯几何函数，离线可测（不依赖 ROS）。"""

import math


def ray_rect_distance(angle, half_x, half_y):
    """原点沿 angle 方向的射线与 [-half_x, half_x] x [-half_y, half_y]
    矩形的交点距离；无交点返回 None。

    slab 方法：对每个轴分别求进入/离开矩形的 t 区间再取交集。
    车位于 (0,0)（矩形内部）时每个方向都会命中。
    """
    dx = math.cos(angle)
    dy = math.sin(angle)
    tmin = -math.inf
    tmax = math.inf
    for origin, direction, half in ((0.0, dx, half_x), (0.0, dy, half_y)):
        if abs(direction) < 1e-12:
            # 与轴平行：原点必须在 slab 内，否则整条射线无交点
            if origin < -half or origin > half:
                return None
        else:
            t1 = (-half - origin) / direction
            t2 = (half - origin) / direction
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
    if tmax < tmin or tmax < 0.0:
        return None
    if tmin < 0.0:
        tmin = tmax  # 原点在矩形内部且沿反方向先穿出
    return tmin
