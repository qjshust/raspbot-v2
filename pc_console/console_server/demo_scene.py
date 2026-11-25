"""演示场景与地图算法（仅标准库）。

提供：

* 12 m × 8 m 室内场景（外墙 + 两个房间 + 走廊 + 家具），0.05 m 栅格
* 占用栅格、DDA 射线投影（模拟 LD19 360 线雷达）
* 栅格膨胀、A* 路径规划、路径直线化（string pulling）
* 灰度 PNG 编码（zlib + struct，无 Pillow）
* PGM(P5) 读写、地图 YAML 最小解析/生成（无 PyYAML）
* :class:`MapGrid`：与 ROS ``nav_msgs/OccupancyGrid`` 一致的行序（行 0 = y 最小）
"""

from __future__ import annotations

import logging
import math
import os
import re
import struct
import zlib
from pathlib import Path

log = logging.getLogger("console.demo.scene")

# ---- 场景常量 ------------------------------------------------------
RESOLUTION = 0.05  # m / cell
WIDTH = 240        # 12.0 m
HEIGHT = 160       # 8.0 m
ORIGIN_X = 0.0
ORIGIN_Y = 0.0

#: 演示地图取值（与 PGM / PNG 灰度一致）
UNKNOWN = 205
FREE = 254
OCCUPIED = 0

ROBOT_RADIUS = 0.18  # m
INFLATE_CELLS = 4    # 0.20 m，规划/碰撞用膨胀

LIDAR_RAYS = 360
LIDAR_MAX_RANGE = 8.0
LIDAR_MIN_RANGE = 0.10

#: 面积阈值：像素值 >= FREE_THRESHOLD 视为自由；<= OCC_THRESHOLD 视为占用
FREE_THRESHOLD = 250
OCC_THRESHOLD = 50

_NAME_RE = re.compile(r"^[A-Za-z0-9_.-]{1,40}$")


def wrap_angle(angle: float) -> float:
    """归一化到 (-pi, pi]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


# ===================================================================
# 场景（真值占用栅格）
# ===================================================================
class Scene:
    """12×8 m 室内真值场景。

    坐标系：世界 ``(x, y)`` 单位 m，x∈[0,12]，y∈[0,8]；
    栅格 ``(ix, iy)`` 行序 y 递增（``iy*width+ix``），与 OccupancyGrid 一致。
    """

    def __init__(self) -> None:
        self.width = WIDTH
        self.height = HEIGHT
        self.resolution = RESOLUTION
        self.origin = (ORIGIN_X, ORIGIN_Y)
        self.occupied = bytearray(WIDTH * HEIGHT)  # 1=占用 0=空闲
        self._build()
        self.inflated = self._inflate(self.occupied, INFLATE_CELLS)

    # ---- 构建 -------------------------------------------------
    def _fill(self, x0: float, y0: float, x1: float, y1: float) -> None:
        """把世界坐标矩形（含边框）填成占用。"""
        ix0 = max(0, int(math.floor(x0 / RESOLUTION)))
        iy0 = max(0, int(math.floor(y0 / RESOLUTION)))
        ix1 = min(WIDTH - 1, int(math.ceil(x1 / RESOLUTION)) - 1)
        iy1 = min(HEIGHT - 1, int(math.ceil(y1 / RESOLUTION)) - 1)
        for iy in range(iy0, iy1 + 1):
            base = iy * WIDTH
            for ix in range(ix0, ix1 + 1):
                self.occupied[base + ix] = 1

    def _build(self) -> None:
        # 外墙（厚 0.15 m）
        self._fill(0.00, 0.00, 12.00, 0.15)   # 下
        self._fill(0.00, 7.85, 12.00, 8.00)   # 上
        self._fill(0.00, 0.15, 0.15, 7.85)    # 左
        self._fill(11.85, 0.15, 12.00, 7.85)  # 右

        # 走廊北墙 y=4.85..5.00（走廊 y∈[3.0,4.85]）
        self._fill(0.15, 4.85, 2.20, 5.00)    # 房间 A 南墙左段（门 2.2~3.2）
        self._fill(3.20, 4.85, 5.15, 5.00)    # 房间 A 南墙右段（接东墙）
        self._fill(6.85, 4.85, 8.80, 5.00)    # 房间 B 南墙左段（门 8.8~9.8）
        self._fill(9.80, 4.85, 11.85, 5.00)   # 房间 B 南墙右段

        # 房间 A 东墙 x=5.00..5.15（门 y 5.8~6.8）
        self._fill(5.00, 5.00, 5.15, 5.80)
        self._fill(5.00, 6.80, 5.15, 7.85)

        # 房间 B 西墙 x=6.85..7.00
        self._fill(6.85, 5.00, 7.00, 7.85)

        # 走廊南墙 y=2.85..3.00（两个门洞 1.2~2.2、9.0~10.0）
        self._fill(0.15, 2.85, 1.20, 3.00)
        self._fill(2.20, 2.85, 9.00, 3.00)
        self._fill(10.00, 2.85, 11.85, 3.00)

        # 家具（真值障碍）
        self._fill(8.20, 0.60, 9.40, 1.60)    # 桌子
        self._fill(5.00, 0.35, 6.80, 1.05)    # 沙发
        self._fill(4.20, 3.40, 5.40, 3.70)    # 走廊置物架
        self._fill(0.60, 6.40, 1.70, 7.20)    # 房间 A 书桌
        self._fill(10.00, 5.50, 11.20, 6.50)  # 房间 B 箱子
        self._fill(10.30, 1.90, 10.90, 2.50)  # 立柱

    @staticmethod
    def _inflate(occupied: bytearray, cells: int) -> bytearray:
        inflated = bytearray(occupied)
        offsets: list[tuple[int, int]] = []
        r2 = cells * cells
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                if dx * dx + dy * dy <= r2:
                    offsets.append((dx, dy))
        for iy in range(HEIGHT):
            base = iy * WIDTH
            for ix in range(WIDTH):
                if not occupied[base + ix]:
                    continue
                for dx, dy in offsets:
                    nx, ny = ix + dx, iy + dy
                    if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                        inflated[ny * WIDTH + nx] = 1
        return inflated

    # ---- 查询 -------------------------------------------------
    def cell_of(self, x: float, y: float) -> tuple[int, int] | None:
        ix = int(math.floor(x / RESOLUTION))
        iy = int(math.floor(y / RESOLUTION))
        if 0 <= ix < WIDTH and 0 <= iy < HEIGHT:
            return ix, iy
        return None

    def is_occupied(self, ix: int, iy: int) -> bool:
        if 0 <= ix < WIDTH and 0 <= iy < HEIGHT:
            return bool(self.occupied[iy * WIDTH + ix])
        return True

    def is_free(self, x: float, y: float) -> bool:
        cell = self.cell_of(x, y)
        return cell is not None and not self.occupied[cell[1] * WIDTH + cell[0]]

    def is_free_inflated(self, x: float, y: float) -> bool:
        cell = self.cell_of(x, y)
        return cell is not None and not self.inflated[cell[1] * WIDTH + cell[0]]


# ===================================================================
# DDA 射线投影
# ===================================================================
def cast_ray(
    occupied: bytearray,
    width: int,
    height: int,
    ox: float,
    oy: float,
    angle: float,
    max_range: float = LIDAR_MAX_RANGE,
    resolution: float = RESOLUTION,
    collect: bool = False,
) -> tuple[float, list[tuple[int, int]], bool]:
    """Amanatides-Woo DDA 射线。

    返回 ``(distance, cells, hit)``：``cells`` 为经过的栅格（含起点与命中格），
    ``hit=False`` 时 distance 为 max_range（无障碍）。
    """
    dx = math.cos(angle)
    dy = math.sin(angle)
    ix = int(math.floor(ox / resolution))
    iy = int(math.floor(oy / resolution))
    if not (0 <= ix < width and 0 <= iy < height):
        return max_range, [], False
    cells: list[tuple[int, int]] = []
    if collect:
        cells.append((ix, iy))
    if occupied[iy * width + ix]:
        return 0.0, cells, True

    if dx > 0:
        step_x, t_delta_x = 1, resolution / dx
        t_max_x = ((ix + 1) * resolution - ox) / dx
    elif dx < 0:
        step_x, t_delta_x = -1, -resolution / dx
        t_max_x = (ix * resolution - ox) / dx
    else:
        step_x, t_delta_x, t_max_x = 0, math.inf, math.inf

    if dy > 0:
        step_y, t_delta_y = 1, resolution / dy
        t_max_y = ((iy + 1) * resolution - oy) / dy
    elif dy < 0:
        step_y, t_delta_y = -1, -resolution / dy
        t_max_y = (iy * resolution - oy) / dy
    else:
        step_y, t_delta_y, t_max_y = 0, math.inf, math.inf

    t = 0.0
    while t <= max_range:
        if t_max_x < t_max_y:
            t = t_max_x
            ix += step_x
            t_max_x += t_delta_x
        else:
            t = t_max_y
            iy += step_y
            t_max_y += t_delta_y
        if not (0 <= ix < width and 0 <= iy < height):
            return max_range, cells, False
        if collect:
            cells.append((ix, iy))
        if occupied[iy * width + ix]:
            if t > max_range:
                return max_range, cells, False
            return t, cells, True
    return max_range, cells, False


def scan_points(
    scene: Scene, x: float, y: float, yaw: float
) -> tuple[list[float], list[tuple[float, list[tuple[int, int]], bool]], float, float]:
    """生成 360 线激光（map frame 扁平点数组）+ 每条射线的命中栅格。

    返回 ``(points, rays, front_m, min_m)``；``rays`` 为
    ``[(distance, cells, hit), ...]``，供 SLAM 建图揭示使用。
    """
    points: list[float] = []
    rays: list[tuple[float, list[tuple[int, int]], bool]] = []
    front = LIDAR_MAX_RANGE
    minimum = LIDAR_MAX_RANGE
    for i in range(LIDAR_RAYS):
        rel = -math.pi + i * (2.0 * math.pi / LIDAR_RAYS)
        angle = wrap_angle(yaw + rel)
        dist, cells, hit = cast_ray(
            scene.occupied,
            scene.width,
            scene.height,
            x,
            y,
            angle,
            LIDAR_MAX_RANGE,
            scene.resolution,
            collect=True,
        )
        if not hit:
            dist = LIDAR_MAX_RANGE
        # 角度 0 附近视为正前方；用与 yaw 夹角最小的那条射线
        rel_abs = abs(rel)
        if rel_abs <= math.pi / LIDAR_RAYS:
            front = max(LIDAR_MIN_RANGE, dist)
        minimum = min(minimum, max(LIDAR_MIN_RANGE, dist))
        points.append(round(x + dist * math.cos(angle), 3))
        points.append(round(y + dist * math.sin(angle), 3))
        rays.append((dist, cells, hit))
    return points, rays, round(front, 3), round(minimum, 3)


# ===================================================================
# 栅格膨胀 / 路径规划
# ===================================================================
def inflate_blocked(blocked: bytearray, width: int, height: int, cells: int) -> bytearray:
    """对 0/1 障碍图做圆形膨胀。"""
    inflated = bytearray(blocked)
    offsets: list[tuple[int, int]] = []
    r2 = cells * cells
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if dx * dx + dy * dy <= r2:
                offsets.append((dx, dy))
    for iy in range(height):
        base = iy * width
        for ix in range(width):
            if not blocked[base + ix]:
                continue
            for dx, dy in offsets:
                nx, ny = ix + dx, iy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    inflated[ny * width + nx] = 1
    return inflated


def blocked_from_map_pixels(
    pixels: bytearray, width: int, height: int, unknown_is_blocked: bool = True
) -> bytearray:
    """由地图灰度（``MapGrid.pixels``，行 0 = y 最小）生成 0/1 障碍图。

    返回值与输入同样采用行 0 = y 最小（occupancy grid 行序）。
    """
    blocked = bytearray(width * height)
    if len(pixels) != width * height:
        raise MapFormatError(
            f"地图数据尺寸不匹配：{len(pixels)} != {width}*{height}"
        )
    for index in range(width * height):
        value = pixels[index]
        if value <= OCC_THRESHOLD:
            blocked[index] = 1
        elif value < FREE_THRESHOLD:
            blocked[index] = 1 if unknown_is_blocked else 0
    return blocked


def _line_free(
    blocked: bytearray,
    width: int,
    height: int,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
) -> bool:
    """Bresenham 直线可通行检查（同时检查对角两侧，避免切角）。"""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        if not (0 <= x < width and 0 <= y < height):
            return False
        if blocked[y * width + x]:
            return False
        if x == x1 and y == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def _nearest_free_cell(
    blocked: bytearray, width: int, height: int, ix: int, iy: int, radius: int = 6
) -> tuple[int, int] | None:
    if 0 <= ix < width and 0 <= iy < height and not blocked[iy * width + ix]:
        return ix, iy
    best: tuple[int, int] | None = None
    best_d = 1e9
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            nx, ny = ix + dx, iy + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if blocked[ny * width + nx]:
                continue
            d = dx * dx + dy * dy
            if d < best_d:
                best_d = d
                best = (nx, ny)
    return best


def astar_cells(
    blocked: bytearray,
    width: int,
    height: int,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]] | None:
    """8 邻域 A*（octile 启发式），返回栅格路径或 None。"""
    import heapq

    sx, sy = start
    gx, gy = goal
    if not (0 <= gx < width and 0 <= gy < height):
        return None
    if blocked[gy * width + gx]:
        return None
    start_cell = _nearest_free_cell(blocked, width, height, sx, sy)
    if start_cell is None:
        return None
    sx, sy = start_cell
    if (sx, sy) == (gx, gy):
        return [(sx, sy)]

    size = width * height
    g_score = [math.inf] * size
    parent = [-1] * size
    closed = bytearray(size)
    start_idx = sy * width + sx
    goal_idx = gy * width + gx
    g_score[start_idx] = 0.0

    def heuristic(x: int, y: int) -> float:
        dx = abs(x - gx)
        dy = abs(y - gy)
        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

    heap: list[tuple[float, int]] = [(heuristic(sx, sy), start_idx)]
    neighbors = (
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    )
    while heap:
        _, idx = heapq.heappop(heap)
        if closed[idx]:
            continue
        closed[idx] = 1
        if idx == goal_idx:
            path: list[tuple[int, int]] = []
            cur = idx
            while cur != -1:
                path.append((cur % width, cur // width))
                cur = parent[cur]
            path.reverse()
            return path
        cx = idx % width
        cy = idx // width
        for dx, dy, cost in neighbors:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            nidx = ny * width + nx
            if blocked[nidx] or closed[nidx]:
                continue
            # 禁止斜穿墙角
            if dx and dy and (blocked[cy * width + nx] or blocked[ny * width + cx]):
                continue
            ng = g_score[idx] + cost
            if ng < g_score[nidx]:
                g_score[nidx] = ng
                parent[nidx] = idx
                heapq.heappush(heap, (ng + heuristic(nx, ny), nidx))
    return None


def smooth_cells(
    path: list[tuple[int, int]], blocked: bytearray, width: int, height: int
) -> list[tuple[int, int]]:
    """直线化（string pulling）：能直连则跳过中间点。"""
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        j = n - 1
        while j > i + 1:
            if _line_free(
                blocked, width, height, path[i][0], path[i][1], path[j][0], path[j][1]
            ):
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan_path(
    blocked: bytearray,
    width: int,
    height: int,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    resolution: float = RESOLUTION,
    origin: tuple[float, float] = (ORIGIN_X, ORIGIN_Y),
    robot_radius_cells: int = INFLATE_CELLS,
) -> list[tuple[float, float]] | None:
    """规划世界坐标路径（含膨胀），返回折线（m）或 None。

    返回路径首点为 ``start_xy``，末点为 ``goal_xy``，中间为栅格中心。
    """
    inflated = (
        blocked
        if robot_radius_cells <= 0
        else inflate_blocked(blocked, width, height, robot_radius_cells)
    )
    start_cell = (
        int(math.floor((start_xy[0] - origin[0]) / resolution)),
        int(math.floor((start_xy[1] - origin[1]) / resolution)),
    )
    goal_cell = (
        int(math.floor((goal_xy[0] - origin[0]) / resolution)),
        int(math.floor((goal_xy[1] - origin[1]) / resolution)),
    )
    cells = astar_cells(inflated, width, height, start_cell, goal_cell)
    if cells is None:
        return None
    cells = smooth_cells(cells, inflated, width, height)
    world: list[tuple[float, float]] = []
    for ix, iy in cells:
        wx = origin[0] + (ix + 0.5) * resolution
        wy = origin[1] + (iy + 0.5) * resolution
        world.append((round(wx, 3), round(wy, 3)))
    world[0] = (round(start_xy[0], 3), round(start_xy[1], 3))
    world[-1] = (round(goal_xy[0], 3), round(goal_xy[1], 3))
    # 去掉重复点
    dedup: list[tuple[float, float]] = []
    for pt in world:
        if not dedup or abs(pt[0] - dedup[-1][0]) > 1e-6 or abs(pt[1] - dedup[-1][1]) > 1e-6:
            dedup.append(pt)
    return dedup if len(dedup) >= 2 else None


def path_length(points: list[tuple[float, float]]) -> float:
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


# ===================================================================
# MapGrid：y 递增行序（occupancy grid），像素值 205/254/0
# ===================================================================
class MapGrid:
    """演示地图。

    * ``pixels`` 行序与 ``nav_msgs/OccupancyGrid`` 一致：行 0 = y 最小
    * 像素取值与 PGM/PNG 灰度一致：205 未知 / 254 空闲 / 0 占用
    * PNG 与 PGM 输出时翻转行序（文件行 0 = y 最大）
    """

    __slots__ = ("width", "height", "resolution", "origin", "pixels", "version", "stamp", "frame_id")

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float = RESOLUTION,
        origin: tuple[float, float, float] = (ORIGIN_X, ORIGIN_Y, 0.0),
        pixels: bytearray | None = None,
        version: int = 0,
    ) -> None:
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        if pixels is None:
            pixels = bytearray([UNKNOWN]) * (width * height)
        if len(pixels) != width * height:
            raise MapFormatError(
                f"地图数据尺寸不匹配：{len(pixels)} != {width}*{height}"
            )
        self.pixels = bytearray(pixels)
        self.version = version
        self.stamp = 0.0
        self.frame_id = "map"

    # ---- 构造 -------------------------------------------------
    @classmethod
    def from_scene(cls, scene: Scene, unknown: bool = False) -> "MapGrid":
        pixels = bytearray(scene.width * scene.height)
        for iy in range(scene.height):
            base = iy * scene.width
            for ix in range(scene.width):
                pixels[base + ix] = OCCUPIED if scene.occupied[base + ix] else FREE
        if unknown:
            pixels = bytearray([UNKNOWN]) * (scene.width * scene.height)
        return cls(scene.width, scene.height, scene.resolution, (ORIGIN_X, ORIGIN_Y, 0.0), pixels)

    @classmethod
    def from_pgm(
        cls,
        width: int,
        height: int,
        rows_top_first: bytes,
        resolution: float,
        origin: tuple[float, float, float],
    ) -> "MapGrid":
        if len(rows_top_first) != width * height:
            raise MapFormatError(
                f"PGM 数据尺寸不匹配：{len(rows_top_first)} != {width}*{height}"
            )
        pixels = bytearray(width * height)
        w = width
        for row in range(height):
            src = row * w
            dst = (height - 1 - row) * w
            pixels[dst : dst + w] = rows_top_first[src : src + w]
        return cls(width, height, resolution, origin, pixels)

    # ---- 序列化 -----------------------------------------------
    def rows_top_first(self) -> bytes:
        """PNG/PGM 行序（行 0 = y 最大）。"""
        w = self.width
        out = bytearray(w * self.height)
        for row in range(self.height):
            src = (self.height - 1 - row) * w
            out[row * w : (row + 1) * w] = self.pixels[src : src + w]
        return bytes(out)

    def to_meta(self, available: bool = True) -> dict:
        if not available:
            # 契约允许其余字段为 null；给数值占位以通过严格 schema 校验（available=false
            # 时前端必须忽略这些值）。
            return {
                "available": False,
                "version": self.version,
                "width": self.width,
                "height": self.height,
                "resolution": self.resolution,
                "origin": {
                    "x": float(self.origin[0]),
                    "y": float(self.origin[1]),
                    "yaw": float(self.origin[2]),
                },
                "frame_id": "map",
                "stamp": round(self.stamp, 3) if self.stamp else 0.0,
            }
        return {
            "available": True,
            "version": self.version,
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": {
                "x": float(self.origin[0]),
                "y": float(self.origin[1]),
                "yaw": float(self.origin[2]),
            },
            "frame_id": self.frame_id,
            "stamp": round(self.stamp, 3) if self.stamp else None,
        }

    # ---- 栅格访问 ---------------------------------------------
    def index(self, ix: int, iy: int) -> int:
        return iy * self.width + ix

    def get_cell(self, ix: int, iy: int) -> int:
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return self.pixels[iy * self.width + ix]
        return UNKNOWN

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        ix = int(math.floor((x - self.origin[0]) / self.resolution))
        iy = int(math.floor((y - self.origin[1]) / self.resolution))
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return ix, iy
        return None

    def cell_to_world(self, ix: int, iy: int) -> tuple[float, float]:
        return (
            self.origin[0] + (ix + 0.5) * self.resolution,
            self.origin[1] + (iy + 0.5) * self.resolution,
        )


class MapFormatError(ValueError):
    """地图文件格式错误。"""


# ===================================================================
# PNG（8-bit 灰度，标准库实现）
# ===================================================================
def _png_chunk(tag: bytes, data: bytes) -> bytes:
    return (
        struct.pack(">I", len(data))
        + tag
        + data
        + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    )


def encode_png_gray(width: int, height: int, rows_top_first: bytes, level: int = 6) -> bytes:
    """把灰度行数据（行 0 = 顶 = y 最大）编码为 8-bit 灰度 PNG。"""
    if len(rows_top_first) != width * height:
        raise ValueError(
            f"PNG 数据尺寸不匹配：{len(rows_top_first)} != {width}*{height}"
        )
    raw = bytearray()
    stride = width
    for row in range(height):
        raw.append(0)  # filter type 0 (None)
        start = row * stride
        raw.extend(rows_top_first[start : start + stride])
    compressed = zlib.compress(bytes(raw), level)
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", ihdr)
        + _png_chunk(b"IDAT", compressed)
        + _png_chunk(b"IEND", b"")
    )


def map_png(map_grid: MapGrid, level: int = 6) -> bytes:
    """地图 → PNG 字节。"""
    return encode_png_gray(map_grid.width, map_grid.height, map_grid.rows_top_first(), level)


# ===================================================================
# PGM（P5）读写
# ===================================================================
def encode_pgm(width: int, height: int, rows_top_first: bytes) -> bytes:
    if len(rows_top_first) != width * height:
        raise ValueError("PGM 数据尺寸不匹配")
    header = f"P5\n# generated by raspbot pc_console demo\n{width} {height}\n255\n".encode("ascii")
    return header + bytes(rows_top_first)


def write_pgm(path: str | os.PathLike, width: int, height: int, rows_top_first: bytes) -> None:
    Path(path).write_bytes(encode_pgm(width, height, rows_top_first))


def read_pgm(path: str | os.PathLike) -> tuple[int, int, bytes]:
    """读取 P5 灰度 PGM，返回 ``(width, height, rows_top_first)``。"""
    data = Path(path).read_bytes()
    return parse_pgm(data)


def parse_pgm(data: bytes) -> tuple[int, int, bytes]:
    if not data:
        raise MapFormatError("PGM 文件为空")
    pos = 0
    n = len(data)

    def _skip_ws() -> None:
        nonlocal pos
        while pos < n:
            ch = data[pos : pos + 1]
            if ch in b" \t\r\n":
                pos += 1
            elif ch == b"#":
                while pos < n and data[pos : pos + 1] not in b"\r\n":
                    pos += 1
            else:
                return

    _skip_ws()
    magic = data[pos : pos + 2]
    if magic != b"P5":
        raise MapFormatError("不是 P5 格式 PGM")
    pos += 2
    numbers: list[int] = []
    while len(numbers) < 3:
        _skip_ws()
        start = pos
        while pos < n and data[pos : pos + 1].isdigit():
            pos += 1
        if start == pos:
            raise MapFormatError("PGM 头部解析失败")
        numbers.append(int(data[start:pos]))
    width, height, maxval = numbers
    if width <= 0 or height <= 0 or maxval <= 0 or maxval > 255:
        raise MapFormatError("PGM 头部数值非法")
    # 头部后恰好一个空白字符
    if pos < n and data[pos : pos + 1] in b" \t\r\n":
        pos += 1
    pixels = data[pos:]
    if len(pixels) < width * height:
        raise MapFormatError("PGM 像素数据不足")
    return width, height, pixels[: width * height]


# ===================================================================
# 地图 YAML（最小安全解析器，不依赖 PyYAML）
# ===================================================================
_YAML_LINE_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*?)\s*$")


def _strip_comment(text: str) -> str:
    in_bracket = 0
    for i, ch in enumerate(text):
        if ch in "[({":
            in_bracket += 1
        elif ch in "])}":
            in_bracket = max(0, in_bracket - 1)
        elif ch == "#" and in_bracket == 0:
            return text[:i].rstrip()
    return text.rstrip()


def parse_map_yaml(path: str | os.PathLike) -> dict:
    """解析 map_server 风格 YAML（image/resolution/origin/...）。坏文件抛 MapFormatError。"""
    yaml_path = Path(path)
    try:
        text = yaml_path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise MapFormatError(f"无法读取地图 YAML：{exc}") from exc
    data: dict = {}
    for raw_line in text.splitlines():
        line = _strip_comment(raw_line)
        if not line.strip() or line.lstrip().startswith("---"):
            continue
        match = _YAML_LINE_RE.match(line)
        if not match:
            continue
        key, value = match.group(1), match.group(2)
        if value == "":
            data[key] = None
            continue
        if value.startswith("[") and value.endswith("]"):
            inner = value[1:-1].strip()
            try:
                data[key] = [float(part) for part in inner.split(",") if part.strip()]
            except ValueError:
                data[key] = value
            continue
        if (value.startswith('"') and value.endswith('"')) or (
            value.startswith("'") and value.endswith("'")
        ):
            data[key] = value[1:-1]
            continue
        if value.lower() in ("true", "false"):
            data[key] = value.lower() == "true"
            continue
        data[key] = value

    for required in ("image", "resolution", "origin"):
        if required not in data or data[required] is None:
            raise MapFormatError(f"缺少必需字段 {required}")
    try:
        resolution = float(data["resolution"])
    except (TypeError, ValueError) as exc:
        raise MapFormatError("resolution 不是数字") from exc
    origin = data["origin"]
    if not isinstance(origin, list) or len(origin) < 3:
        raise MapFormatError("origin 必须是 [x, y, yaw]")
    image = data["image"]
    if not isinstance(image, str) or not image:
        raise MapFormatError("image 必须是非空字符串")
    return {
        "image": image,
        "resolution": resolution,
        "origin": (float(origin[0]), float(origin[1]), float(origin[2])),
        "mode": data.get("mode", "trinary"),
        "negate": data.get("negate", 0),
        "occupied_thresh": float(data.get("occupied_thresh", 0.65)),
        "free_thresh": float(data.get("free_thresh", 0.196)),
    }


def write_map_yaml(
    path: str | os.PathLike,
    image_name: str,
    resolution: float,
    origin: tuple[float, float, float],
    mode: str = "trinary",
    negate: int = 0,
    occupied_thresh: float = 0.65,
    free_thresh: float = 0.196,
) -> None:
    """写 map_server 风格 YAML（字段与设计文档 §3.3.2 一致）。"""
    content = (
        f"image: {image_name}\n"
        f"mode: {mode}\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
        f"negate: {negate}\n"
        f"occupied_thresh: {occupied_thresh}\n"
        f"free_thresh: {free_thresh}\n"
    )
    Path(path).write_text(content, encoding="utf-8", newline="\n")


def load_map_grid(yaml_path: str | os.PathLike) -> MapGrid:
    """从 YAML + PGM 加载 :class:`MapGrid`。"""
    yaml_path = Path(yaml_path)
    meta = parse_map_yaml(yaml_path)
    image = Path(meta["image"])
    if not image.is_absolute():
        image = yaml_path.parent / image
    if not image.is_file():
        raise MapFormatError(f"地图图片不存在：{image}")
    width, height, rows = read_pgm(image)
    return MapGrid.from_pgm(width, height, rows, meta["resolution"], meta["origin"])


def valid_map_name(name) -> bool:
    return isinstance(name, str) and bool(_NAME_RE.match(name))


__all__ = [
    "RESOLUTION",
    "WIDTH",
    "HEIGHT",
    "ORIGIN_X",
    "ORIGIN_Y",
    "FREE",
    "OCCUPIED",
    "UNKNOWN",
    "LIDAR_RAYS",
    "LIDAR_MAX_RANGE",
    "INFLATE_CELLS",
    "ROBOT_RADIUS",
    "wrap_angle",
    "clamp",
    "Scene",
    "cast_ray",
    "scan_points",
    "inflate_blocked",
    "blocked_from_map_pixels",
    "astar_cells",
    "smooth_cells",
    "plan_path",
    "path_length",
    "MapGrid",
    "MapFormatError",
    "encode_png_gray",
    "map_png",
    "encode_pgm",
    "write_pgm",
    "read_pgm",
    "parse_pgm",
    "parse_map_yaml",
    "write_map_yaml",
    "load_map_grid",
    "valid_map_name",
]
