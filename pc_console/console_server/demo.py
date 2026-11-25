"""内存演示后端（``--demo``）。

在 PC 进程内实现与 Pi 网关 **完全一致** 的 Wire API v1（设计文档 §3，含 SSE），
不依赖 ROS / Pi / 网络，用于无硬件验收与 Web UI 联调：

* 12 m × 8 m 室内场景、0.05 m 栅格（:mod:`console_server.demo_scene`）
* ``slam``：地图从未知开始，随移动与 360° 雷达扫描逐步揭示
* ``nav``：内置 ``demo_maps/demo_home.pgm/.yaml``，initialpose → 2~3 s 激活 →
  A* 路径 → 沿路径行驶 → ``succeeded``；不可达目标 ``failed/aborted``
* 遥控：麦轮全向积分 + 墙体碰撞保护 + 500 ms 看门狗；急停锁存/解除
* 会话启动/停止 1~3 s 延迟，完整走过 starting/running/stopping/error
* 保存地图（PGM+YAML）、地图列表/预览、诊断、事件日志、会话日志、``/api/ros``
* SSE：state 10 Hz、scan 5 Hz、path/map/session/log 即时；多客户端、断开安全清理
"""

from __future__ import annotations

import json
import logging
import math
import os
import queue
import random
import threading
import time
import uuid
from collections import deque
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from .demo_scene import (
    FREE,
    INFLATE_CELLS,
    LIDAR_MAX_RANGE,
    LIDAR_MIN_RANGE,
    MapFormatError,
    MapGrid,
    OCCUPIED,
    RESOLUTION,
    Scene,
    UNKNOWN,
    blocked_from_map_pixels,
    clamp,
    load_map_grid,
    map_png,
    parse_map_yaml,
    path_length,
    plan_path,
    read_pgm,
    scan_points,
    valid_map_name,
    wrap_angle,
    write_map_yaml,
    write_pgm,
)
from .upstream import WireResponse, WireStream

log = logging.getLogger("console.demo")

VERSION = "1.0.0"
API_VERSION = 1
NAME = "raspbot-console"

LIMITS = {"max_linear": 0.30, "max_lateral": 0.30, "max_angular": 1.5}
TELEOP_WATCHDOG_S = 0.5
MAX_SSE_CLIENTS = 4
ACTIVATION_TOTAL = 6
ACTIVATION_PHASES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "collision_monitor",
    "navigate_to_pose",
]
MODE_LABELS = {"base": "手动遥控", "slam": "建图", "nav": "导航"}

_MISSING = object()


class DemoHttpError(Exception):
    """演示后端返回给 HTTP 层的错误（转成统一 JSON）。"""

    def __init__(self, status: int, code: str, message: str, detail: dict | None = None):
        super().__init__(message)
        self.status = status
        self.code = code
        self.message = message
        self.detail = detail or {}

    def payload(self) -> dict:
        error = {"code": self.code, "message": self.message}
        if self.detail:
            error["detail"] = self.detail
        return {"error": error}


class TooManySseClients(Exception):
    pass


class EventHub:
    """SSE 广播中心：每客户端独立队列（maxsize 200，丢弃最旧）。"""

    def __init__(self, max_clients: int = MAX_SSE_CLIENTS, maxsize: int = 200):
        self._lock = threading.Lock()
        self._clients: set[queue.Queue] = set()
        self._max_clients = max_clients
        self._maxsize = maxsize
        self._closed = False

    def count(self) -> int:
        with self._lock:
            return len(self._clients)

    def subscribe(self) -> queue.Queue:
        with self._lock:
            if self._closed:
                raise TooManySseClients("演示后端已关闭")
            if len(self._clients) >= self._max_clients:
                raise TooManySseClients(
                    f"SSE 客户端已达上限（{self._max_clients}）"
                )
            client: queue.Queue = queue.Queue(maxsize=self._maxsize)
            self._clients.add(client)
            return client

    def unsubscribe(self, client: queue.Queue) -> None:
        with self._lock:
            self._clients.discard(client)

    def publish(self, event: str, data: dict) -> None:
        message = (
            f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False, separators=(',', ':'))}\n\n"
        ).encode("utf-8")
        with self._lock:
            clients = list(self._clients)
        for client in clients:
            try:
                client.put_nowait(message)
            except queue.Full:
                try:
                    client.get_nowait()  # 丢弃最旧
                except queue.Empty:
                    pass
                try:
                    client.put_nowait(message)
                except queue.Full:
                    pass

    def close(self) -> None:
        with self._lock:
            self._closed = True
            clients = list(self._clients)
            self._clients.clear()
        for client in clients:
            try:
                client.put_nowait(None)
            except queue.Full:
                try:
                    client.get_nowait()
                    client.put_nowait(None)
                except (queue.Empty, queue.Full):
                    pass


class DemoBackend:
    """演示后端：实现 ``request`` / ``event_stream`` / ``start`` / ``stop``。"""

    def __init__(self, demo_maps_dir: str | os.PathLike | None = None) -> None:
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._session_thread: threading.Thread | None = None
        self._hub = EventHub()
        self._random = random.Random(20261124)
        self._created_wall = time.time()
        self._created_mono = time.monotonic()

        # 场景与地图
        self._scene = Scene()
        base = Path(__file__).resolve().parent.parent
        self.demo_maps_dir = (
            Path(demo_maps_dir) if demo_maps_dir else base / "demo_maps"
        )
        self._ensure_demo_maps_dir()
        self._ensure_demo_map()

        # 会话
        self._session = {
            "mode": "idle",
            "state": "idle",
            "mock": False,
            "managed": True,
            "external": False,
            "available_modes": ["base", "slam", "nav"],
            "since": round(self._created_wall, 3),
            "progress": 0.0,
            "message": "空闲：可选择运行模式",
            "error": None,
            "map": None,
        }
        self._session_busy = False
        self._session_thread = None

        # 机器人 / 运动
        self._pose = {"x": 1.5, "y": 1.5, "yaw": 0.0}
        self._speed = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._collision_blocked = False

        # 遥控
        self._teleop = {"active": False, "vx": 0.0, "vy": 0.0, "wz": 0.0, "last_cmd": None}
        self._teleop_seq: int | None = None
        self._last_cmd_mono: float | None = None

        # 急停
        self._estop = {"active": False, "since": None}

        # 外设
        self._peripherals = {
            "buzzer": False,
            "led": {"r": 0.0, "g": 0.0, "b": 0.0},
            "servo": {"pan": 90.0, "tilt": 45.0},
        }

        # 地图
        self._map: MapGrid | None = None
        self._map_available = False
        self._map_stamp: float | None = None
        self._map_version_seq = 0
        self._blocked: bytearray | None = None
        self._png_cache: tuple[int, bytes] | None = None

        # 导航
        self._nav = self._empty_nav()
        self._nav_path: list[tuple[float, float]] | None = None
        self._nav_path_index = 1
        self._nav_started_mono: float | None = None
        self._nav_fail_at: float | None = None
        self._goal_id: str | None = None
        self._activation_start = 0.0
        self._activation_duration = 0.0

        # 激光 / 路径
        self._scan_stamp = 0.0
        self._scan_front: float | None = None
        self._scan_min: float | None = None
        self._path_stamp = 0.0

        # 日志
        self._logs: deque[dict] = deque(maxlen=300)
        self._session_logs: deque[dict] = deque(maxlen=500)

        # 发布节奏
        self._next_state_pub = 0.0
        self._next_scan_pub = 0.0

        self._log("info", "PC 控制台演示后端已启动（内置仿真，无需 Pi/ROS）")
        self._log(
            "info",
            f"演示场景已加载：12.0 m × 8.0 m @ {RESOLUTION} m，"
            f"栅格 {self._scene.width}×{self._scene.height}，机器人起点 (1.5, 1.5)",
        )

    # ===============================================================
    # 生命周期
    # ===============================================================
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._created_mono = time.monotonic()
        self._thread = threading.Thread(target=self._loop, name="demo-sim", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._hub.close()
        for thread in (self._thread, self._session_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)

    # ===============================================================
    # 工具
    # ===============================================================
    @staticmethod
    def _empty_nav() -> dict:
        return {
            "state": "unavailable",
            "message": "未进入导航模式",
            "initialpose": None,
            "goal": None,
            "goal_state": "idle",
            "distance_remaining": None,
            "navigation_time_s": None,
            "recoveries": None,
            "lifecycle": {},
            "activation": {"done": 0, "total": ACTIVATION_TOTAL, "phase": ""},
        }

    @staticmethod
    def _empty_map_meta() -> dict:
        # 契约允许 map.available=false 时其余字段为 null；这里给出数值占位（version=0、
        # 尺寸=0、stamp=0.0），既满足「前端不得依赖 available=false 的字段」，
        # 也便于 tools/console/contract_check.py 的严格 schema 校验。
        return {
            "available": False,
            "version": 0,
            "width": 0,
            "height": 0,
            "resolution": 0.0,
            "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "frame_id": "map",
            "stamp": 0.0,
        }

    def _json_response(self, status: int, obj: dict) -> WireResponse:
        data = json.dumps(obj, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        return WireResponse(
            status,
            [
                ("Content-Type", "application/json; charset=utf-8"),
                ("Cache-Control", "no-store"),
            ],
            data,
        )

    def _binary_response(self, status: int, content_type: str, data: bytes) -> WireResponse:
        return WireResponse(
            status,
            [("Content-Type", content_type), ("Cache-Control", "no-store")],
            data,
        )

    def _log(self, level: str, message: str) -> None:
        entry = {"ts": round(time.time(), 3), "level": level, "msg": str(message)}
        self._logs.append(entry)
        self._hub.publish("log", entry)

    def _session_log(self, line: str) -> None:
        self._session_logs.append({"ts": round(time.time(), 3), "line": str(line)})

    def _num(self, body: dict, key: str, default=_MISSING) -> float:
        if key not in body:
            if default is _MISSING:
                raise DemoHttpError(
                    400, "bad_request", f"缺少字段 {key}", {"field": key}
                )
            return float(default)
        value = body[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise DemoHttpError(
                400, "bad_request", f"字段 {key} 必须是数值", {"field": key}
            )
        value = float(value)
        if not math.isfinite(value):
            raise DemoHttpError(
                400, "bad_request", f"字段 {key} 必须是有限数值", {"field": key}
            )
        return value

    @staticmethod
    def _json_body(body: bytes) -> dict:
        if not body:
            return {}
        try:
            data = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise DemoHttpError(400, "bad_request", f"请求体不是合法 JSON：{exc}") from exc
        if not isinstance(data, dict):
            raise DemoHttpError(400, "bad_request", "请求体必须是 JSON 对象")
        return data

    # ---- 快照 -----------------------------------------------------
    def _session_snapshot(self) -> dict:
        return dict(self._session)

    def _nav_snapshot(self) -> dict:
        nav = dict(self._nav)
        nav["lifecycle"] = dict(self._nav.get("lifecycle") or {})
        nav["activation"] = dict(self._nav.get("activation") or {})
        if self._nav.get("initialpose"):
            nav["initialpose"] = dict(self._nav["initialpose"])
        if self._nav.get("goal"):
            nav["goal"] = dict(self._nav["goal"])
        return nav

    def _estop_snapshot(self) -> dict:
        return {"active": bool(self._estop["active"]), "since": self._estop["since"]}

    def _peripherals_snapshot(self) -> dict:
        return {
            "buzzer": bool(self._peripherals["buzzer"]),
            "led": dict(self._peripherals["led"]),
            "servo": dict(self._peripherals["servo"]),
        }

    # ===============================================================
    # 演示地图预生成 / 列表
    # ===============================================================
    def _ensure_demo_maps_dir(self) -> None:
        try:
            self.demo_maps_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:  # pragma: no cover - 权限异常时仍可运行
            log.warning("无法创建演示地图目录 %s: %s", self.demo_maps_dir, exc)

    @property
    def demo_home_yaml(self) -> Path:
        return self.demo_maps_dir / "demo_home.yaml"

    @property
    def demo_home_pgm(self) -> Path:
        return self.demo_maps_dir / "demo_home.pgm"

    def _ensure_demo_map(self) -> None:
        """首次运行生成演示地图 ``demo_home.pgm/.yaml``（12×8 m 全已知）。"""
        if self.demo_home_yaml.is_file() and self.demo_home_pgm.is_file():
            return
        try:
            self._ensure_demo_maps_dir()
            grid = MapGrid.from_scene(self._scene, unknown=False)
            write_pgm(
                self.demo_home_pgm, grid.width, grid.height, grid.rows_top_first()
            )
            write_map_yaml(
                self.demo_home_yaml,
                "demo_home.pgm",
                grid.resolution,
                (grid.origin[0], grid.origin[1], grid.origin[2]),
            )
            log.info("已生成演示地图: %s", self.demo_home_yaml)
        except OSError as exc:  # pragma: no cover
            log.warning("生成演示地图失败: %s", exc)

    def _list_maps(self) -> dict:
        maps: list[dict] = []
        dirs = [str(self.demo_maps_dir.resolve())]
        if self.demo_maps_dir.is_dir():
            for yaml_path in sorted(self.demo_maps_dir.glob("*.yaml")):
                name = yaml_path.stem
                if not valid_map_name(name):
                    continue
                entry = {
                    "name": name,
                    "path": str(yaml_path.resolve()),
                    "yaml": str(yaml_path.resolve()),
                    "pgm": str((yaml_path.parent / f"{name}.pgm").resolve()),
                    "mtime": round(yaml_path.stat().st_mtime, 3),
                    "resolution": None,
                    "width": None,
                    "height": None,
                    "preview_url": f"/api/maps/{name}/preview.png",
                    "valid": False,
                }
                try:
                    meta = parse_map_yaml(yaml_path)
                    image = Path(meta["image"])
                    if not image.is_absolute():
                        image = yaml_path.parent / image
                    width, height, _rows = read_pgm(image)
                    entry.update(
                        resolution=meta["resolution"],
                        width=width,
                        height=height,
                        pgm=str(image.resolve()),
                        valid=True,
                    )
                except (MapFormatError, OSError):
                    pass
                maps.append(entry)
        return {"maps": maps, "dirs": dirs}

    # ===============================================================
    # 主循环：仿真步进 + 发布节奏
    # ===============================================================
    def _loop(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            now_mono = time.monotonic()
            dt = clamp(now_mono - last, 0.0, 0.12)
            last = now_mono
            try:
                with self._lock:
                    self._step(now_mono, dt)
            except Exception:  # pragma: no cover - 仿真线程不能死
                log.exception("演示仿真步进异常")
            wall = time.time()
            if wall >= self._next_state_pub:
                self._next_state_pub = wall + 0.1
                if self._hub.count() > 0:
                    try:
                        self._publish_state()
                    except Exception:  # pragma: no cover
                        log.exception("推送 state 事件失败")
            if now_mono >= self._next_scan_pub:
                self._next_scan_pub = now_mono + 0.2
                with self._lock:
                    if self._session["state"] == "running":
                        try:
                            self._do_scan(time.time())
                        except Exception:  # pragma: no cover
                            log.exception("激光仿真异常")
            self._stop.wait(0.02)

    def _step(self, now_mono: float, dt: float) -> None:
        """一次仿真步进（调用方持有锁）。"""
        # 1) 遥控看门狗
        if (
            self._teleop["active"]
            and self._last_cmd_mono is not None
            and now_mono - self._last_cmd_mono > TELEOP_WATCHDOG_S
        ):
            self._teleop.update(active=False, vx=0.0, vy=0.0, wz=0.0)
            self._last_cmd_mono = None
            self._speed = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
            self._log("info", "遥控指令超时（500 ms 看门狗），已自动停车")

        # 2) Nav2 激活进度 / 规划失败延迟
        if self._nav["state"] == "activating":
            self._advance_activation(now_mono)
        if self._nav_fail_at is not None and now_mono >= self._nav_fail_at:
            self._nav_fail_at = None
            self._fail_nav("目标不可达：无法规划到目标的路径")

        # 3) 速度来源：急停 > 导航 > 遥控
        vx = vy = wz = 0.0
        if self._estop["active"]:
            self._speed = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
            return
        if self._nav["state"] == "goal_active" and self._nav_path:
            vx, vy, wz = self._nav_control(now_mono)
        elif self._teleop["active"]:
            vx = self._teleop["vx"]
            vy = self._teleop["vy"]
            wz = self._teleop["wz"]
        self._apply_motion(vx, vy, wz, dt)

    # ---- 运动学 ---------------------------------------------------
    def _apply_motion(self, vx: float, vy: float, wz: float, dt: float) -> None:
        vx = clamp(vx, -LIMITS["max_linear"], LIMITS["max_linear"])
        vy = clamp(vy, -LIMITS["max_lateral"], LIMITS["max_lateral"])
        wz = clamp(wz, -LIMITS["max_angular"], LIMITS["max_angular"])
        x = self._pose["x"]
        y = self._pose["y"]
        yaw = self._pose["yaw"]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        wx = vx * cos_yaw - vy * sin_yaw
        wy = vx * sin_yaw + vy * cos_yaw
        new_yaw = wrap_angle(yaw + wz * dt)
        nx = x + wx * dt
        ny = y + wy * dt
        applied_wx = applied_wy = 0.0
        linear_cmd = abs(wx) > 1e-9 or abs(wy) > 1e-9
        if self._scene.is_free_inflated(nx, ny):
            self._pose["x"] = nx
            self._pose["y"] = ny
            applied_wx, applied_wy = wx, wy
        elif self._scene.is_free_inflated(nx, y):
            self._pose["x"] = nx
            applied_wx = wx
        elif self._scene.is_free_inflated(x, ny):
            self._pose["y"] = ny
            applied_wy = wy
        else:
            # 撞墙：整车停住
            if linear_cmd and not self._collision_blocked:
                self._collision_blocked = True
                self._log("warn", "底盘碰撞保护：前方障碍，已停止")
        if applied_wx or applied_wy:
            self._collision_blocked = False
        self._pose["yaw"] = new_yaw
        if applied_wx == 0.0 and applied_wy == 0.0:
            # 原地自转时 wz 仍然有效；被墙挡住则全部归零
            self._speed = {
                "vx": 0.0,
                "vy": 0.0,
                "wz": 0.0 if linear_cmd else round(wz, 3),
            }
        else:
            cos_new = math.cos(new_yaw)
            sin_new = math.sin(new_yaw)
            self._speed = {
                "vx": round(applied_wx * cos_new + applied_wy * sin_new, 3),
                "vy": round(-applied_wx * sin_new + applied_wy * cos_new, 3),
                "wz": round(wz, 3),
            }

    # ---- 激光 / SLAM ----------------------------------------------
    def _do_scan(self, now_wall: float) -> None:
        x = self._pose["x"]
        y = self._pose["y"]
        yaw = self._pose["yaw"]
        points, rays, front, minimum = scan_points(self._scene, x, y, yaw)
        self._scan_stamp = now_wall
        self._scan_front = front
        self._scan_min = minimum
        self._hub.publish(
            "scan",
            {"stamp": round(now_wall, 3), "frame": "map", "points": points},
        )
        if (
            self._session["mode"] == "slam"
            and self._session["state"] == "running"
            and self._map is not None
        ):
            self._reveal_map(rays, now_wall)

    def _reveal_map(
        self,
        rays: list[tuple[float, list[tuple[int, int]], bool]],
        now_wall: float,
    ) -> None:
        grid = self._map
        if grid is None:
            return
        pixels = grid.pixels
        width = grid.width
        changed = False
        for _dist, cells, hit in rays:
            if not cells:
                continue
            limit = len(cells) - 1 if hit else len(cells)
            for ix, iy in cells[1:limit]:
                idx = iy * width + ix
                if pixels[idx] == UNKNOWN:
                    pixels[idx] = FREE
                    changed = True
            if hit:
                ix, iy = cells[-1]
                idx = iy * width + ix
                if pixels[idx] != OCCUPIED:
                    pixels[idx] = OCCUPIED
                    changed = True
        if changed:
            self._map_version_seq += 1
            grid.version = self._map_version_seq
            grid.stamp = now_wall
            self._map_stamp = now_wall
            self._map_available = True
            self._png_cache = None
            self._hub.publish("map", {"meta": grid.to_meta(True)})

    # ===============================================================
    # 导航
    # ===============================================================
    def _advance_activation(self, now_mono: float) -> None:
        duration = self._activation_duration
        if duration <= 0:
            duration = 2.5
        frac = clamp((now_mono - self._activation_start) / duration, 0.0, 1.0)
        done = int(frac * ACTIVATION_TOTAL)
        nav = self._nav
        if done >= ACTIVATION_TOTAL:
            nav["state"] = "ready"
            nav["message"] = "导航就绪，点击地图设置目的地"
            nav["activation"] = {
                "done": ACTIVATION_TOTAL,
                "total": ACTIVATION_TOTAL,
                "phase": "完成",
            }
            nav["lifecycle"] = {name: "active" for name in ACTIVATION_PHASES}
            self._activation_duration = 0.0
            self._log("info", "Nav2 生命周期激活完成，导航就绪")
            self._publish_state()
        else:
            phase = ACTIVATION_PHASES[done]
            nav["activation"] = {
                "done": done,
                "total": ACTIVATION_TOTAL,
                "phase": phase,
            }
            nav["lifecycle"] = {
                name: "active" for name in ACTIVATION_PHASES[:done]
            }
            nav["message"] = f"正在激活导航（{done}/{ACTIVATION_TOTAL}）：{phase}"

    def _nav_control(self, now_mono: float) -> tuple[float, float, float]:
        path = self._nav_path
        if not path:
            return 0.0, 0.0, 0.0
        x = self._pose["x"]
        y = self._pose["y"]
        yaw = self._pose["yaw"]
        index = self._nav_path_index
        while index < len(path):
            tx, ty = path[index]
            if math.hypot(tx - x, ty - y) < 0.15:
                index += 1
                continue
            break
        self._nav_path_index = index
        if index >= len(path):
            self._complete_nav(now_mono)
            return 0.0, 0.0, 0.0
        tx, ty = path[index]
        dist = math.hypot(tx - x, ty - y)
        remaining = dist
        for i in range(index, len(path) - 1):
            remaining += math.hypot(
                path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]
            )
        heading = math.atan2(ty - y, tx - x)
        err = wrap_angle(heading - yaw)
        wz = clamp(err * 2.5, -LIMITS["max_angular"], LIMITS["max_angular"])
        speed = min(LIMITS["max_linear"], max(0.10, 1.2 * dist))
        wx = speed * math.cos(heading)
        wy = speed * math.sin(heading)
        vx = wx * math.cos(yaw) + wy * math.sin(yaw)
        vy = -wx * math.sin(yaw) + wy * math.cos(yaw)
        elapsed = 0.0
        if self._nav_started_mono is not None:
            elapsed = max(0.0, now_mono - self._nav_started_mono)
        self._nav["distance_remaining"] = round(remaining, 2)
        self._nav["navigation_time_s"] = round(elapsed, 2)
        self._nav["message"] = (
            f"前往目的地：剩余 {remaining:.2f} m / 已用 {elapsed:.0f} s"
        )
        return vx, vy, wz

    def _complete_nav(self, now_mono: float) -> None:
        elapsed = 0.0
        if self._nav_started_mono is not None:
            elapsed = max(0.0, now_mono - self._nav_started_mono)
        self._nav["state"] = "succeeded"
        self._nav["goal_state"] = "succeeded"
        self._nav["message"] = "已到达目的地"
        self._nav["distance_remaining"] = 0.0
        self._nav["navigation_time_s"] = round(elapsed, 2)
        self._nav_path = None
        self._nav_path_index = 1
        self._speed = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._log("info", "导航完成：已到达目的地")
        self._publish_state()

    def _fail_nav(self, message: str) -> None:
        elapsed = 0.0
        if self._nav_started_mono is not None:
            elapsed = max(0.0, time.monotonic() - self._nav_started_mono)
        self._nav["state"] = "failed"
        self._nav["goal_state"] = "aborted"
        self._nav["message"] = message
        self._nav["distance_remaining"] = None
        self._nav["navigation_time_s"] = round(elapsed, 2)
        self._nav_path = None
        self._nav_path_index = 1
        self._emit_path([])
        self._log("warn", f"导航失败：{message}")
        self._publish_state()

    def _cancel_nav(self, message: str) -> None:
        if self._nav["state"] != "goal_active":
            return
        elapsed = 0.0
        if self._nav_started_mono is not None:
            elapsed = max(0.0, time.monotonic() - self._nav_started_mono)
        self._nav["state"] = "canceled"
        self._nav["goal_state"] = "canceled"
        self._nav["message"] = message
        self._nav["navigation_time_s"] = round(elapsed, 2)
        self._nav["distance_remaining"] = None
        self._nav_path = None
        self._nav_path_index = 1
        self._speed = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._emit_path([])
        self._log("info", f"导航目标已取消：{message}")

    def _emit_path(self, path: list[tuple[float, float]]) -> None:
        flat: list[float] = []
        for x, y in path:
            flat.append(round(x, 3))
            flat.append(round(y, 3))
        self._path_stamp = time.time()
        self._hub.publish(
            "path",
            {
                "stamp": round(self._path_stamp, 3),
                "frame": "map",
                "points": flat,
            },
        )

    def _emit_map(self) -> None:
        """推送 ``map`` 事件（地图内容/版本变化时）。"""
        if self._map is None:
            return
        self._hub.publish(
            "map", {"meta": self._map.to_meta(self._map_available)}
        )

    # ===============================================================
    # 事件发布
    # ===============================================================
    def _publish_state(self) -> None:
        self._hub.publish("state", {"state": self._build_state()})

    def _publish_session(self) -> None:
        self._hub.publish("session", {"session": self._session_snapshot()})

    def _build_state(self) -> dict:
        now = time.time()
        session = self._session_snapshot()
        nav = self._nav_snapshot()
        running = session["state"] == "running"
        map_meta = (
            self._map.to_meta(self._map_available)
            if self._map is not None
            else self._empty_map_meta()
        )

        robot = None
        if running:
            map_frame = session["mode"] in ("slam", "nav") and self._map_available
            scan_age = (
                round(max(0.0, now - self._scan_stamp), 3)
                if self._scan_stamp
                else None
            )
            robot = {
                "frame": "map" if map_frame else "odom",
                "x": round(self._pose["x"], 3),
                "y": round(self._pose["y"], 3),
                "yaw": round(wrap_angle(self._pose["yaw"]), 4),
                "stamp": round(now, 3),
                "speed": {
                    "vx": round(self._speed["vx"], 3),
                    "vy": round(self._speed["vy"], 3),
                    "wz": round(self._speed["wz"], 3),
                },
                "imu": {
                    "yaw_deg": round(math.degrees(self._pose["yaw"]) % 360.0, 1),
                    "age_s": 0.02,
                },
                "scan": {
                    "front_m": self._scan_front,
                    "min_m": self._scan_min,
                    "age_s": scan_age,
                },
            }

        return {
            "api": API_VERSION,
            "server_time": round(now, 3),
            "uptime_s": round(time.time() - self._created_wall, 1),
            "session": session,
            "ros": self._build_ros(session, nav, now),
            "limits": dict(LIMITS),
            "robot": robot,
            "nav": nav,
            "map": map_meta,
            "estop": self._estop_snapshot(),
            "teleop": {
                "active": bool(self._teleop["active"]),
                "vx": round(self._teleop["vx"], 3),
                "vy": round(self._teleop["vy"], 3),
                "wz": round(self._teleop["wz"], 3),
                "last_cmd": self._teleop["last_cmd"],
            },
            "peripherals": self._peripherals_snapshot(),
            "diagnostics": self._build_diagnostics(session),
            "logs": [dict(entry) for entry in list(self._logs)[-20:]],
        }

    def _build_ros(self, session: dict, nav: dict, now: float) -> dict:
        running = session["state"] == "running"
        mode = session["mode"]
        nav_state = nav.get("state")
        ready = {
            "base": running,
            "teleop": running,
            "map": running and self._map_available,
            "localization": bool(
                running
                and mode == "nav"
                and nav_state
                in ("activating", "ready", "goal_active", "succeeded", "failed", "canceled")
            ),
            "nav2": bool(
                running
                and mode == "nav"
                and nav_state
                in ("ready", "goal_active", "succeeded", "failed", "canceled")
            ),
            "diagnostics": True,
        }
        if not running:
            nodes = 0
        elif mode == "base":
            nodes = 8
        elif mode == "slam":
            nodes = 14
        else:
            nodes = 27
        map_age = (
            round(max(0.0, now - self._map_stamp), 3)
            if running and self._map_stamp
            else None
        )
        return {
            "ok": running,
            "nodes": nodes,
            "ready": ready,
            "rates": {
                "scan_hz": 5.0 if running else None,
                "odom_hz": 30.0 if running else None,
                "map_age_s": map_age,
                "diagnostics_age_s": 0.2 if running else None,
            },
        }

    def _build_diagnostics(self, session: dict) -> dict:
        running = session["state"] == "running"
        now = time.time()
        base_level = "ok" if running else "unknown"
        base_msg = "OK (30.0 Hz)" if running else "会话未运行"
        lidar_msg = "OK (360 点 @ 5 Hz)" if running else "会话未运行"
        imu_msg = "OK (100 Hz，偏航已融合)" if running else "会话未运行"
        estop_active = bool(self._estop["active"])
        items = [
            {
                "name": "Raspbot Base: Odometry",
                "level": base_level,
                "message": base_msg,
                "hardware_id": "base",
            },
            {
                "name": "Raspbot Lidar: LD19 Scan",
                "level": base_level,
                "message": lidar_msg,
                "hardware_id": "lidar",
            },
            {
                "name": "Raspbot IMU: HWT101",
                "level": base_level,
                "message": imu_msg,
                "hardware_id": "imu",
            },
            {
                "name": "PC: CPU Load",
                "level": "ok",
                "message": f"{9.5 + 2.0 * math.sin(now / 30.0):.1f}% (4 核)",
                "hardware_id": "pc_cpu",
            },
            {
                "name": "PC: Temperature",
                "level": "ok",
                "message": f"{46.0 + math.sin(now / 50.0):.1f} °C",
                "hardware_id": "pc_temp",
            },
            {
                "name": "Raspbot Safety: E-Stop",
                "level": "warn" if estop_active else "ok",
                "message": "已锁存，运动被禁止" if estop_active else "OK（未锁存）",
                "hardware_id": "estop",
            },
        ]
        counts = {"ok": 0, "warn": 0, "error": 0}
        unknown = 0
        for item in items:
            item_level = item["level"]
            if item_level in counts:
                counts[item_level] += 1
            else:
                unknown += 1
        # 聚合：error > warn > unknown > ok
        if counts["error"]:
            level = "error"
        elif counts["warn"]:
            level = "warn"
        elif unknown:
            level = "unknown"
        else:
            level = "ok"
        return {
            "level": level,
            "counts": counts,
            "items": items,
            "age_s": 0.2 if running else None,
        }

    def _build_ros_debug(self) -> dict:
        session = self._session
        if session["state"] != "running":
            return {"nodes": [], "topics": []}
        mode = session["mode"]
        nodes = [
            "/robot_state_publisher",
            "/raspbot_base",
            "/ld19_lidar",
            "/hwt101_imu",
            "/twist_mux",
            "/cmd_vel_safety",
        ]
        topics = [
            ("/scan", ["sensor_msgs/msg/LaserScan"]),
            ("/odom", ["nav_msgs/msg/Odometry"]),
            ("/imu/data", ["sensor_msgs/msg/Imu"]),
            ("/cmd_vel", ["geometry_msgs/msg/Twist"]),
            ("/cmd_vel_teleop", ["geometry_msgs/msg/Twist"]),
            ("/cmd_vel_safe", ["geometry_msgs/msg/Twist"]),
            ("/tf", ["tf2_msgs/msg/TFMessage"]),
            ("/diagnostics", ["diagnostic_msgs/msg/DiagnosticArray"]),
        ]
        if mode in ("slam", "nav"):
            nodes.append("/map_server")
            topics.append(("/map", ["nav_msgs/msg/OccupancyGrid"]))
        if mode == "slam":
            nodes.append("/slam_toolbox")
            topics.extend(
                [
                    ("/map_updates", ["map_msgs/msg/OccupancyGridUpdate"]),
                    ("/pose", ["geometry_msgs/msg/PoseWithCovarianceStamped"]),
                ]
            )
        if mode == "nav":
            nodes.extend(
                [
                    "/amcl",
                    "/controller_server",
                    "/planner_server",
                    "/behavior_server",
                    "/bt_navigator",
                    "/collision_monitor",
                    "/lifecycle_manager_nav2",
                ]
            )
            topics.extend(
                [
                    ("/initialpose", ["geometry_msgs/msg/PoseWithCovarianceStamped"]),
                    ("/plan", ["nav_msgs/msg/Path"]),
                    (
                        "/navigate_to_pose/_action/status",
                        ["action_msgs/msg/GoalStatusArray"],
                    ),
                ]
            )
        return {
            "nodes": nodes,
            "topics": [
                {"name": name, "types": types} for name, types in topics
            ],
        }

    # ===============================================================
    # 会话控制
    # ===============================================================
    def _sleep_abortable(self, seconds: float) -> bool:
        deadline = time.monotonic() + seconds
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                return self._stop.is_set()
            if self._stop.wait(min(0.05, remain)):
                return True

    def _set_session_phase(self, state: str, message: str, progress: float) -> None:
        with self._lock:
            self._session.update(
                state=state,
                message=message,
                progress=round(clamp(progress, 0.0, 1.0), 2),
                since=time.time(),
            )
            self._publish_session()

    def _push_bringup_logs(self, phase: str, mode: str = "base", map_path: str | None = None) -> None:
        now = time.time()
        if phase == "stop":
            lines = [
                "[INFO] [launch]: 收到 SIGINT，正在关闭 bringup 进程组…",
                "[INFO] [raspbot_base-2]: 底盘驱动已停止，速度归零",
                "[INFO] [ld19_lidar-3]: 雷达已关闭",
                "[INFO] [launch]: 进程组已退出（code=0）",
            ]
        else:
            mode_arg = {"base": "none", "slam": "slam", "nav": "nav"}[mode]
            lines = [
                "[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-11-24-23-00-00-000000",
                "[INFO] [launch]: Default logging verbosity is set to INFO",
                f"[INFO] [bringup.launch.py]: mode:={mode_arg} use_mock:=False",
                "[INFO] [robot_state_publisher-1]: started",
                "[INFO] [raspbot_base-2]: 底盘驱动已启动（I2C bus=1 addr=0x2B）",
                "[INFO] [ld19_lidar-3]: 雷达就绪，/scan 已发布",
                "[INFO] [hwt101_imu-4]: IMU 就绪，零偏标定完成",
                "[INFO] [twist_mux-5]: 遥控/导航速度切换就绪",
            ]
            if mode == "slam":
                lines.append("[INFO] [slam_toolbox-6]: 建图已启动（map→odom 由 slam_toolbox 发布）")
            elif mode == "nav":
                lines.extend(
                    [
                        "[INFO] [map_server-6]: 加载地图: " + (map_path or ""),
                        "[INFO] [amcl-7]: 等待初始位姿 /initialpose",
                        "[INFO] [lifecycle_manager_nav2-8]: 等待手动激活（autostart:=false）",
                    ]
                )
        for offset, line in enumerate(lines):
            self._session_logs.append({"ts": round(now + offset * 0.01, 3), "line": line})

    def _start_session(self, body: dict) -> WireResponse:
        mode = body.get("mode")
        if mode not in ("base", "slam", "nav"):
            raise DemoHttpError(
                400,
                "bad_request",
                "mode 必须是 base / slam / nav 之一",
                {"field": "mode"},
            )
        map_arg = body.get("map")
        map_path: str | None = None
        if mode == "nav":
            if not isinstance(map_arg, str) or not map_arg.strip():
                raise DemoHttpError(
                    400,
                    "bad_request",
                    "导航模式必须提供 map（.yaml 绝对路径）",
                    {"field": "map"},
                )
            candidate = Path(map_arg.strip())
            if candidate.suffix.lower() not in (".yaml", ".yml"):
                raise DemoHttpError(
                    400, "bad_request", "map 必须是 .yaml 文件路径", {"field": "map"}
                )
            if not candidate.is_absolute():
                raise DemoHttpError(
                    400, "bad_request", "map 必须是绝对路径", {"field": "map"}
                )
            if not candidate.is_file():
                raise DemoHttpError(
                    400,
                    "bad_request",
                    f"地图文件不存在：{candidate}",
                    {"field": "map"},
                )
            try:
                with candidate.open("rb"):
                    pass
            except OSError as exc:
                raise DemoHttpError(
                    400,
                    "bad_request",
                    f"地图文件不可读：{candidate}（{exc}）",
                    {"field": "map"},
                ) from exc
            map_path = str(candidate)

        with self._lock:
            if self._estop["active"]:
                raise DemoHttpError(
                    409, "estop_active", "急停已锁存，禁止启动会话"
                )
            if self._session_busy or self._session["state"] in ("starting", "stopping"):
                raise DemoHttpError(409, "busy", "会话正在启动/停止，请稍候")
            prev_running = self._session["state"] == "running"
            self._session_busy = True
            label = MODE_LABELS[mode]
            self._session.update(
                mode=mode,
                state="starting",
                mock=bool(body.get("use_mock", False)),
                managed=True,
                external=False,
                since=time.time(),
                progress=0.0,
                message=f"正在启动{label}…",
                error=None,
                map=map_path if mode == "nav" else None,
            )
            if prev_running:
                self._session["message"] = f"正在切换运行模式：停止当前进程并启动{label}…"
            self._log(
                "info",
                f"会话启动请求：mode={mode}"
                + (f"，map={map_path}" if map_path else ""),
            )
            self._publish_session()
            self._publish_state()
            snapshot = self._session_snapshot()
        thread = threading.Thread(
            target=self._run_start,
            args=(mode, map_path, prev_running),
            name="demo-session-start",
            daemon=True,
        )
        self._session_thread = thread
        thread.start()
        return self._json_response(202, {"ok": True, "session": snapshot})

    def _run_start(self, mode: str, map_path: str | None, prev_running: bool) -> None:
        label = MODE_LABELS[mode]
        try:
            if prev_running:
                self._set_session_phase(
                    "stopping", "正在切换运行模式：停止当前进程…", 0.15
                )
                with self._lock:
                    self._push_bringup_logs("stop")
                if self._sleep_abortable(1.0 + self._random.random() * 0.6):
                    return
                with self._lock:
                    self._reset_runtime()
                    self._session["mode"] = mode
                    self._session["state"] = "starting"
                    self._session["progress"] = 0.05
                    self._session["message"] = f"正在启动{label}…"
                    self._session["since"] = time.time()
                    self._publish_session()

            self._set_session_phase("starting", f"正在启动{label}…", 0.45)
            with self._lock:
                self._push_bringup_logs("start", mode, map_path)
            if self._sleep_abortable(1.0 + self._random.random() * 1.0):
                return

            with self._lock:
                self._prepare_mode(mode, map_path)
            if self._sleep_abortable(0.15):
                return

            with self._lock:
                self._session.update(
                    state="running",
                    progress=1.0,
                    message=f"{label}运行中",
                    since=time.time(),
                    error=None,
                )
                if mode == "slam":
                    self._nav = self._empty_nav()
                self._log("info", f"{label}会话已就绪")
                self._publish_session()
                self._publish_state()
        except MapFormatError as exc:
            self._enter_session_error(f"加载地图失败：{exc}")
        except Exception as exc:  # pragma: no cover - 防御性
            log.exception("演示会话启动异常")
            self._enter_session_error(f"内部错误：{exc.__class__.__name__}: {exc}")
        finally:
            with self._lock:
                self._session_busy = False

    def _enter_session_error(self, message: str) -> None:
        with self._lock:
            self._session.update(
                state="error",
                progress=0.0,
                message=f"启动失败：{message}",
                error=message,
                since=time.time(),
            )
            self._log("error", f"会话启动失败：{message}")
            self._publish_session()
            self._publish_state()

    def _prepare_mode(self, mode: str, map_path: str | None) -> None:
        """切换到目标模式（调用方持有锁）。"""
        self._stop_teleop_locked()
        self._cancel_nav("会话切换，目标已取消")
        self._nav = self._empty_nav()
        self._nav_path = None
        self._nav_path_index = 1
        self._speed = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._png_cache = None
        if mode == "base":
            self._map = None
            self._map_available = False
            self._map_stamp = None
            self._blocked = None
        elif mode == "slam":
            grid = MapGrid.from_scene(self._scene, unknown=True)
            self._map_version_seq += 1
            grid.version = self._map_version_seq
            grid.stamp = time.time()
            self._map = grid
            self._map_available = False
            self._map_stamp = grid.stamp
            self._blocked = None
            self._scan_front = None
            self._scan_min = None
            self._emit_map()
        elif mode == "nav":
            if not map_path:
                raise MapFormatError("导航模式缺少地图")
            grid = load_map_grid(map_path)
            self._map_version_seq += 1
            grid.version = self._map_version_seq
            grid.stamp = time.time()
            self._map = grid
            self._map_available = True
            self._map_stamp = grid.stamp
            self._blocked = blocked_from_map_pixels(
                grid.pixels, grid.width, grid.height, unknown_is_blocked=True
            )
            self._nav = self._empty_nav()
            self._nav["state"] = "waiting_initialpose"
            self._nav["message"] = "请在地图上设置初始位姿"
            self._emit_map()
            self._png_cache = None

    def _reset_runtime(self) -> None:
        """停止后清空运行期状态（调用方持有锁）。"""
        self._stop_teleop_locked()
        self._nav = self._empty_nav()
        self._nav_path = None
        self._nav_path_index = 1
        self._nav_fail_at = None
        self._activation_duration = 0.0
        self._speed = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._map = None
        self._map_available = False
        self._map_stamp = None
        self._blocked = None
        self._png_cache = None
        self._emit_path([])

    def _stop_teleop_locked(self) -> None:
        self._teleop.update(active=False, vx=0.0, vy=0.0, wz=0.0)
        self._last_cmd_mono = None
        self._speed = {"vx": 0.0, "vy": 0.0, "wz": 0.0}

    def _stop_session(self) -> WireResponse:
        with self._lock:
            state = self._session["state"]
            if self._session_busy or state in ("starting", "stopping"):
                raise DemoHttpError(409, "busy", "会话正在启动/停止，请稍候")
            if state == "idle":
                return self._json_response(200, {"ok": True})
            self._session_busy = True
            self._session.update(
                state="stopping",
                progress=0.2,
                message="正在停止会话…",
                since=time.time(),
            )
            self._log("info", "会话停止请求")
            self._publish_session()
            self._publish_state()
            snapshot = self._session_snapshot()
        thread = threading.Thread(
            target=self._run_stop, name="demo-session-stop", daemon=True
        )
        self._session_thread = thread
        thread.start()
        return self._json_response(202, {"ok": True, "session": snapshot})

    def _run_stop(self) -> None:
        try:
            with self._lock:
                self._push_bringup_logs("stop")
            if self._sleep_abortable(0.9 + self._random.random() * 0.7):
                return
            with self._lock:
                self._reset_runtime()
                self._session.update(
                    mode="idle",
                    state="idle",
                    progress=0.0,
                    message="空闲：可选择运行模式",
                    error=None,
                    map=None,
                    since=time.time(),
                )
                self._log("info", "会话已停止")
                self._publish_session()
                self._publish_state()
        finally:
            with self._lock:
                self._session_busy = False

    # ===============================================================
    # 遥控 / 急停 / 导航命令
    # ===============================================================
    def _teleop_cmd(self, body: dict) -> WireResponse:
        with self._lock:
            if self._estop["active"]:
                raise DemoHttpError(409, "estop_active", "急停已锁存，禁止遥控")
            if self._session["state"] != "running":
                raise DemoHttpError(
                    409, "session_not_running", "会话未运行，无法遥控"
                )
            vx = clamp(
                self._num(body, "vx", 0.0),
                -LIMITS["max_linear"],
                LIMITS["max_linear"],
            )
            vy = clamp(
                self._num(body, "vy", 0.0),
                -LIMITS["max_lateral"],
                LIMITS["max_lateral"],
            )
            wz = clamp(
                self._num(body, "wz", 0.0),
                -LIMITS["max_angular"],
                LIMITS["max_angular"],
            )
            seq = body.get("seq")
            if seq is not None:
                if isinstance(seq, bool) or not isinstance(seq, int):
                    raise DemoHttpError(
                        400, "bad_request", "seq 必须是整数", {"field": "seq"}
                    )
                if self._teleop_seq is not None and seq <= self._teleop_seq:
                    return self._json_response(200, {"ok": True})  # 丢弃过期帧
                self._teleop_seq = seq
            if self._nav["state"] == "goal_active":
                self._cancel_nav("遥控接管，已取消导航目标")
            self._teleop.update(active=True, vx=vx, vy=vy, wz=wz)
            self._last_cmd_mono = time.monotonic()
            self._teleop["last_cmd"] = round(time.time(), 3)
        return self._json_response(200, {"ok": True})

    def _teleop_stop(self) -> WireResponse:
        with self._lock:
            self._stop_teleop_locked()
        return self._json_response(200, {"ok": True})

    def _set_estop(self, body: dict) -> WireResponse:
        active = body.get("active")
        if not isinstance(active, bool):
            raise DemoHttpError(
                400, "bad_request", "active 必须是布尔值", {"field": "active"}
            )
        with self._lock:
            if active:
                self._estop["active"] = True
                self._estop["since"] = round(time.time(), 3)
                self._stop_teleop_locked()
                self._cancel_nav("急停触发，已取消导航目标")
                self._log("warn", "急停已锁存：运动指令被禁止")
            elif self._estop["active"]:
                self._estop["active"] = False
                self._estop["since"] = None
                self._log("info", "急停已解除")
            snapshot = self._estop_snapshot()
            self._publish_state()
        return self._json_response(200, {"ok": True, "estop": snapshot})

    def _initialpose(self, body: dict) -> WireResponse:
        x = self._num(body, "x")
        y = self._num(body, "y")
        yaw = self._num(body, "yaw", 0.0)
        with self._lock:
            if not (
                self._session["mode"] == "nav"
                and self._session["state"] == "running"
            ):
                raise DemoHttpError(
                    409, "nav_not_ready", "仅在导航模式运行时可设置初始位姿"
                )
            grid = self._map
            if grid is None or not self._map_available:
                raise DemoHttpError(409, "no_map", "尚未加载地图")
            if grid.world_to_cell(x, y) is None:
                raise DemoHttpError(
                    400,
                    "bad_request",
                    "初始位姿超出地图范围",
                    {"field": "x"},
                )
            self._cancel_nav("重新设置初始位姿，目标已取消")
            self._pose = {"x": x, "y": y, "yaw": wrap_angle(yaw)}
            self._nav["initialpose"] = {
                "x": round(x, 3),
                "y": round(y, 3),
                "yaw": round(wrap_angle(yaw), 4),
                "stamp": round(time.time(), 3),
            }
            self._nav["state"] = "activating"
            self._nav["message"] = "正在激活导航（等待 TF map→base_link）…"
            self._nav["lifecycle"] = {}
            self._nav["goal"] = None
            self._nav["goal_state"] = "idle"
            self._nav["distance_remaining"] = None
            self._nav["navigation_time_s"] = None
            self._nav["recoveries"] = None
            self._nav["activation"] = {
                "done": 0,
                "total": ACTIVATION_TOTAL,
                "phase": "等待 TF map→base_link",
            }
            self._activation_start = time.monotonic()
            self._activation_duration = 2.0 + self._random.random() * 1.0
            self._log("info", f"已设置初始位姿: ({x:.2f}, {y:.2f}, {yaw:.2f})")
            self._publish_state()
            nav = self._nav_snapshot()
        return self._json_response(200, {"ok": True, "nav": nav})

    def _nav_activate(self) -> WireResponse:
        with self._lock:
            if not (
                self._session["mode"] == "nav"
                and self._session["state"] == "running"
            ):
                raise DemoHttpError(409, "nav_not_ready", "当前不在导航模式")
            self._cancel_nav("重新激活 Nav2，目标已取消")
            self._nav["state"] = "activating"
            self._nav["message"] = "正在重新激活导航…"
            self._nav["lifecycle"] = {}
            self._nav["activation"] = {
                "done": 0,
                "total": ACTIVATION_TOTAL,
                "phase": "controller_server",
            }
            self._activation_start = time.monotonic()
            self._activation_duration = 2.0 + self._random.random() * 1.0
            self._log("info", "手动触发 Nav2 生命周期激活")
            self._publish_state()
        return self._json_response(202, {"ok": True})

    def _goal(self, body: dict) -> WireResponse:
        x = self._num(body, "x")
        y = self._num(body, "y")
        yaw_value = body.get("yaw")
        if yaw_value is None:
            yaw = self._pose["yaw"]
        else:
            if isinstance(yaw_value, bool) or not isinstance(
                yaw_value, (int, float)
            ):
                raise DemoHttpError(
                    400, "bad_request", "yaw 必须是数值或 null", {"field": "yaw"}
                )
            yaw = float(yaw_value)
            if not math.isfinite(yaw):
                raise DemoHttpError(
                    400, "bad_request", "yaw 必须是有限数值", {"field": "yaw"}
                )
        with self._lock:
            if self._estop["active"]:
                raise DemoHttpError(409, "estop_active", "急停已锁存，禁止发送目标")
            session = self._session
            if not (session["mode"] == "nav" and session["state"] == "running"):
                raise DemoHttpError(
                    409, "nav_not_ready", "需要导航模式运行中才能发送目标"
                )
            nav_state = self._nav["state"]
            if nav_state in ("unavailable", "waiting_initialpose", "activating"):
                raise DemoHttpError(
                    409,
                    "nav_not_ready",
                    "导航尚未就绪：请先设置初始位姿",
                    {"nav_state": nav_state},
                )
            if nav_state == "goal_active":
                raise DemoHttpError(409, "goal_active", "已有导航目标，请先取消")
            grid = self._map
            if grid is None or not self._map_available or self._blocked is None:
                raise DemoHttpError(409, "no_map", "尚未加载地图")
            if grid.world_to_cell(x, y) is None:
                raise DemoHttpError(
                    400, "bad_request", "目标点超出地图范围", {"field": "x"}
                )
            goal_id = uuid.uuid4().hex
            goal = {
                "x": round(x, 3),
                "y": round(y, 3),
                "yaw": round(wrap_angle(yaw), 4),
                "stamp": round(time.time(), 3),
                "goal_id": goal_id,
            }
            path = plan_path(
                self._blocked,
                grid.width,
                grid.height,
                (self._pose["x"], self._pose["y"]),
                (x, y),
                grid.resolution,
                (grid.origin[0], grid.origin[1]),
            )
            self._goal_id = goal_id
            self._nav["goal"] = goal
            self._nav["goal_state"] = "accepted"
            self._nav["state"] = "goal_active"
            self._nav["recoveries"] = 0
            self._nav["navigation_time_s"] = 0.0
            self._nav_path = None
            self._nav_path_index = 1
            self._nav_fail_at = None
            self._nav_started_mono = time.monotonic()
            if path is None:
                self._nav["message"] = "正在规划路径…"
                self._nav["distance_remaining"] = None
                self._nav_fail_at = time.monotonic() + 0.3
                self._log(
                    "warn",
                    f"导航目标不可达（无可行路径）: ({x:.2f}, {y:.2f})",
                )
            else:
                total = path_length(path)
                self._nav_path = path
                self._nav_path_index = 1
                self._nav["goal_state"] = "active"
                self._nav["distance_remaining"] = round(total, 2)
                self._nav["message"] = f"前往目的地：剩余 {total:.2f} m"
                self._emit_path(path)
                self._log(
                    "info",
                    f"收到导航目标 ({x:.2f}, {y:.2f})：路径 {len(path)} 点 / {total:.2f} m",
                )
            self._publish_state()
            goal_copy = dict(goal)
        return self._json_response(202, {"ok": True, "goal": goal_copy, "goal_id": goal_id})

    def _goal_cancel(self) -> WireResponse:
        with self._lock:
            self._cancel_nav("用户取消导航目标")
            self._publish_state()
        return self._json_response(200, {"ok": True})

    def _actuator(self, kind: str, body: dict) -> WireResponse:
        with self._lock:
            if self._session["state"] != "running":
                raise DemoHttpError(
                    409, "session_not_running", "会话未运行，外设不可用"
                )
            if kind == "buzzer":
                on = body.get("on")
                if not isinstance(on, bool):
                    raise DemoHttpError(
                        400, "bad_request", "on 必须是布尔值", {"field": "on"}
                    )
                self._peripherals["buzzer"] = on
                self._log("info", f"蜂鸣器 {'开' if on else '关'}")
            elif kind == "led":
                for channel in ("r", "g", "b"):
                    value = clamp(self._num(body, channel, 0.0), 0.0, 1.0)
                    self._peripherals["led"][channel] = round(value, 3)
                led = self._peripherals["led"]
                self._log(
                    "info",
                    f"RGB 灯: ({led['r']:.2f}, {led['g']:.2f}, {led['b']:.2f})",
                )
            elif kind == "servo":
                pan = clamp(self._num(body, "pan", self._peripherals["servo"]["pan"]), 0.0, 180.0)
                tilt = clamp(self._num(body, "tilt", self._peripherals["servo"]["tilt"]), 0.0, 110.0)
                self._peripherals["servo"] = {
                    "pan": round(pan, 1),
                    "tilt": round(tilt, 1),
                }
                self._log("info", f"云台: pan={pan:.0f}° tilt={tilt:.0f}°")
            else:  # pragma: no cover - 路由保证
                raise DemoHttpError(404, "not_found", "未知外设")
            snapshot = self._peripherals_snapshot()
            self._publish_state()
        return self._json_response(200, {"ok": True, "peripherals": snapshot})

    # ===============================================================
    # 地图
    # ===============================================================
    def _map_meta(self) -> WireResponse:
        with self._lock:
            if self._map is None:
                return self._json_response(200, self._empty_map_meta())
            return self._json_response(
                200, self._map.to_meta(self._map_available)
            )

    def _map_png(self) -> WireResponse:
        with self._lock:
            grid = self._map
            available = self._map_available
            if grid is None or not available:
                raise DemoHttpError(404, "no_map", "当前没有可用地图")
            cache = self._png_cache
            version = grid.version
        if cache is not None and cache[0] == version:
            return self._binary_response(200, "image/png", cache[1])
        png = map_png(grid)
        with self._lock:
            self._png_cache = (version, png)
        return self._binary_response(200, "image/png", png)

    def _maps(self) -> WireResponse:
        return self._json_response(200, self._list_maps())

    def _preview_png(self, name: str) -> WireResponse:
        if not valid_map_name(name):
            raise DemoHttpError(404, "not_found", "地图名称非法")
        yaml_path = self.demo_maps_dir / f"{name}.yaml"
        if not yaml_path.is_file():
            raise DemoHttpError(404, "not_found", f"地图不存在：{name}")
        try:
            grid = load_map_grid(yaml_path)
        except (MapFormatError, OSError) as exc:
            raise DemoHttpError(404, "not_found", f"地图无法解析：{exc}") from exc
        return self._binary_response(200, "image/png", map_png(grid, level=3))

    def _save_map(self, body: dict) -> WireResponse:
        name = body.get("name")
        if not valid_map_name(name):
            raise DemoHttpError(
                400,
                "bad_request",
                "地图名只能包含字母/数字/下划线/点/短横线且长度 1-40",
                {"field": "name"},
            )
        overwrite = bool(body.get("overwrite", False))
        with self._lock:
            if self._session["state"] != "running":
                raise DemoHttpError(409, "session_idle", "会话未运行，无法保存地图")
            if self._map is None or not self._map_available:
                raise DemoHttpError(409, "no_map", "当前没有可保存的地图")
            source = self._map
            snapshot = MapGrid(
                source.width,
                source.height,
                source.resolution,
                source.origin,
                source.pixels,
                source.version,
            )
        yaml_path = self.demo_maps_dir / f"{name}.yaml"
        pgm_path = self.demo_maps_dir / f"{name}.pgm"
        if (yaml_path.exists() or pgm_path.exists()) and not overwrite:
            raise DemoHttpError(
                409,
                "map_exists",
                f"同名地图已存在：{name}（overwrite=true 可覆盖）",
                {"field": "name"},
            )
        try:
            self._ensure_demo_maps_dir()
            write_pgm(pgm_path, snapshot.width, snapshot.height, snapshot.rows_top_first())
            write_map_yaml(
                yaml_path,
                f"{name}.pgm",
                snapshot.resolution,
                (snapshot.origin[0], snapshot.origin[1], snapshot.origin[2]),
            )
            mtime = os.path.getmtime(yaml_path)
        except OSError as exc:
            raise DemoHttpError(500, "internal_error", f"写地图文件失败：{exc}") from exc
        self._log("info", f"地图已保存：{name}")
        return self._json_response(
            200,
            {
                "ok": True,
                "name": name,
                "yaml": str(yaml_path.resolve()),
                "pgm": str(pgm_path.resolve()),
                "width": snapshot.width,
                "height": snapshot.height,
                "resolution": snapshot.resolution,
                "mtime": round(mtime, 3),
            },
        )

    # ===============================================================
    # 日志 / 调试
    # ===============================================================
    def _logs_response(self, query: dict) -> WireResponse:
        limit = 200
        raw = (query.get("limit") or [None])[0]
        if raw is not None:
            try:
                limit = int(raw)
            except (TypeError, ValueError):
                raise DemoHttpError(
                    400, "bad_request", "limit 必须是整数", {"field": "limit"}
                ) from None
        limit = max(1, min(300, limit))
        with self._lock:
            entries = [dict(entry) for entry in list(self._logs)[-limit:]]
        return self._json_response(200, {"logs": entries})

    def _session_logs_response(self, query: dict) -> WireResponse:
        limit = 200
        raw = (query.get("limit") or [None])[0]
        if raw is not None:
            try:
                limit = int(raw)
            except (TypeError, ValueError):
                raise DemoHttpError(
                    400, "bad_request", "limit 必须是整数", {"field": "limit"}
                ) from None
        limit = max(1, min(500, limit))
        with self._lock:
            entries = [dict(entry) for entry in list(self._session_logs)[-limit:]]
        return self._json_response(200, {"logs": entries})

    # ===============================================================
    # HTTP 后端接口
    # ===============================================================
    def request(
        self,
        method: str,
        path_and_query: str,
        body: bytes = b"",
        headers: dict | None = None,
    ) -> WireResponse:
        parsed = urlsplit(path_and_query)
        path = parsed.path
        query = parse_qs(parsed.query)
        try:
            if not path.startswith("/api/"):
                raise DemoHttpError(404, "not_found", "接口不存在")
            route = path[len("/api") :]
            if method in ("GET", "HEAD"):
                return self._handle_get(route, query)
            if method == "POST":
                return self._handle_post(route, body)
            raise DemoHttpError(
                405, "method_not_allowed", f"不支持的方法 {method}"
            )
        except DemoHttpError as exc:
            log.debug("演示接口 %s %s -> %s %s", method, path, exc.status, exc.code)
            return self._json_response(exc.status, exc.payload())
        except Exception:  # pragma: no cover - 防御性
            log.exception("演示后端内部错误: %s %s", method, path)
            return self._json_response(
                500,
                {
                    "error": {
                        "code": "internal_error",
                        "message": "演示后端内部错误，详见 PC 控制台日志",
                    }
                },
            )

    def _handle_get(self, route: str, query: dict) -> WireResponse:
        if route == "/health":
            now = time.time()
            return self._json_response(
                200,
                {
                    "ok": True,
                    "name": NAME,
                    "version": VERSION,
                    "api": API_VERSION,
                    "server_time": round(now, 3),
                    "uptime_s": round(now - self._created_wall, 1),
                    "demo": True,
                },
            )
        if route == "/state":
            with self._lock:
                return self._json_response(200, self._build_state())
        if route == "/map/meta":
            return self._map_meta()
        if route == "/map.png":
            return self._map_png()
        if route == "/maps":
            return self._maps()
        if route.startswith("/maps/") and route.endswith("/preview.png"):
            name = route[len("/maps/") : -len("/preview.png")]
            return self._preview_png(name)
        if route == "/logs":
            return self._logs_response(query)
        if route == "/session/logs":
            return self._session_logs_response(query)
        if route == "/ros":
            with self._lock:
                return self._json_response(200, self._build_ros_debug())
        if route in _POST_ROUTES:
            raise DemoHttpError(
                405, "method_not_allowed", f"{route} 只接受 POST"
            )
        raise DemoHttpError(404, "not_found", f"未知接口 /api{route}")

    def _handle_post(self, route: str, body: bytes) -> WireResponse:
        if route == "/session/start":
            return self._start_session(self._json_body(body))
        if route == "/session/stop":
            return self._stop_session()
        if route == "/teleop":
            return self._teleop_cmd(self._json_body(body))
        if route == "/teleop/stop":
            return self._teleop_stop()
        if route == "/estop":
            return self._set_estop(self._json_body(body))
        if route == "/goal":
            return self._goal(self._json_body(body))
        if route == "/goal/cancel":
            return self._goal_cancel()
        if route == "/initialpose":
            return self._initialpose(self._json_body(body))
        if route == "/nav/activate":
            return self._nav_activate()
        if route == "/map/save":
            return self._save_map(self._json_body(body))
        if route == "/actuator/buzzer":
            return self._actuator("buzzer", self._json_body(body))
        if route == "/actuator/led":
            return self._actuator("led", self._json_body(body))
        if route == "/actuator/servo":
            return self._actuator("servo", self._json_body(body))
        if route in _GET_ROUTES:
            raise DemoHttpError(
                405, "method_not_allowed", f"{route} 只接受 GET"
            )
        raise DemoHttpError(404, "not_found", f"未知接口 /api{route}")

    # ===============================================================
    # SSE
    # ===============================================================
    def event_stream(
        self, path_and_query: str = "/api/events", headers: dict | None = None
    ) -> WireStream:
        try:
            client = self._hub.subscribe()
        except TooManySseClients as exc:
            payload = json.dumps(
                {"error": {"code": "too_many_clients", "message": str(exc)}},
                ensure_ascii=False,
            ).encode("utf-8")
            return WireStream(
                503,
                [("Content-Type", "application/json; charset=utf-8")],
                (),
                buffered=True,
                body=payload,
            )

        backend = self

        def _generate():
            try:
                yield b"retry: 2000\n\n"
                with backend._lock:
                    initial = backend._build_state()
                yield (
                    "event: state\ndata: "
                    + json.dumps(
                        {"state": initial},
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                    + "\n\n"
                ).encode("utf-8")
                last_ping = time.monotonic()
                while not backend._stop.is_set():
                    try:
                        item = client.get(timeout=0.5)
                    except queue.Empty:
                        if time.monotonic() - last_ping >= 10.0:
                            last_ping = time.monotonic()
                            yield b": ping\n\n"
                        continue
                    if item is None:
                        break
                    yield item
            finally:
                backend._hub.unsubscribe(client)

        return WireStream(
            200,
            [
                ("Content-Type", "text/event-stream; charset=utf-8"),
                ("Cache-Control", "no-cache"),
                ("X-Accel-Buffering", "no"),
            ],
            _generate(),
        )

    # ===============================================================
    # 其它
    # ===============================================================
    def uptime(self) -> float:
        return time.time() - self._created_wall


_GET_ROUTES = {
    "/health",
    "/state",
    "/map/meta",
    "/map.png",
    "/maps",
    "/logs",
    "/session/logs",
    "/ros",
}
_POST_ROUTES = {
    "/session/start",
    "/session/stop",
    "/teleop",
    "/teleop/stop",
    "/estop",
    "/goal",
    "/goal/cancel",
    "/initialpose",
    "/nav/activate",
    "/map/save",
    "/actuator/buzzer",
    "/actuator/led",
    "/actuator/servo",
}


__all__ = ["DemoBackend", "DemoHttpError", "EventHub", "WireResponse", "WireStream"]
