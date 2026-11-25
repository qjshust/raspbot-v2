"""演示后端集成测试（--demo 全流程）。

覆盖（设计文档 §8.1 / DoD）：

* ``/api/health`` / ``/api/state`` 必填字段与 schema
* SSE：HTTP/1.1 chunked、逐帧、state + scan + map + path 事件
* SLAM：地图从未知逐步揭示、version 递增、保存地图（PGM+YAML）、地图列表/预览
* 导航：initialpose → 2~3 s 激活 → goal → path → succeeded；不可达 failed/aborted；取消
* 遥控：位姿变化、500 ms 看门狗、急停锁存/解除
* 会话：starting/running/stopping/error 状态机
* 错误路径：409/400/404/405

运行::

    cd pc_console && python -m unittest discover -s tests -v
"""

from __future__ import annotations

import json
import os
import shutil
import struct
import time
import unittest
import urllib.error
import urllib.request
import uuid
import zlib
from pathlib import Path

from console_server.config import Config
from console_server.http_server import create_server

_TMP_ROOT = Path(__file__).resolve().parent.parent / "_tmp"


def make_temp_dir(prefix: str = "demo_") -> Path:
    path = _TMP_ROOT / (prefix + uuid.uuid4().hex[:10])
    path.mkdir(parents=True, exist_ok=True)
    return path


# ---- 文件格式辅助 ---------------------------------------------------
def png_info(data: bytes) -> dict:
    """最小 PNG 解析（仅校验 8-bit 灰度、尺寸、filter 0）。"""
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise AssertionError("不是合法 PNG：magic 错误")
    pos = 8
    info: dict = {}
    idat = bytearray()
    while pos + 8 <= len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        tag = data[pos + 4 : pos + 8]
        chunk = data[pos + 8 : pos + 8 + length]
        pos += 12 + length
        if tag == b"IHDR":
            width, height, depth, color, _comp, _filt, _inter = struct.unpack(
                ">IIBBBBB", chunk
            )
            info.update(width=width, height=height, depth=depth, color=color)
        elif tag == b"IDAT":
            idat += chunk
        elif tag == b"IEND":
            break
    raw = zlib.decompress(bytes(idat))
    width = info["width"]
    height = info["height"]
    info["raw_len"] = len(raw)
    info["filters"] = {
        raw[index * (width + 1)] for index in range(height)
    }
    return info


def read_pgm_bytes(data: bytes) -> tuple[int, int, bytes]:
    if not data.startswith(b"P5"):
        raise AssertionError("不是 P5 PGM")
    pos = 2
    numbers: list[int] = []
    size = len(data)
    while len(numbers) < 3:
        while pos < size and data[pos : pos + 1] in b" \t\r\n":
            pos += 1
        if data[pos : pos + 1] == b"#":
            while pos < size and data[pos : pos + 1] not in b"\r\n":
                pos += 1
            continue
        start = pos
        while pos < size and data[pos : pos + 1].isdigit():
            pos += 1
        if start == pos:
            raise AssertionError("PGM 头部解析失败")
        numbers.append(int(data[start:pos]))
    pos += 1
    width, height, maxval = numbers
    if maxval != 255:
        raise AssertionError(f"PGM maxval 应为 255，实际 {maxval}")
    pixels = data[pos : pos + width * height]
    if len(pixels) != width * height:
        raise AssertionError("PGM 像素数据不足")
    return width, height, pixels


class DemoCase(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = make_temp_dir()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.config = Config(
            host="127.0.0.1",
            port=0,
            pi_host="127.0.0.1",
            pi_port=1,
            token="",
            browser="none",
            config_path=str(self.tmp / "config.json"),
        )
        self.server = create_server(
            self.config,
            demo=True,
            demo_maps_dir=str(self.tmp / "demo_maps"),
        )
        self.addCleanup(self.server.stop)
        self.server.start()
        self.base = self.server.url.rstrip("/")
        self._sse_responses: list = []

    def tearDown(self) -> None:
        for response in self._sse_responses:
            try:
                response.close()
            except Exception:  # pragma: no cover
                pass
        self._sse_responses.clear()

    # ---- HTTP 辅助 ------------------------------------------------
    def api(self, path: str, method: str = "GET", body: dict | None = None):
        data = json.dumps(body).encode("utf-8") if body is not None else None
        headers = {"Content-Type": "application/json"} if data is not None else {}
        request = urllib.request.Request(
            self.base + path, data=data, method=method, headers=headers
        )
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                raw = response.read()
                return response.status, (json.loads(raw) if raw else None)
        except urllib.error.HTTPError as exc:
            try:
                raw = exc.read()
                return exc.code, (json.loads(raw) if raw else None)
            finally:
                exc.close()

    def raw(self, path: str, method: str = "GET"):
        request = urllib.request.Request(self.base + path, method=method)
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, dict(response.headers), response.read()
        except urllib.error.HTTPError as exc:
            try:
                return exc.code, dict(exc.headers), exc.read()
            finally:
                exc.close()

    def state(self) -> dict:
        status, body = self.api("/api/state")
        self.assertEqual(status, 200, body)
        return body

    def wait(self, predicate, timeout: float, desc: str, interval: float = 0.05):
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                value = predicate()
            except AssertionError as exc:  # 轮询过程中的临时断言不视为失败
                last_error = exc
                value = None
            if value:
                return value
            time.sleep(interval)
        raise AssertionError(
            f"等待超时（{timeout:g}s）：{desc}" + (f"；最后错误 {last_error}" if last_error else "")
        )

    def wait_state(self, predicate, timeout: float, desc: str):
        seen: list[str] = []

        def check():
            current = self.state()
            marker = f"{current['session']['state']}/{current['session']['mode']}"
            if not seen or seen[-1] != marker:
                seen.append(marker)
            return current if predicate(current) else None

        result = self.wait(check, timeout, desc)
        return result, seen

    def start_session(self, mode: str, map_path: str | None = None) -> dict:
        body: dict = {"mode": mode}
        if map_path is not None:
            body["map"] = map_path
        status, data = self.api("/api/session/start", method="POST", body=body)
        self.assertEqual(status, 202, data)
        return data

    def demo_home_entry(self) -> dict:
        status, data = self.api("/api/maps")
        self.assertEqual(status, 200)
        for entry in data["maps"]:
            if entry["name"] == "demo_home":
                return entry
        raise AssertionError("演示地图 demo_home 未生成")

    def stop_session(self) -> None:
        status, _data = self.api("/api/session/stop", method="POST")
        self.assertIn(status, (200, 202))
        self.wait(
            lambda: self.state()["session"]["state"] == "idle",
            timeout=8.0,
            desc="会话回到 idle",
        )

    # ---- SSE 辅助 --------------------------------------------------
    def open_sse(self):
        response = urllib.request.urlopen(self.base + "/api/events", timeout=15)
        self._sse_responses.append(response)
        return response

    def collect_sse(self, response, predicate, timeout: float = 6.0) -> list[tuple[str, dict]]:
        events: list[tuple[str, dict]] = []
        current: str | None = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = response.readline()
            if not line:
                break
            text = line.decode("utf-8", "replace").strip()
            if not text:
                current = None
                continue
            if text.startswith("event: "):
                current = text[len("event: ") :]
                continue
            if text.startswith("data: "):
                payload = json.loads(text[len("data: ") :])
                events.append((current or "message", payload))
                if predicate(events):
                    return events
        return events


# ===================================================================
# 静态字段 / 地图生成
# ===================================================================
class HealthStateSchemaTest(DemoCase):
    def test_health(self) -> None:
        status, data = self.api("/api/health")
        self.assertEqual(status, 200)
        self.assertTrue(data["ok"])
        self.assertEqual(data["name"], "raspbot-console")
        self.assertEqual(data["version"], "1.0.0")
        self.assertEqual(data["api"], 1)
        self.assertIsInstance(data["server_time"], float)
        self.assertGreaterEqual(data["uptime_s"], 0.0)
        self.assertTrue(data["demo"])

    def test_idle_state_schema(self) -> None:
        state = self.state()
        self.assertEqual(state["api"], 1)
        self.assertIsInstance(state["server_time"], float)
        self.assertGreaterEqual(state["uptime_s"], 0.0)

        session = state["session"]
        for key in (
            "mode",
            "state",
            "mock",
            "managed",
            "external",
            "available_modes",
            "since",
            "progress",
            "message",
            "error",
            "map",
        ):
            self.assertIn(key, session)
        self.assertEqual(session["mode"], "idle")
        self.assertEqual(session["state"], "idle")
        self.assertEqual(session["available_modes"], ["base", "slam", "nav"])
        self.assertIsNone(session["error"])
        self.assertIsNone(session["map"])

        self.assertEqual(state["limits"], {"max_linear": 0.30, "max_lateral": 0.30, "max_angular": 1.5})
        self.assertIsNone(state["robot"])

        nav = state["nav"]
        self.assertEqual(nav["state"], "unavailable")
        self.assertIsNone(nav["initialpose"])
        self.assertIsNone(nav["goal"])
        self.assertEqual(nav["goal_state"], "idle")
        self.assertEqual(nav["activation"]["total"], 6)

        self.assertFalse(state["map"]["available"])
        self.assertEqual(state["map"]["version"], 0)
        # 契约允许 available=false 时尺寸为 null；演示后端给数值占位（便于严格 schema 校验）
        self.assertEqual(state["map"]["width"], 0)
        self.assertEqual(state["map"]["height"], 0)
        self.assertEqual(state["map"]["frame_id"], "map")

        self.assertFalse(state["estop"]["active"])
        self.assertIsNone(state["estop"]["since"])
        self.assertFalse(state["teleop"]["active"])
        self.assertIsNone(state["teleop"]["last_cmd"])

        peripherals = state["peripherals"]
        self.assertFalse(peripherals["buzzer"])
        self.assertEqual(peripherals["led"], {"r": 0.0, "g": 0.0, "b": 0.0})
        self.assertEqual(peripherals["servo"], {"pan": 90.0, "tilt": 45.0})

        diagnostics = state["diagnostics"]
        self.assertIn(diagnostics["level"], ("ok", "warn", "error", "unknown"))
        self.assertGreaterEqual(len(diagnostics["items"]), 4)
        self.assertLessEqual(len(diagnostics["items"]), 8)
        for item in diagnostics["items"]:
            self.assertIn("name", item)
            self.assertIn(item["level"], ("ok", "warn", "error", "unknown"))
            self.assertIn("message", item)
        self.assertEqual(
            set(diagnostics["counts"]), {"ok", "warn", "error"}
        )

        self.assertIsInstance(state["logs"], list)

        ros = state["ros"]
        self.assertFalse(ros["ok"])
        self.assertEqual(ros["nodes"], 0)
        self.assertEqual(
            set(ros["ready"]),
            {"base", "teleop", "map", "localization", "nav2", "diagnostics"},
        )
        self.assertEqual(
            set(ros["rates"]), {"scan_hz", "odom_hz", "map_age_s", "diagnostics_age_s"}
        )

    def test_idle_debug_endpoints(self) -> None:
        status, data = self.api("/api/ros")
        self.assertEqual(status, 200)
        self.assertEqual(data["nodes"], [])
        self.assertEqual(data["topics"], [])
        status, data = self.api("/api/logs?limit=5")
        self.assertEqual(status, 200)
        self.assertGreaterEqual(len(data["logs"]), 1)
        self.assertLessEqual(len(data["logs"]), 5)
        for entry in data["logs"]:
            self.assertIn("ts", entry)
            self.assertIn(entry["level"], ("info", "warn", "error"))
            self.assertIn("msg", entry)
        status, data = self.api("/api/session/logs?limit=5")
        self.assertEqual(status, 200)
        self.assertIn("logs", data)
        status, data = self.api("/api/map/meta")
        self.assertEqual(status, 200)
        self.assertFalse(data["available"])
        status, data = self.api("/api/map.png")
        self.assertEqual(status, 404)
        self.assertEqual(data["error"]["code"], "no_map")


class DemoMapGenerationTest(DemoCase):
    def test_demo_home_listed_and_valid(self) -> None:
        entry = self.demo_home_entry()
        self.assertTrue(entry["valid"])
        self.assertEqual(entry["width"], 240)
        self.assertEqual(entry["height"], 160)
        self.assertAlmostEqual(entry["resolution"], 0.05)
        self.assertTrue(Path(entry["pgm"]).is_file())
        self.assertTrue(Path(entry["yaml"]).is_file())
        self.assertEqual(entry["preview_url"], "/api/maps/demo_home/preview.png")

    def test_pgm_y_order_and_known_cells(self) -> None:
        entry = self.demo_home_entry()
        width, height, pixels = read_pgm_bytes(Path(entry["pgm"]).read_bytes())
        self.assertEqual((width, height), (240, 160))
        # 行 0 = 世界 y 最大（上外墙 7.85~8.0 m）→ 整行占用
        self.assertTrue(all(value == 0 for value in pixels[0:width]))
        # 最后一行 = y 最小（下外墙 0~0.15 m）→ 整行占用
        self.assertTrue(all(value == 0 for value in pixels[-width:]))
        # 机器人起点 (1.5, 1.5) → 空闲 254；行号 = 159 - iy
        iy = int(1.5 / 0.05)
        col = int(1.5 / 0.05)
        self.assertEqual(pixels[(height - 1 - iy) * width + col], 254)
        # 桌子 (8.8, 1.1) 在真值中占用
        col_table = int(8.8 / 0.05)
        iy_table = int(1.1 / 0.05)
        self.assertEqual(pixels[(height - 1 - iy_table) * width + col_table], 0)

    def test_preview_png(self) -> None:
        entry = self.demo_home_entry()
        status, headers, body = self.raw(entry["preview_url"])
        self.assertEqual(status, 200)
        self.assertEqual(headers.get("Content-Type"), "image/png")
        info = png_info(body)
        self.assertEqual(info["width"], 240)
        self.assertEqual(info["height"], 160)
        self.assertEqual(info["depth"], 8)
        self.assertEqual(info["color"], 0)
        self.assertEqual(info["filters"], {0})
        self.assertEqual(info["raw_len"], 160 * (240 + 1))


# ===================================================================
# base 会话 / 外设 / 诊断
# ===================================================================
class BaseSessionTest(DemoCase):
    def test_base_session_lifecycle(self) -> None:
        data = self.start_session("base")
        self.assertEqual(data["session"]["state"], "starting")
        self.assertEqual(data["session"]["mode"], "base")
        running, seen = self.wait_state(
            lambda current: current["session"]["state"] == "running",
            timeout=8.0,
            desc="base 会话 running",
        )
        self.assertIn("starting/base", seen)
        self.assertEqual(running["session"]["mode"], "base")
        robot = running["robot"]
        self.assertIsNotNone(robot)
        self.assertEqual(robot["frame"], "odom")  # base 无地图 → 未定位
        self.assertGreater(robot["x"], 0.0)
        self.assertGreater(robot["y"], 0.0)
        self.assertIn("speed", robot)
        self.assertIn("imu", robot)
        self.assertIn("scan", robot)

        ros = running["ros"]
        self.assertTrue(ros["ok"])
        self.assertTrue(ros["ready"]["base"])
        self.assertTrue(ros["ready"]["teleop"])
        self.assertFalse(ros["ready"]["map"])
        self.assertFalse(ros["ready"]["nav2"])

        diagnostics = running["diagnostics"]
        self.assertEqual(diagnostics["level"], "ok")
        self.assertGreaterEqual(diagnostics["counts"]["ok"], 4)
        self.assertEqual(diagnostics["counts"]["warn"], 0)
        self.assertEqual(diagnostics["counts"]["error"], 0)
        self.assertGreaterEqual(len(diagnostics["items"]), 4)
        self.assertLessEqual(len(diagnostics["items"]), 8)

        # 调试信息 & 会话日志
        status, debug = self.api("/api/ros")
        self.assertEqual(status, 200)
        self.assertTrue(any("raspbot_base" in node for node in debug["nodes"]))
        status, logs = self.api("/api/session/logs?limit=50")
        self.assertEqual(status, 200)
        self.assertGreaterEqual(len(logs["logs"]), 4)
        self.assertIn("line", logs["logs"][0])

        # 外设（clamp / 回显）
        status, data = self.api("/api/actuator/buzzer", "POST", {"on": True})
        self.assertEqual(status, 200)
        self.assertTrue(data["peripherals"]["buzzer"])
        status, data = self.api("/api/actuator/led", "POST", {"r": 2.0, "g": 0.5, "b": -1})
        self.assertEqual(status, 200)
        self.assertEqual(data["peripherals"]["led"], {"r": 1.0, "g": 0.5, "b": 0.0})
        status, data = self.api("/api/actuator/servo", "POST", {"pan": -5, "tilt": 200})
        self.assertEqual(status, 200)
        self.assertEqual(data["peripherals"]["servo"], {"pan": 0.0, "tilt": 110.0})
        current = self.state()
        self.assertTrue(current["peripherals"]["buzzer"])

        # 停止 → stopping → idle
        status, _ = self.api("/api/session/stop", method="POST")
        self.assertIn(status, (200, 202))
        idle, seen_stop = self.wait_state(
            lambda state: state["session"]["state"] == "idle",
            timeout=8.0,
            desc="base 会话停止",
        )
        self.assertTrue(any(marker.startswith("stopping/") for marker in seen_stop), seen_stop)
        self.assertEqual(idle["session"]["mode"], "idle")
        self.assertIsNone(idle["robot"])
        self.assertFalse(idle["map"]["available"])

        # 空闲后遥控 409（错误码与 Pi 网关一致：session_not_running）
        status, data = self.api("/api/teleop", "POST", {"vx": 0.1})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "session_not_running")


# ===================================================================
# SLAM：建图 / SSE / 保存地图
# ===================================================================
class SlamSseSaveMapTest(DemoCase):
    def test_slam_map_reveal_sse_and_save(self) -> None:
        self.start_session("slam")
        self.wait(
            lambda: self.state()["session"]["state"] == "running",
            timeout=8.0,
            desc="slam 会话 running",
        )
        self.wait(
            lambda: self.state()["map"]["available"],
            timeout=6.0,
            desc="首帧地图可用",
        )
        map_meta = self.state()["map"]
        self.assertEqual((map_meta["width"], map_meta["height"]), (240, 160))
        self.assertAlmostEqual(map_meta["resolution"], 0.05)
        self.assertEqual(map_meta["origin"]["x"], 0.0)
        self.assertEqual(map_meta["frame_id"], "map")
        version_before = map_meta["version"]

        # SSE：chunked + 逐帧；打开后移动机器人触发 map 事件
        response = self.open_sse()
        headers = {key.lower(): value for key, value in response.headers.items()}
        self.assertIn("text/event-stream", headers.get("content-type", ""))
        self.assertEqual(headers.get("transfer-encoding"), "chunked")
        self.assertEqual(headers.get("x-accel-buffering"), "no")
        self.assertNotIn("content-length", headers)
        self.assertEqual(headers.get("cache-control"), "no-cache")

        for seq in range(10):
            status, _ = self.api(
                "/api/teleop", "POST", {"vx": 0.25, "vy": 0.0, "wz": 0.0, "seq": seq}
            )
            self.assertEqual(status, 200)
            time.sleep(0.07)
        self.api("/api/teleop/stop", method="POST")

        events = self.collect_sse(
            response,
            lambda collected: {"state", "scan", "map"}
            <= {name for name, _payload in collected},
            timeout=6.0,
        )
        kinds = {name for name, _payload in events}
        self.assertIn("state", kinds)
        self.assertIn("scan", kinds)
        self.assertIn("map", kinds)
        for name, payload in events:
            if name == "state":
                self.assertEqual(payload["state"]["api"], 1)
            elif name == "scan":
                self.assertEqual(payload["frame"], "map")
                self.assertEqual(len(payload["points"]) % 2, 0)
                self.assertLessEqual(len(payload["points"]), 360 * 2)
                self.assertGreater(len(payload["points"]), 0)
            elif name == "map":
                self.assertTrue(payload["meta"]["available"])
        response.close()

        self.wait(
            lambda: self.state()["map"]["version"] > version_before,
            timeout=5.0,
            desc="建图 version 递增",
        )
        revealed = self.state()["map"]
        self.assertGreater(revealed["version"], version_before)

        status, headers, body = self.raw("/api/map.png")
        self.assertEqual(status, 200)
        self.assertEqual(headers.get("Content-Type"), "image/png")
        info = png_info(body)
        self.assertEqual((info["width"], info["height"]), (240, 160))
        self.assertEqual(info["depth"], 8)
        self.assertEqual(info["color"], 0)

        # 保存地图
        status, saved = self.api("/api/map/save", "POST", {"name": "slam_e2e"})
        self.assertEqual(status, 200, saved)
        self.assertTrue(saved["ok"])
        self.assertEqual(saved["name"], "slam_e2e")
        yaml_path = Path(saved["yaml"])
        pgm_path = Path(saved["pgm"])
        self.assertTrue(yaml_path.is_file())
        self.assertTrue(pgm_path.is_file())
        width, height, pixels = read_pgm_bytes(pgm_path.read_bytes())
        self.assertEqual((width, height), (240, 160))
        text = yaml_path.read_text(encoding="utf-8")
        self.assertIn("image: slam_e2e.pgm", text)
        self.assertIn("mode: trinary", text)
        self.assertIn("resolution: 0.05", text)
        self.assertIn("origin: [0.0, 0.0, 0.0]", text)
        self.assertIn("negate: 0", text)
        self.assertIn("occupied_thresh: 0.65", text)
        self.assertIn("free_thresh: 0.196", text)
        # SLAM 地图：外圈仍是未知 205，起点附近已揭示
        self.assertEqual(pixels[0], 205)  # 顶部外墙尚未扫描到
        self.assertTrue(any(value == 0 for value in pixels))
        self.assertTrue(any(value == 254 for value in pixels))

        # 列表 / 预览
        status, maps = self.api("/api/maps")
        names = {entry["name"]: entry for entry in maps["maps"]}
        self.assertIn("demo_home", names)
        self.assertIn("slam_e2e", names)
        self.assertTrue(names["slam_e2e"]["valid"])
        self.assertEqual(names["slam_e2e"]["width"], 240)
        status, headers, body = self.raw("/api/maps/slam_e2e/preview.png")
        self.assertEqual(status, 200)
        self.assertEqual(png_info(body)["width"], 240)

        # 冲突 / 覆盖 / 非法名称
        status, data = self.api("/api/map/save", "POST", {"name": "slam_e2e"})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "map_exists")
        status, data = self.api(
            "/api/map/save", "POST", {"name": "slam_e2e", "overwrite": True}
        )
        self.assertEqual(status, 200)
        status, data = self.api("/api/map/save", "POST", {"name": "../evil"})
        self.assertEqual(status, 400)
        self.assertEqual(data["error"]["code"], "bad_request")
        self.assertEqual(data["error"]["detail"]["field"], "name")


# ===================================================================
# 导航闭环
# ===================================================================
class NavFlowTest(DemoCase):
    def test_nav_initialpose_goal_success(self) -> None:
        entry = self.demo_home_entry()
        self.start_session("nav", entry["path"])
        self.wait(
            lambda: self.state()["session"]["state"] == "running",
            timeout=8.0,
            desc="nav 会话 running",
        )
        current = self.state()
        self.assertEqual(current["nav"]["state"], "waiting_initialpose")
        self.assertTrue(current["map"]["available"])
        self.assertTrue(current["ros"]["ready"]["map"])

        # 未设初始位姿 → goal 409
        status, data = self.api("/api/goal", "POST", {"x": 2.0, "y": 1.5})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "nav_not_ready")

        # 初始位姿 → activating → ready
        status, data = self.api(
            "/api/initialpose", "POST", {"x": 1.5, "y": 1.5, "yaw": 0.0}
        )
        self.assertEqual(status, 200)
        self.assertEqual(data["nav"]["state"], "activating")
        activating, _seen = self.wait_state(
            lambda state: state["nav"]["state"] == "activating",
            timeout=2.0,
            desc="nav activating 状态",
        )
        self.assertGreaterEqual(activating["nav"]["activation"]["total"], 1)
        ready, _seen2 = self.wait_state(
            lambda state: state["nav"]["state"] == "ready",
            timeout=8.0,
            desc="nav ready",
        )
        self.assertEqual(ready["nav"]["activation"]["done"], 6)
        self.assertTrue(ready["ros"]["ready"]["localization"])
        self.assertTrue(ready["ros"]["ready"]["nav2"])

        # 手动重试激活
        status, _data = self.api("/api/nav/activate", method="POST")
        self.assertEqual(status, 202)
        self.wait_state(
            lambda state: state["nav"]["state"] == "ready",
            timeout=8.0,
            desc="nav 重新激活 ready",
        )

        # 目标 → path 事件 → succeeded
        response = self.open_sse()
        status, data = self.api("/api/goal", "POST", {"x": 2.5, "y": 1.5, "yaw": None})
        self.assertEqual(status, 202)
        self.assertIn("goal_id", data)
        self.assertEqual(data["goal"]["yaw"] is not None, True)
        events = self.collect_sse(
            response, lambda collected: any(name == "path" for name, _ in collected), timeout=6.0
        )
        path_events = [payload for name, payload in events if name == "path"]
        self.assertTrue(path_events, "未收到 path 事件")
        self.assertEqual(path_events[0]["frame"], "map")
        self.assertGreaterEqual(len(path_events[0]["points"]), 4)
        self.assertEqual(len(path_events[0]["points"]) % 2, 0)
        response.close()

        succeeded, _seen3 = self.wait_state(
            lambda state: state["nav"]["state"] == "succeeded",
            timeout=20.0,
            desc="导航 succeeded",
        )
        self.assertEqual(succeeded["nav"]["goal_state"], "succeeded")
        self.assertEqual(succeeded["nav"]["distance_remaining"], 0.0)
        self.assertIsNotNone(succeeded["nav"]["navigation_time_s"])
        robot = succeeded["robot"]
        self.assertLess(
            ((robot["x"] - 2.5) ** 2 + (robot["y"] - 1.5) ** 2) ** 0.5, 0.3
        )

        # 不可达目标 → failed/aborted
        status, _data = self.api("/api/goal", "POST", {"x": 8.8, "y": 1.1})
        self.assertEqual(status, 202)
        failed, _seen4 = self.wait_state(
            lambda state: state["nav"]["state"] == "failed",
            timeout=6.0,
            desc="不可达目标 failed",
        )
        self.assertEqual(failed["nav"]["goal_state"], "aborted")
        self.assertIn("不可达", failed["nav"]["message"])

        # 取消
        status, _data = self.api("/api/goal", "POST", {"x": 5.0, "y": 1.5})
        self.assertEqual(status, 202)
        self.wait_state(
            lambda state: state["nav"]["state"] == "goal_active",
            timeout=3.0,
            desc="目标 active",
        )
        status, _data = self.api("/api/goal/cancel", method="POST")
        self.assertEqual(status, 200)
        canceled, _seen5 = self.wait_state(
            lambda state: state["nav"]["state"] == "canceled",
            timeout=3.0,
            desc="取消目标",
        )
        self.assertEqual(canceled["nav"]["goal_state"], "canceled")

        # 急停取消导航目标并锁存
        status, _data = self.api("/api/goal", "POST", {"x": 5.0, "y": 1.5})
        self.assertEqual(status, 202)
        self.wait_state(
            lambda state: state["nav"]["state"] == "goal_active",
            timeout=3.0,
            desc="目标再次 active",
        )
        status, data = self.api("/api/estop", "POST", {"active": True})
        self.assertEqual(status, 200)
        self.assertTrue(data["estop"]["active"])
        self.assertIsNotNone(data["estop"]["since"])
        stopped, _seen6 = self.wait_state(
            lambda state: state["nav"]["state"] == "canceled",
            timeout=3.0,
            desc="急停取消导航",
        )
        self.assertEqual(stopped["nav"]["goal_state"], "canceled")
        status, data = self.api("/api/teleop", "POST", {"vx": 0.1})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "estop_active")
        # 与 Pi 网关一致：急停不禁止 initialpose（契约仅禁止 session start/goal/teleop）
        status, data = self.api("/api/initialpose", "POST", {"x": 1.5, "y": 1.5, "yaw": 0.0})
        self.assertEqual(status, 200)
        self.assertEqual(data["nav"]["state"], "activating")
        status, data = self.api("/api/estop", "POST", {"active": False})
        self.assertEqual(status, 200)
        self.assertFalse(data["estop"]["active"])

        status, _ = self.api("/api/session/stop", method="POST")
        self.assertIn(status, (200, 202))
        self.wait(
            lambda: self.state()["session"]["state"] == "idle",
            timeout=8.0,
            desc="nav 会话停止",
        )


# ===================================================================
# 遥控 / 看门狗 / 急停
# ===================================================================
class TeleopEstopTest(DemoCase):
    def test_teleop_watchdog_and_estop(self) -> None:
        self.start_session("base")
        self.wait(
            lambda: self.state()["session"]["state"] == "running",
            timeout=8.0,
            desc="base 会话 running",
        )
        start = self.state()["robot"]
        for seq in range(8):
            status, _ = self.api(
                "/api/teleop",
                "POST",
                {"vx": 0.2, "vy": 0.0, "wz": 0.0, "seq": seq},
            )
            self.assertEqual(status, 200)
            time.sleep(0.05)
        time.sleep(0.15)
        moved_state = self.state()
        moved = moved_state["robot"]
        self.assertGreater(
            abs(moved["x"] - start["x"]) + abs(moved["y"] - start["y"]),
            0.04,
            "遥控未改变位姿",
        )
        self.assertAlmostEqual(moved["speed"]["vx"], 0.2, places=2)
        self.assertTrue(moved_state["teleop"]["active"])

        # 显式停
        status, _ = self.api("/api/teleop/stop", method="POST")
        self.assertEqual(status, 200)
        time.sleep(0.1)
        stopped = self.state()
        self.assertFalse(stopped["teleop"]["active"])
        self.assertEqual(stopped["robot"]["speed"], {"vx": 0.0, "vy": 0.0, "wz": 0.0})

        # 500 ms 看门狗
        self.api("/api/teleop", "POST", {"vx": 0.2, "seq": 100})
        time.sleep(0.75)
        after_watchdog = self.state()
        self.assertFalse(after_watchdog["teleop"]["active"])
        self.assertEqual(after_watchdog["robot"]["speed"]["vx"], 0.0)

        # seq 过期帧被丢弃
        self.api("/api/teleop", "POST", {"vx": 0.2, "seq": 200})
        self.api("/api/teleop", "POST", {"vx": -0.2, "seq": 150})
        time.sleep(0.1)
        self.assertAlmostEqual(self.state()["teleop"]["vx"], 0.2, places=2)
        self.api("/api/teleop/stop", method="POST")

        # 非法数值 → 400
        status, data = self.api("/api/teleop", "POST", {"vx": "fast"})
        self.assertEqual(status, 400)
        status, data = self.api("/api/teleop", "POST", {"vx": True})
        self.assertEqual(status, 400)

        # 急停：锁存 / 拒绝命令 / 允许停止
        status, data = self.api("/api/estop", "POST", {"active": True})
        self.assertEqual(status, 200)
        self.assertTrue(data["estop"]["active"])
        status, data = self.api("/api/teleop", "POST", {"vx": 0.1})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "estop_active")
        status, data = self.api("/api/session/start", "POST", {"mode": "slam"})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "estop_active")
        status, data = self.api("/api/goal", "POST", {"x": 2.0, "y": 1.5})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "estop_active")
        status, data = self.api("/api/initialpose", "POST", {"x": 1.5, "y": 1.5, "yaw": 0})
        self.assertEqual(status, 409)
        # 急停时允许停止会话
        status, _ = self.api("/api/session/stop", method="POST")
        self.assertIn(status, (200, 202))
        self.wait(
            lambda: self.state()["session"]["state"] == "idle",
            timeout=8.0,
            desc="急停中停止会话",
        )
        # 解除后遥控可用
        status, data = self.api("/api/estop", "POST", {"active": False})
        self.assertEqual(status, 200)
        self.assertFalse(data["estop"]["active"])
        self.assertIsNone(data["estop"]["since"])
        status, data = self.api("/api/session/start", "POST", {"mode": "base"})
        self.assertEqual(status, 202)
        self.wait(
            lambda: self.state()["session"]["state"] == "running",
            timeout=8.0,
            desc="解除急停后启动",
        )
        status, _ = self.api("/api/teleop", "POST", {"vx": 0.1})
        self.assertEqual(status, 200)


# ===================================================================
# 错误路径 / 会话 error
# ===================================================================
class ErrorPathsTest(DemoCase):
    def test_errors_when_idle(self) -> None:
        status, data = self.api("/api/teleop", "POST", {"vx": 0.1})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "session_not_running")
        status, data = self.api("/api/map/save", "POST", {"name": "idle_map"})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "session_idle")
        status, data = self.api("/api/goal", "POST", {"x": 1.0, "y": 1.0})
        self.assertEqual(status, 409)
        status, data = self.api("/api/initialpose", "POST", {"x": 1.0, "y": 1.0, "yaw": 0})
        self.assertEqual(status, 409)
        status, data = self.api("/api/nav/activate", method="POST")
        self.assertEqual(status, 409)
        status, data = self.api("/api/actuator/buzzer", "POST", {"on": True})
        self.assertEqual(status, 409)
        self.assertEqual(data["error"]["code"], "session_not_running")
        status, data = self.api("/api/goal/cancel", method="POST")
        self.assertEqual(status, 200)
        status, data = self.api("/api/session/stop", method="POST")
        self.assertEqual(status, 200)
        self.assertTrue(data["ok"])

    def test_invalid_session_start(self) -> None:
        status, data = self.api("/api/session/start", "POST", {"mode": "fly"})
        self.assertEqual(status, 400)
        self.assertEqual(data["error"]["code"], "bad_request")
        status, data = self.api("/api/session/start", "POST", {"mode": "nav"})
        self.assertEqual(status, 400)
        self.assertEqual(data["error"]["detail"]["field"], "map")
        status, data = self.api(
            "/api/session/start", "POST", {"mode": "nav", "map": str(self.tmp / "nope.yaml")}
        )
        self.assertEqual(status, 400)
        status, data = self.api(
            "/api/session/start", "POST", {"mode": "nav", "map": "relative.yaml"}
        )
        self.assertEqual(status, 400)

    def test_corrupt_map_enters_session_error(self) -> None:
        bad_yaml = self.tmp / "demo_maps" / "broken.yaml"
        bad_yaml.parent.mkdir(parents=True, exist_ok=True)
        bad_yaml.write_text(
            "image: missing.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
            encoding="utf-8",
        )
        status, data = self.api(
            "/api/session/start", "POST", {"mode": "nav", "map": str(bad_yaml)}
        )
        self.assertEqual(status, 202)
        error_state, seen = self.wait_state(
            lambda state: state["session"]["state"] == "error",
            timeout=8.0,
            desc="坏地图 → session error",
        )
        self.assertIn("starting/nav", seen)
        self.assertIsNotNone(error_state["session"]["error"])
        self.assertIn("地图", error_state["session"]["error"])
        status, _ = self.api("/api/session/stop", method="POST")
        self.assertIn(status, (200, 202))
        self.wait(
            lambda: self.state()["session"]["state"] == "idle",
            timeout=8.0,
            desc="error 后停止会话",
        )


if __name__ == "__main__":
    unittest.main()
