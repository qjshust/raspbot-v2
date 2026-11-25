# RASPBOT-V2 PC 上位机控制台（WP-C2）

一台 PC 上运行的**观察 + 控制**控制台：本地 HTTP/SSE 服务 + 静态 Web UI + 到 Pi 网关的反向代理，
并内置 **`--demo` 演示模式**（纯 Python 仿真后端，无需 Pi / ROS / 硬件，协议与 Pi 网关完全一致）。

- 唯一依赖：**Python 3.10+ 标准库**（开发/验收环境 Windows 11 + Python 3.14）
- 前端 UI 由 WP-C3 开发，位于 `pc_console/web/`；本程序只负责静态服务与 API
- 接口契约：`docs/PC上位机_设计文档_v1.md` §3 / §4（冻结版本 v1.0）

---

## 1. 快速开始

### 1.1 演示模式（推荐；无硬件即可完整体验）

```bat
cd pc_console
start-demo.bat
:: 等价于：
python main.py --demo
```

浏览器自动打开 `http://127.0.0.1:8090/`。可完整体验：

建图（实时揭示地图）→ 保存地图 → 导航（设初始位姿 → 点目标 → 到达）→ 遥控 → 急停 → 诊断/日志。

### 1.2 连接真车

```bat
cd pc_console
start.bat
:: 等价于：
python main.py --pi 192.168.50.98:8080 --token 你的token
```

默认目标为 `192.168.50.98:8080`（设计文档约定），可在 `config.json` 或 UI 设置里修改。
Pi 侧网关启动方式见 `ros2_ws/src/raspbot_console/README.md`（WP-C1）。

### 1.3 常用命令

```bat
python main.py                                   :: 真车模式（默认配置）
python main.py --demo                            :: 演示模式
python main.py --demo --port 8099 --no-browser   :: 指定端口、不自动开浏览器
python main.py --pi 192.168.50.98:8080 --token abc
python main.py --fallback-demo                   :: 探测不到 Pi 则自动切演示模式
python main.py --port 0                          :: 由系统分配临时端口（打印实际 URL，测试用）
python main.py --verbose                         :: 输出每个 HTTP 请求等调试日志
python main.py --help
```

> 端口约定：本地默认 `8090`，Pi 网关默认 `8080`。除显式指定外，程序不会占用这两个端口
> （演示冒烟可用 `8099` 等空闲端口）。

---

## 2. 命令行参数

| 参数 | 说明 |
|---|---|
| `--host` | 本地监听地址，默认 `127.0.0.1`（仅本机；需要局域网访问可设 `0.0.0.0`） |
| `--port` | 本地监听端口，默认 `8090`；`0` = 系统分配临时端口并打印实际 URL |
| `--pi HOST[:PORT]` | Pi 网关地址，默认 `192.168.50.98:8080`；支持 `[::1]:8080` 形式 |
| `--token` | 访问 Pi 网关的 `X-Console-Token`（可留空） |
| `--demo` | 演示模式：使用内置仿真后端，无需 Pi/ROS |
| `--fallback-demo` | 启动时探测上游 `/api/health`，失败则自动切换演示模式 |
| `--browser auto\|app\|default\|none` | 浏览器启动方式，默认 `auto`（Windows 优先 Edge/Chrome `--app` 窗口，找不到回退默认浏览器） |
| `--no-browser` | 不自动打开浏览器（等价 `--browser none`） |
| `--config PATH` | 配置文件路径，默认 `pc_console/config.json` |
| `--verbose` | 调试日志（含逐请求日志） |

浏览器启动失败**不影响服务运行**，控制台始终打印可访问 URL。

---

## 3. 配置文件

- 位置：`pc_console/config.json`（首次运行自动生成，**不提交**，已加入 `.gitignore`）
- 优先级：**CLI 参数 > `config.json` > 内置默认值**
- 写入方式：同目录临时文件 + `os.replace` 原子写；POSIX 权限收至 `0o600`

| 字段 | 默认值 | 说明 |
|---|---|---|
| `host` | `"127.0.0.1"` | 本地监听地址 |
| `port` | `8090` | 本地监听端口 |
| `pi_host` | `"192.168.50.98"` | Pi 网关主机名/IP |
| `pi_port` | `8080` | Pi 网关端口 |
| `token` | `""` | `X-Console-Token`，空 = 不鉴权 |
| `browser` | `"auto"` | `auto / app / default / none` |

`config.example.json` 为示例（字段同上）。

本地接口：

- `GET /local/config` → 当前配置与配置路径
- `POST /local/config` `{"pi_host":"...","pi_port":8080,"token":""}` → 校验、原子持久化、**即时切换代理目标**；演示模式返回 `409 demo_mode`
- `GET /local/ping` → 探测目标 `/api/health`，返回 `reachable / latency_ms / target / error`

---

## 4. 目录结构

```
pc_console/
  main.py                     # 入口：参数解析、浏览器启动、Ctrl-C 生命周期
  console_server/
    __init__.py               # 包导出（create_server / load_config ...）
    config.py                 # 配置加载/校验/原子保存（CLI > config.json > 默认）
    http_server.py            # ThreadingHTTPServer：静态资源、/local/*、/api/* 分发、SSE chunked
    upstream.py               # 真车模式：http.client 反向代理（token/超时/502/SSE 逐行）
    demo.py                   # 演示后端：与 Pi 网关一致的 Wire API v1 + 仿真状态机
    demo_scene.py             # 12×8 m 场景、栅格、DDA 射线、A*、PNG/PGM/YAML（标准库）
  tests/                      # 标准库 unittest（62 个用例）
  web/                        # WP-C3 Web UI（本工作包不修改）
  demo_maps/                  # 演示模式运行时生成（demo_home.pgm/.yaml，git 忽略）
  config.example.json
  config.json                 # 首次运行生成（git 忽略）
  start.bat / start-demo.bat
  README.md
```

可编程接口（供测试 / `tools/console/contract_check.py`）：

```python
from console_server import create_server, load_config

cfg = load_config()                    # 读取 pc_console/config.json
server = create_server(cfg, demo=True) # demo=False 为真车代理模式
server.start()
print(server.url)                      # http://127.0.0.1:8090/
...                                    # 可直接发 HTTP/SSE 请求
server.stop()
```

`python main.py` 只做参数解析与生命周期，服务实现全部在 `console_server` 包内。

---

## 5. 已实现 API 摘要

### 5.1 PC 本地接口

| 方法/路径 | 说明 |
|---|---|
| `GET /` | → `web/index.html`（静态资源白名单：`.html/.css/.js/.mjs/.svg/.png/.ico/.json/.woff2`，防 `..` 穿越，`Cache-Control: no-store`） |
| `GET /local/config` | 当前配置（demo/target/token_set/config_path/实际端口） |
| `POST /local/config` | 修改 Pi 目标并持久化、即时生效；演示模式 409 |
| `GET /local/ping` | 探测上游 `/api/health` |
| `GET /api/*` | 真车模式反向代理到 Pi 网关（保留方法/状态码/Content-Type/二进制）；上游不可达 → `502 {"error":{"code":"upstream_unreachable"}}`，上游超时 → `504 upstream_timeout` |
| `GET /api/events` | SSE。**HTTP/1.1 + `Transfer-Encoding: chunked`**，逐帧 flush、不缓冲、无 Content-Length，附 `X-Accel-Buffering: no`；上游逐跳头不转发，由本地重新编码 |

### 5.2 Wire API v1（演示后端与 Pi 网关一致）

| 方法/路径 | 说明 |
|---|---|
| `GET /api/health` | 存活探针（免 token）；演示模式附 `"demo": true` |
| `GET /api/state` | 完整状态快照（§3.1 schema：api/server_time/uptime_s/session/ros/limits/robot/nav/map/estop/teleop/peripherals/diagnostics/logs） |
| `GET /api/events` | SSE：`state` 10 Hz、`scan` 5 Hz、`path`/`map`/`session`/`log` 事件 |
| `GET /api/map/meta` | 地图元数据（无图 `available=false`） |
| `GET /api/map.png?v=N` | 灰度 8-bit PNG（标准库 zlib 生成，可解码） |
| `GET /api/maps` | 已保存地图列表（name/path/pgm/mtime/resolution/width/height/preview_url/valid） |
| `GET /api/maps/<name>/preview.png` | 地图缩略图（PGM→PNG） |
| `POST /api/map/save` | 保存当前地图为 PGM(P5)+YAML；`409 map_exists / no_map / session_idle` |
| `POST /api/session/start` | `{"mode":"base\|slam\|nav","map":null,"use_mock":false,...}` → `202`；演示模式 1~3 s 异步启动 |
| `POST /api/session/stop` | 空闲 `200`；运行中 `202` 异步停止 |
| `POST /api/teleop` | `{"vx","vy","wz","seq"?}`，clamp 到 limits；500 ms 看门狗自动停 |
| `POST /api/teleop/stop` | 立即零速并退出遥控 |
| `POST /api/estop` | `{"active":true/false}` 锁存/解除；急停期间 teleop/goal/session start 409 |
| `POST /api/goal` | `{"x","y","yaw"}` → `202 {goal,goal_id}`；不可达异步 `failed/aborted` |
| `POST /api/goal/cancel` | 取消当前导航目标 |
| `POST /api/initialpose` | `{"x","y","yaw"}` → `nav.state=activating` → 2~3 s 后 `ready` |
| `POST /api/nav/activate` | 手动重试 Nav2 激活 |
| `POST /api/actuator/buzzer\|led\|servo` | 蜂鸣器 / RGB / 云台（clamp + 回显，会话未运行 409） |
| `GET /api/logs?limit=200` | 网关事件日志（≤300） |
| `GET /api/session/logs?limit=200` | 模拟 bringup stdout 尾部（≤500） |
| `GET /api/ros` | 节点/话题调试信息 |

统一错误格式：`{"error":{"code":"...","message":"...","detail":{...}}}`；
状态码：`400/404/405/409/413/502/503/504/500`。

---

## 6. 演示模式（`--demo`）能力

- **场景**：12 m × 8 m 室内，外墙 + 两个房间 + 走廊 + 家具（桌子/沙发/置物架/书桌/箱子/立柱），
  0.05 m 栅格（240×160）；机器人起点 `(1.5, 1.5, 0°)`。
- **SLAM**：地图从未知（205）开始，随机器人移动 + 360° 雷达扫描逐步揭示（free 254 / occupied 0）；
  内容变化才递增 `map.version` 并推送 `map` 事件；`/api/map.png` 为合法可解码 PNG。
- **导航**：首次运行在 `pc_console/demo_maps/` 生成 `demo_home.pgm/.yaml`（出现在 `/api/maps` 且可预览）；
  `initialpose` → 2~3 s 激活（`activating` → `ready`）→ 点目标 → A* 规划（推送 `path` 事件）
  → 沿路径行驶 → `succeeded`；不可达目标 `failed/aborted`；支持取消；导航中遥控会取消目标。
- **遥控**：麦轮全向积分 + 墙体碰撞保护（撞墙停）；500 ms 无指令自动停车；急停锁存/解除。
- **会话**：启动/停止 1~3 s 延迟；完整走 `starting → running → stopping → idle`，
  坏地图/加载失败走 `session.state=error`；模式切换自动「停止→启动」。
- **其它**：保存地图（PGM+YAML，正确 y 行序）、地图列表/预览、6 条诊断项（OK/WARN 聚合）、
  事件日志、会话日志、`/api/ros`、外设回显、SSE 多客户端（上限 4，超出 503）。
- **时序**：遥测 `state` 10 Hz、`scan` 5 Hz、仿真步进 50 Hz（≥20 Hz）。

---

## 7. 测试与验收

### 7.1 单元 / 集成测试（离线，全绿）

```bat
cd pc_console
python -m unittest discover -s tests -v
```

覆盖：配置读写/校验/优先级、演示 health/state schema、SSE（chunked + state/scan/map/path 事件）、
SLAM 建图与保存地图（PGM/YAML/预览/地图列表）、导航 initialpose→ready→goal→succeeded、
不可达/取消/急停取消、遥控位姿变化+看门狗、急停 409、错误路径 400/404/405/409/413、
代理转发（HTTP 状态/二进制/token/SSE）、上游不可达 502、超时 504、目标切换。

本地实测（2026-11-24，Windows 11 / Python 3.14.7）：

```
Ran 62 tests in 37.2s
OK
```

> 测试会在 `pc_console/_tmp/` 下自建临时目录并在结束后清理，不依赖网络、不占用 8080/8090。

### 7.2 编译检查

```bat
python -m compileall -q pc_console
```

### 7.3 手动冒烟（DoD 第 3 条）

终端 1：

```bat
cd pc_console
python main.py --demo --port 8099 --no-browser
```

终端 2（Windows PowerShell / curl / 任意 HTTP 客户端）：

```bat
:: 1) 存活探针
curl http://127.0.0.1:8099/api/health

:: 2) 状态快照
curl http://127.0.0.1:8099/api/state

:: 3) 地图 PNG（需先进入带地图的会话：slam 扫描后或 nav 模式）
curl -o map.png http://127.0.0.1:8099/api/map.png
```

跨平台 Python 版（不需要进入会话也能验证 health/state；map.png 需先启动 nav）：

```python
import json, time, urllib.request

BASE = "http://127.0.0.1:8099"

def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return r.status, r.read()

def post(path, body):
    req = urllib.request.Request(
        BASE + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.status, json.loads(r.read())

print(get("/api/health"))                                  # 200，含 "demo": true
print(get("/api/state")[0])                                # 200
status, raw = get("/api/maps")
nav_map = next(m["path"] for m in json.loads(raw)["maps"] if m["name"] == "demo_home")
print(post("/api/session/start", {"mode": "nav", "map": nav_map}))   # 202
for _ in range(80):
    state = json.loads(get("/api/state")[1])
    if state["session"]["state"] == "running":
        break
    time.sleep(0.1)
status, png = get("/api/map.png")
print(status, len(png), png[:8])                           # 200 <len> b'\x89PNG...'
print(post("/api/session/stop", {}))                       # 202
```

### 7.4 契约一致性检查（WP-C4 工具，集成时使用）

只读检查（不改变车辆状态）：

```bat
python tools/console/contract_check.py --base http://127.0.0.1:8099
```

全流程检查（会启停会话/遥控/导航）。**演示模式请显式指定演示地图与坐标**：
默认 `--initialpose 0 0 0`、`--goal 0.8 0 0` 是给真车地图约定用的，而演示地图原点为 `(0, 0)`、
起点 `(1.5, 1.5)`，默认坐标落在墙体内：

```bat
python tools/console/contract_check.py --base http://127.0.0.1:8099 --full ^
    --nav-map demo_home --initialpose 1.5 1.5 0 --goal 2.5 1.5 0
```

本地实测：只读 `8 PASS / 0 FAIL / 1 SKIP`；全流程 `33 PASS / 0 FAIL / 1 SKIP`（20.4 s）。

---

## 8. 已知限制与假设

### 8.1 已知限制

1. **PC 程序退出不代表 Pi 会话停止**：真车模式下 Ctrl-C 只关闭本地代理与 UI；Pi 上由网关托管的
   ROS bringup / 已下发导航目标仍按 Pi 侧看门狗与 Nav2 策略继续运行。需要停车请：
   先点「急停」或「停止会话」，再退出 PC 程序（参考设计文档 §4.1、§5.1）。
2. **演示模式不是物理仿真**：只用于 UI/协议验收，不模拟 Nav2 调参效果、雷达噪声、打滑、
   电机响应；`initialpose` 直接把机器人位姿设为给定值（真实网关为 AMCL 修正）。
3. 演示模式地图保存目录固定为 `pc_console/demo_maps/`（真实网关为 `/mnt/ssd/maps` 等）。
4. 无编码器/真实定位精度等硬件限制见主 README 与设计文档 §9。
5. Windows 下浏览器 app 窗口依赖本机安装 Edge/Chrome；找不到时回退默认浏览器。
6. 单机 SSE 客户端上限 4（演示模式同样生效，超出返回 503）；浏览器断线由 EventSource
   自动重连（首帧前发送 `retry: 2000`）。
7. 静态资源仅暴露白名单后缀；Web UI 缺失时 `/` 返回 404 文本，但 `/api/*` 与 `/local/*` 不受影响。

### 8.2 与契约的偏差 / 实现假设（均不影响前端兼容）

1. `GET /api/health` 演示模式**追加** `"demo": true` 字段（契约允许新增字段）。
2. `map.available=false` 时，演示后端返回**数值占位**（width/height=0、resolution=0.0、
   origin 全 0、stamp=0.0）而不是 `null`；契约允许 null，占位便于严格 schema 校验，前端在
   `available=false` 时必须忽略这些字段。
3. 上游**超时**契约未规定状态码，本实现返回 `504 {"error":{"code":"upstream_timeout"}}`；
   连接失败返回 `502 upstream_unreachable`。连接阶段超时按「不可达」处理（502）。
4. `POST /local/config` 演示模式返回 `409 {"error":{"code":"demo_mode"}}`。
5. `GET /local/ping` 演示模式返回 `reachable=true, target={"host":"demo","port":0}, demo=true`。
6. `POST /api/nav/activate` 演示模式会取消当前目标并重新走一遍 2~3 s 激活流程。
7. 导航处于 `succeeded / failed / canceled` 时，演示后端允许直接发送新目标（真实网关通常需
   重新设置初始位姿或重新激活；UI 仍应按契约在 `ready` 时发送）。
8. 演示模式 `goal` 的 `yaw` 仅回显、到达判定用位置阈值 0.15 m（不要求最终朝向）。
9. 演示模式 `use_mock` 字段被忽略但记录到 `session.mock`（契约字段保留）。
10. `tools/console/contract_check.py`（WP-C4）当前没有 `--demo` 参数；设计文档 §8.1 中带
    `--demo` 的命令需要改为本节 7.4 的参数形式（属工具侧差异，已反馈 lead）。
11. 演示后端错误码 / 响应结构与 Pi 网关（WP-C1）对齐：会话未运行时的遥控/外设返回
    `409 session_not_running`、保存地图返回 `409 session_idle`、导航未就绪
    `409 nav_not_ready`、非法地图名 `400 bad_request`（detail.field=name）、
    `/api/ros` 的 `topics` 为 `[{name,types}]` 结构；急停放行 `initialpose`/`nav/activate`
    （契约只禁止 session start / goal / teleop）。

---

## 9. 故障排查

| 现象 | 排查 |
|---|---|
| 浏览器打不开 / 连不上 | 确认控制台终端打印的 URL；检查本地端口是否被占用（`netstat -ano \| findstr 8090`）；用 `--port 8099` 换端口 |
| 真车模式一直「未连接」 | 用 `GET /local/ping` 看错误；确认 PC 与 Pi 同网段、Pi 网关已启动、token 一致；可先用 `--fallback-demo` 验证 PC 程序本身 |
| 502 `upstream_unreachable` | Pi 网关未启动/地址端口错/防火墙；`/local/config` 修改后即时生效 |
| 演示模式没有地图 | `/api/map.png` 在 `idle/base` 下返回 404 属正常；先开始建图或导航（`/api/session/start`） |
| 导航目标失败 `aborted` | 目标点在墙/家具/未知区域内不可达；点击地图空闲区域重试 |
| 端口被占用 | 默认只监听 127.0.0.1；指定 `--port 0` 可由系统分配临时端口并打印实际 URL |
| 想彻底停止车辆 | 先「急停」或「停止会话」，再退出 PC 程序；然后可在 Pi 上停止 ROS 会话 |

---

## 10. 相关文档

- `docs/PC上位机_设计文档_v1.md` —— 接口契约（冻结）
- `docs/PC上位机_操作手册_v1.md` —— 使用手册（WP-C4）
- `tools/console/` —— 部署、契约检查、验收（WP-C4）
- `ros2_ws/src/raspbot_console/` —— Pi 网关（WP-C1）
