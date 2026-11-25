# RASPBOT-V2 PC 上位机控制台 · 设计文档与接口契约 v1.0

- 文档状态：**实现契约（冻结）**，v1.0，2026-11-24
- 适用系统：RASPBOT-V2（Pi 5 / Ubuntu 26.04 / ROS 2 Lyrical）＋ PC（Windows 10/11，浏览器）
- 关联文档：`README.md`、`ros2_ws/CONTRACTS.md`、`docs/手机端SLAM导航_可行性分析_v1.md`
- 本文是 **WP-C1（Pi 网关）/ WP-C2（PC 程序）/ WP-C3（Web UI）** 的共同契约。实现前必须通读；任何接口改动先改本文并通知所有工作包。

---

## 0. 结论与范围

在一台 PC 上提供一个**观察 + 控制**小车的控制台，覆盖：

1. **观察**：实时地图、机器人位姿/朝向、激光点、导航路径、速度、诊断、节点/话题健康、连接状态。
2. **建图**：一键「开始建图 / 停止建图 / 保存地图」，建图过程中可遥控，地图实时刷新。
3. **导航**：选择已保存地图启动导航 → 在图上设置初始位姿 → 点击地图设置目的地 → 查看剩余距离/状态 → 取消。
4. **手动控制**：键盘/虚拟摇杆遥控（麦轮全向）、速度档位、松手即停、软件急停。
5. **附加**（按需实现，低成本高价值）：蜂鸣器/ RGB 灯/云台舵机、锚点式状态面板、事件日志、会话日志、地图列表与预览、演示模式（无硬件仿真）。

**不做**（v1 明确不做，避免过度承诺）：

- 不做视频/图传。依据 `docs/手机端SLAM导航_可行性分析_v1.md` §8：SLAM 场景下 UI 主体是 2D 地图，视频会把带宽/复杂度拉高一个量级。相机与 Hailo 感知仍由 ROS 侧按原方式局部使用。
- 不做公网远控、账号体系、多用户。默认同一局域网直连；网关只应监听内网。
- 不承诺无编码器条件下的精确走位；精度边界遵循主 README「已知限制」。
- 不修改 `CONTRACTS.md` 已冻结的话题/TF 契约；本系统只新增进程、端口、HTTP/SSE 接口，**不新增任何 TF 发布者**。

**关键前提（必须在 UI/文档中向用户展示）**：

- 无轮编码器 → `/odom` 是开环积分，长距离漂移；导航定位依赖 AMCL 激光修正。
- Nav2 生命周期不 autostart → 导航模式必须先设置初始位姿，再由网关按顺序激活。
- 2D 雷达扫描面离地约 74 mm → 看不见低矮/玻璃/镜面障碍，必须有人监护并限定场地。

---

## 1. 总体架构

```
┌────────────────────────── PC（Windows） ──────────────────────────┐
│  浏览器（Edge/Chrome app 窗口）                                   │
│    Web UI  http://127.0.0.1:8090/                                 │
│        │ 同源 fetch + EventSource（/api/* 由本地程序代理）         │
│        ▼                                                          │
│  pc_console（Python 3.10+，仅标准库）                              │
│    ├─ 静态资源（web/）                                             │
│    ├─ /local/*：配置、目标切换、连通性探测                          │
│    ├─ /api/*  → 反向代理到 Pi 网关（SSE 透传、二进制透传）           │
│    └─ --demo：内置演示仿真后端（无需 Pi/ROS，协议与 Pi 网关一致）    │
└───────────────────────────────┬───────────────────────────────────┘
                                │ HTTP / SSE（同局域网，默认 :8080）
                                ▼
┌────────────────────────── Raspberry Pi 5 ─────────────────────────┐
│  raspbot_console（ROS 2 包，rclpy，Python 3.12）                  │
│    ├─ HTTP+SSE 网关 :8080（线程化，仅标准库 + rclpy）              │
│    ├─ ROS 适配层：订阅 /map /odom /scan /plan /diagnostics /tf     │
│    ├─ 控制层：/cmd_vel_teleop、/estop、/initialpose、              │
│    │          /buzzer、/led、/servo_pan、/servo_tilt               │
│    ├─ 导航层：Nav2 NavigateToPose action + lifecycle 激活编排       │
│    ├─ 会话层：拉起/停止 `ros2 launch raspbot_bringup ...`           │
│    └─ 地图层：/map → PNG；保存 PGM+YAML；地图列表/预览              │
└───────────────────────────────────────────────────────────────────┘
```

### 1.1 进程与端口

| 组件 | 位置 | 端口 | 启动方式 |
|---|---|---|---|
| PC 控制台程序 | PC | `127.0.0.1:8090`（可配置） | `pc_console/start.bat` 或 `python main.py` |
| Pi 网关 | Pi | `0.0.0.0:8080`（可配置） | `ros2 run raspbot_console console_bridge`（或 systemd） |
| ROS bringup | Pi | 内部 | 由网关按会话模式拉起，或外部 systemd 启动（`--attach` 附加） |

### 1.2 设计原则（硬约束）

1. **网关只消费、不拥有 TF**：不发布 `map→odom`/`odom→base_link` 等任何 TF；不修改既有话题名/类型。
2. **单一执行入口**：网关只发布 `/cmd_vel_teleop`（经 twist_mux 汇入 `/cmd_vel`），绝不直接发布 `/cmd_vel` 或 `/cmd_vel_safe`。Nav2 仍走 `/cmd_vel_nav`。
3. **一个 ROS 图一个 owner**：网关管理 bringup 时，若检测到外部已有 bringup 节点，拒绝再拉起（除非 `--attach` 附加模式）。
4. **安全默认值**：网关启动/退出/断连/会话停止路径都必须保证零速；急停锁存直到显式解除；看门狗 0.5 s。
5. **PC 与 Pi 解耦**：PC 端不安装 ROS；Pi 端不依赖浏览器；两端只用 HTTP/SSE 契约通信。
6. **零第三方依赖**：PC 端与 Pi 网关只用 Python 标准库（Pi 额外依赖 rclpy/ROS 消息包）；前端不用任何 CDN/构建工具，离线可用。
7. **演示可替代真车**：PC 端 `--demo` 必须实现与 Pi 网关**完全一致**的 API，用于无硬件验收和 UI 联调。

---

## 2. 功能清单与验收映射

| 编号 | 功能 | 优先级 | 验收 |
|---|---|---|---|
| C-F01 | 实时显示地图、机器人位姿/朝向、激光点、导航路径 | P0 | 演示模式 + Pi SIL 均可见，位置换算正确（TC-C2） |
| C-F02 | 一键开始/停止建图，地图实时刷新 | P0 | 演示模式 2 s 内出现首帧地图；SIL 同 |
| C-F03 | 保存地图（命名、落盘 PGM+YAML、列表可见） | P0 | 产物成对存在，尺寸/分辨率正确 |
| C-F04 | 选择地图开始导航 + 设置初始位姿 + 点击目的地 | P0 | 演示模式目标 SUCCEEDED；SIL 目标 SUCCEEDED |
| C-F05 | 导航状态/剩余距离/取消/失败原因 | P0 | 状态机与文案正确 |
| C-F06 | 遥控（键盘+摇杆）、速度档、松手即停 | P0 | 速度指令经 `/cmd_vel_teleop`；500 ms 内停 |
| C-F07 | 软件急停（锁存/解除）与状态回显 | P0 | 急停后速度归零、导航取消；解除需二次确认 |
| C-F08 | 连接状态、自动重连、断连提示 | P0 | 拔网/停 Pi 后 ≤3 s 提示，恢复后自动刷新 |
| C-F09 | 诊断/节点/话题健康、事件日志、会话日志 | P1 | 面板可见，错误高亮 |
| C-F10 | 蜂鸣器/灯光/云台（附加控制） | P2 | 发对应话题，状态回显 |
| C-F11 | 无硬件演示模式（仿真场景） | P0 | 全功能可演示，含建图/导航闭环 |
| C-F12 | Pi 一键部署脚本 + 台架 SIL 验收脚本 | P1 | 脚本可执行、含回滚说明 |

---

## 3. 接口契约（Wire API v1）

### 3.0 通用约定

- Base URL：`http://<pi-host>:8080`，所有接口以 `/api/` 开头；PC 端另提供 `/local/`。
- 编码：`application/json; charset=utf-8`；二进制见 `map.png`。
- 时间戳：Unix 秒（float）；持续时间：秒（float）；角度：弧度；长度：米。
- 鉴权：可选。网关配置 `--token <str>` 后，请求必须带 `X-Console-Token: <str>`，GET 流式接口可改用 `?token=<str>`。
- CORS：允许 `*`、方法 `GET, POST, OPTIONS`、请求头 `Content-Type, X-Console-Token`。
- 限流/体积：JSON 请求体 ≤ 64 KB；单机 SSE 客户端 ≤ 4（超出 503）。
- 错误响应统一：

```json
{"error": {"code": "bad_request", "message": "人类可读的原因", "detail": {"field": "x"}}}
```

HTTP 状态约定：`400` 参数错误、`401` token 错误、`404` 不存在、`405` 方法不允许、`409` 状态冲突（模式不对/急停中/非导航模式等）、`503` ROS 未就绪或网关忙、`500` 内部错误。

- **所有状态机字段的权威值来自 `GET /api/state` 与 SSE `state` 事件**；POST 返回成功仅表示「命令已受理」。

### 3.1 状态快照 `GET /api/state`

`GET /api/state` 返回完整快照；SSE `state` 事件负载为 `{"state": <同一 schema>}`，10 Hz 推送。**前端不得依赖未在本文出现的字段**；新增字段只能追加。

```json
{
  "api": 1,
  "server_time": 1764012345.678,
  "uptime_s": 812.4,
  "session": {
    "mode": "idle",                  // idle | base | slam | nav
    "state": "idle",                 // idle | starting | running | stopping | error
    "mock": false,
    "managed": true,                 // false = --attach 附加外部栈
    "external": false,               // 检测到外部 bringup 且本网关未管理
    "available_modes": ["base", "slam", "nav"],
    "since": 1764011000.0,
    "progress": 0.0,                 // 0..1，启动/停止时用于进度条
    "message": "",                   // 中文一句话状态
    "error": null,                   // 启动失败/进程异常退出的原因
    "map": null                      // nav 模式的地图 yaml 绝对路径
  },
  "ros": {
    "ok": true,                      // 核心链路可用（odom + cmd_vel 链）
    "nodes": 27,
    "ready": {
      "base": true,                  // /odom 与 /cmd_vel_safe 有发布者
      "teleop": true,                // /cmd_vel_teleop 有订阅者（twist_mux）
      "map": true,                   // /map 有发布者且网关已收到地图
      "localization": false,         // amcl 节点存在
      "nav2": false,                 // /navigate_to_pose action server 就绪
      "diagnostics": true            // 5 s 内收到 /diagnostics
    },
    "rates": {"scan_hz": 10.1, "odom_hz": 30.0, "map_age_s": 0.4, "diagnostics_age_s": 0.7}
  },
  "limits": {"max_linear": 0.30, "max_lateral": 0.30, "max_angular": 1.5},
  "robot": {
    "frame": "map",                  // map | odom（map 未就绪时退化为 odom）
    "x": 1.23, "y": -0.40, "yaw": 1.57,
    "stamp": 1764012345.60,
    "speed": {"vx": 0.12, "vy": 0.0, "wz": 0.0},
    "imu": {"yaw_deg": 89.9, "age_s": 0.02},
    "scan": {"front_m": 1.35, "min_m": 0.28, "age_s": 0.1}
  },
  "nav": {
    "state": "unavailable",          // unavailable | waiting_initialpose | activating | ready | goal_active | succeeded | failed | canceled | error
    "message": "未进入导航模式",
    "initialpose": null,             // {"x","y","yaw","stamp"}
    "goal": null,                    // {"x","y","yaw","stamp","goal_id"}
    "goal_state": "idle",            // idle | accepted | active | succeeded | canceled | aborted
    "distance_remaining": null,
    "navigation_time_s": null,
    "recoveries": null,
    "lifecycle": {},                  // {"controller_server":"active",...}
    "activation": {"done": 0, "total": 6, "phase": ""}
  },
  "map": {
    "available": true,
    "version": 12,
    "width": 240, "height": 160,
    "resolution": 0.05,
    "origin": {"x": -2.4, "y": -2.0, "yaw": 0.0},
    "frame_id": "map",
    "stamp": 1764012345.2
  },
  "estop": {"active": false, "since": null},
  "teleop": {"active": false, "vx": 0.0, "vy": 0.0, "wz": 0.0, "last_cmd": null},
  "peripherals": {
    "buzzer": false,
    "led": {"r": 0.0, "g": 0.0, "b": 0.0},   // 0..1，与 ColorRGBA 一致
    "servo": {"pan": 90.0, "tilt": 45.0}     // deg；tilt 上限 110
  },
  "diagnostics": {
    "level": "ok",                   // ok | warn | error | unknown
    "counts": {"ok": 8, "warn": 0, "error": 0},
    "items": [                        // 最多 12 条，非 OK 优先
      {"name": "Raspbot Safety: E-Stop", "level": "ok", "message": "OK", "hardware_id": "estop"}
    ],
    "age_s": 0.6
  },
  "logs": [                           // 最近 20 条网关日志，倒序无关（前端按 ts 排序）
    {"ts": 1764012345.1, "level": "info", "msg": "地图已保存: room1"}
  ]
}
```

补充规则：

- `robot.frame` 为 `map` 时位姿来自 TF `map→base_link`；否则为 `odom→base_link` 或 `/odom` 位姿，UI 必须显示「未定位」提示。
- `session.mode` 表示目标/当前运行模式；`session.state=running` 才允许遥控；导航模式下 `nav.state` 才是导航阶段权威。
- 无 ROS 数据时 `robot` 字段可为 `null`；`map.available=false` 时其余地图字段可为 `null`。前端必须容错。
- `estop.active` 优先反映网关命令状态；若网关启动后先从诊断中收到 `Raspbot Safety: E-Stop` 的 `estop=True`，则同步为 true（不自动解除）。

### 3.2 SSE `GET /api/events`

- 响应头：`Content-Type: text/event-stream; charset=utf-8`、`Cache-Control: no-cache`、`X-Accel-Buffering: no`。
- **传输方式**：必须使用 HTTP/1.1 + `Transfer-Encoding: chunked`（推荐），或等价的逐帧增量写法；每帧立即 flush，**不得攒批/缓冲整段**。禁止带 `Content-Length`。
- 首帧前发 `retry: 2000\n\n`；每 10 s 无数据发注释 `: ping\n\n`。
- 事件类型：

| event | data | 频率/触发 |
|---|---|---|
| `state` | `{"state": <§3.1 快照>}` | 10 Hz |
| `scan` | `{"stamp": 176..., "frame": "map", "points": [x0,y0,x1,y1,...]}` | 5 Hz，最多 360 点（扁平数组） |
| `path` | `{"stamp": 176..., "frame": "map", "points": [x0,y0,...]}` | `/plan` 变化时，节流 2 Hz，最多 200 点 |
| `map` | `{"meta": <§3.1 map 对象>}` | 地图内容变化（version 增加）时 |
| `session` | `{"session": <§3.1 session 对象>}` | 会话状态变化时立即推送 |
| `log` | `{"ts":..., "level":"info|warn|error", "msg": "..."}` | 网关事件即时 |

- 前端收到 `map` 后应请求 `GET /api/map.png?v=<version>` 刷新底图；地图图片允许长缓存（版本号变化即换 URL）。

### 3.3 端点清单

| 方法/路径 | 说明 | 关键响应 |
|---|---|---|
| `GET /api/health` | 网关存活探针（**免 token**） | `{"ok":true,"name":"raspbot-console","version":"1.0.0","api":1,"server_time":...,"uptime_s":...}` |
| `GET /api/state` | 完整状态快照 | §3.1 |
| `GET /api/events` | SSE 事件流 | §3.2 |
| `GET /api/map/meta` | 地图元数据 | §3.1 `map` 对象（无图时 `{"available":false}`） |
| `GET /api/map.png?v=N` | 地图图片（`image/png`，灰度 8-bit） | 无图时 404 JSON |
| `GET /api/maps` | 已保存地图列表 | `{"maps":[{...}],"dirs":[...]}`，见下 |
| `GET /api/maps/<name>/preview.png` | 地图缩略图（P5 PGM→PNG） | `image/png`；无预览 404 |
| `POST /api/map/save` | 保存当前 `/map` 为 PGM+YAML | `{"ok":true,"name","yaml","pgm","width","height","resolution","mtime"}` |
| `POST /api/session/start` | 启动/切换运行模式 | `202 {"ok":true,"session":{...}}` |
| `POST /api/session/stop` | 停止托管 bringup | `202 {"ok":true,"session":{...}}`；空闲时 `200 {"ok":true}` |
| `POST /api/teleop` | 发送遥控速度（20 Hz 由前端节流） | `200 {"ok":true}` |
| `POST /api/teleop/stop` | 立即零速并退出遥控 | `200 {"ok":true}` |
| `POST /api/estop` | 急停锁存/解除 | `200 {"estop":{"active":...}}` |
| `POST /api/goal` | 发送 Nav2 目的地 | `202 {"ok":true,"goal":{...},"goal_id":"..."}` |
| `POST /api/goal/cancel` | 取消当前导航目标 | `200 {"ok":true}` |
| `POST /api/initialpose` | 发布初始位姿（导航模式） | `200 {"ok":true,"nav":{"state":"activating"}}` |
| `POST /api/nav/activate` | 手动重试 Nav2 激活 | `202 {"ok":true}` |
| `POST /api/actuator/buzzer` | 蜂鸣器 | `200 {"ok":true}` |
| `POST /api/actuator/led` | RGB 灯 | `200 {"ok":true}` |
| `POST /api/actuator/servo` | 云台 pan/tilt | `200 {"ok":true}` |
| `GET /api/logs?limit=200` | 网关事件日志（最多 300） | `{"logs":[...]}` |
| `GET /api/session/logs?limit=200` | 当前/上次 bringup 的 stdout 尾部 | `{"logs":[{"ts","line"}]}` |
| `GET /api/ros` | 节点/话题调试信息 | `{"nodes":[...],"topics":[...]}`（可裁剪） |

#### 3.3.1 `GET /api/maps` 条目

```json
{
  "maps": [{
    "name": "room",
    "path": "/mnt/ssd/maps/room.yaml",
    "yaml": "/mnt/ssd/maps/room.yaml",
    "pgm": "/mnt/ssd/maps/room.pgm",
    "mtime": 1764012000.0,
    "resolution": 0.05,
    "width": 240,
    "height": 160,
    "preview_url": "/api/maps/room/preview.png",
    "valid": true
  }],
  "dirs": ["/mnt/ssd/maps", "/home/ubuntu/ros2_ws/maps"]
}
```

- 名称必须来自 yaml 文件名（去扩展名）；同名时优先 `/mnt/ssd/maps`。
- `valid=false` 的条目（缺 pgm/字段坏）仍返回，UI 置灰不可选。

#### 3.3.2 `POST /api/map/save`

请求：

```json
{"name": "room_20261124_2310", "overwrite": false}
```

- `name` 规则：`^[A-Za-z0-9_.-]{1,40}$`（禁止路径分隔符/空格/中文，降低跨平台风险）。
- 保存目录：`/mnt/ssd/maps` 若存在且可写，否则回退 `~/ros2_ws/maps`（与 `tools/slam/save_map.sh` 一致）。服务端确保目录存在。
- 保存内容：最新 `/map` 的完整快照 → `name.pgm`（P5，free=254 / occupied=0 / unknown=205，行序按 PGM 从上到下即 y 由大到小）＋ `name.yaml`：

```yaml
image: room_20261124_2310.pgm
mode: trinary
resolution: 0.05
origin: [-2.4, -2.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

- 冲突：同名存在且 `overwrite != true` → `409 map_exists`；无地图 → `409 no_map`；会话未在运行 → `409 session_idle`。
- 保存后推送 `log`，并刷新 `/api/maps` 数据（无需 SSE 事件）。

#### 3.3.3 会话控制

`POST /api/session/start`：

```json
{"mode": "slam", "map": null, "use_mock": false, "use_camera": false, "use_perception": false}
```

- `mode`：`base | slam | nav`。
- `nav` 必须给 `map`（绝对路径 `.yaml`，存在且可读），否则 `400 bad_request`。
- 若 `estop.active` → `409 estop_active`；若正在启动/停止 → `409 busy`；若外部栈存在且本网关未管理 → `409 external_stack`（除非 `--attach`）。
- 若当前为其他模式的 running，则自动「停止→启动」切换（同一请求内完成，`session.state` 经 `stopping/starting`）。
- `use_mock=true` 时用于台架/SIL：对应 bringup 参数 `use_mock:=true`；`mock` 会出现在 session 中。
- 返回 202 后，前端必须依赖 `state/session` 事件显示进度；`session.state=running` 才可用后续功能。

`POST /api/session/stop`：空闲时 200；运行中 202 并异步停止（SIGINT 进程组，20 s 超时后 SIGTERM 5 s 再 SIGKILL）。停止过程中拒绝新的 start（409 busy）。

#### 3.3.4 遥控 `POST /api/teleop`

```json
{"vx": 0.15, "vy": 0.0, "wz": 0.0, "seq": 1024}
```

- 单位 m/s、rad/s；`seq` 可选，网关按 seq 丢弃过期帧（同一次按键序列内单调递增）。
- 网关将值 clamp 到 `state.limits`（超出不报错，clamp 后执行），非法/非数值 → 400。
- 处理：若当前有活动导航目标 → **先取消导航**（`log` 提示），再执行遥控；若 estop 锁存 → 409。
- 网关内部以 20 Hz 发布 `/cmd_vel_teleop`；500 ms 未收到新帧 → 自动发布零速并置 `teleop.active=false`（看门狗，见 §5.1）。
- 允许的会话：`base/slam/nav` 的 running；idle → 409。
- 前端只在按键/摇杆有输入时以 20 Hz 发送；松手立即 POST `/api/teleop/stop`。

#### 3.3.5 急停 `POST /api/estop`

```json
{"active": true}
```

- `true`：发布 `/estop=true`；取消当前导航目标；停止遥控；立即发零速；此后每秒重发一次 `/estop=true`，直到显式 false（防止下游重启后遗忘）。
- `false`：发布一次 `/estop=false`，恢复。
- 急停期间：会话 start、goal、teleop 全部 409；session stop 允许。
- 响应回显 `{"ok":true,"estop":{"active":true,"since":...}}`。

#### 3.3.6 导航目标 `POST /api/goal`

```json
{"x": 1.5, "y": 0.0, "yaw": null}
```

- 前置：`session.mode=nav && session.state=running && nav.state=ready`；否则 409（附当前状态文案）。
- `yaw=null` 时使用机器人当前 yaw（`robot.yaw`）。
- 网关构造 `nav2_msgs/action/NavigateToPose`（frame `map`）并 `send_goal_async`；被拒绝 → 503 `nav_rejected`。
- 目标状态经 SSE `state.nav` 推送；`goal_id` 为网关 UUID，便于前端匹配。
- 只允许单目标：已有活动目标再发 → 409 `goal_active`（前端应先取消）。

#### 3.3.7 初始位姿 `POST /api/initialpose`

```json
{"x": 0.0, "y": 0.0, "yaw": 0.0}
```

- 仅 `mode=nav && running` 可用。
- 网关发布 `geometry_msgs/PoseWithCovarianceStamped`（frame `map`，协方差沿用
  `[0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]`），并在 2 Hz 重复发布最多 20 s 直到 TF `map→base_link` 可用。
- TF 可用后自动进入 Nav2 激活流程（`nav.state=activating`），步骤见 §5.3；成功后 `nav.state=ready`。
- 重复调用可在激活中/就绪后重新定位（会重置 AMCL 位姿；若导航进行中则先取消目标）。

#### 3.3.8 外设

- `POST /api/actuator/buzzer`：`{"on":true}` → 发布 `/buzzer` Bool。
- `POST /api/actuator/led`：`{"r":0.0,"g":0.5,"b":1.0}`（各 0..1，clamp）→ 发布 `/led` ColorRGBA（a=1）。
- `POST /api/actuator/servo`：`{"pan":90.0,"tilt":45.0}` → 发布 `/servo_pan`、`/servo_tilt` Float32；pan clamp 0–180，tilt clamp 0–110（**机械限位**）。
- 仅在会话 running 时可用（否则 409）。网关不读取真实角位置，`state.peripherals` 为「最后命令值」。

---

## 4. PC 端程序（`pc_console/`）

### 4.1 启动与配置

```bash
# 连接真车（默认目标 192.168.50.98:8080）
python main.py                 # 或双击 start.bat：启动本地服务并打开 app 窗口
python main.py --pi 192.168.50.98:8080 --port 8090

# 无硬件演示（仿真协议一致，含建图/导航闭环）
python main.py --demo          # 或双击 start-demo.bat

# 常用参数
--host 127.0.0.1        # 本地监听地址（默认仅本机）
--port 8090             # 本地端口
--token XXX             # 访问 Pi 网关的 token（也可写 config.json / UI 设置）
--browser auto|app|default|none   # app = Edge/Chrome --app 窗口；默认 auto
--no-browser
--fallback-demo         # 启动时探测不到 Pi 则自动切换演示模式
--verbose
```

- 配置文件：`pc_console/config.json`（首次运行按 `config.example.json` 生成；**不提交用户私有配置**）。优先级：CLI > `config.json` > 默认值。
- 本地端点：
  - `GET /local/config` → `{"demo":false,"version":"1.0.0","web":{"host":"127.0.0.1","port":8090},"target":{"host":"192.168.50.98","port":8080},"token_set":false,"config_path":"..."}`
  - `POST /local/config` `{"pi_host":"192.168.50.98","pi_port":8080,"token":""}` → 校验并持久化、立即切换代理目标；演示模式下 409。UI 设置对话框使用。
  - `GET /local/ping` → 探测目标 `/api/health`：`{"reachable":true,"latency_ms":12.3,"target":{...},"error":null}`。
- 静态资源：`/` → `web/index.html`；只暴露 `web/` 下白名单后缀（`.html/.css/.js/.mjs/.svg/.png/.ico/.json/.woff2`），禁止 `..` 穿越。
- `/api/*` 代理：保留方法/状态码/Content-Type；SSE 逐行透传不缓冲；上游不可达返回 `502 {"error":{"code":"upstream_unreachable",...}}`；后端超时 5 s（SSE 除外）。
- 退出：Ctrl-C 优雅关闭；不影响 Pi 侧会话（车仍由 ROS 看门狗/已下发目标控制；文档需说明）。

### 4.2 演示模式（`--demo`）

- 在 PC 本地进程内实现与 §3 **一致**的 API（含 SSE），不依赖 ROS/Pi/网络。
- 场景：12 m × 8 m 室内（外墙 + 两个房间 + 走廊 + 若干家具），0.05 m 栅格；机器人起点约 `(1.5, 1.5, 0°)`。
- 行为：
  - `slam`：地图从未知开始，随机器人移动/雷达扫描逐步揭示；`GET /api/map.png`、`map` 事件、保存地图均可用。
  - `nav`：使用演示地图（首次运行生成于 `pc_console/demo_maps/demo_home.yaml/pgm`，并出现在 `/api/maps`）；设置初始位姿 → 2–3 s 激活 → 点目标 → A* 路径 + 沿路径行驶 → `succeeded`；支持取消、失败（不可达目标 `aborted`）。
  - 遥控：麦轮全向积分 + 墙体碰撞检测（撞墙即停）；急停、外设、诊断、日志全部回显。
  - 会话启动/停止带 1–3 s 延迟，模拟真实启动；`use_mock` 字段忽略但保留。
- 演示地图与日志写入 `pc_console/demo_maps/`（运行时生成，git 忽略）；不得污染真实配置。
- 演示模式须能作为端到端测试夹具（见 §8）。

### 4.3 浏览器启动

- `--browser app`：优先找 Edge（`C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe` 等）或 Chrome，执行 `--app=http://127.0.0.1:8090/ --window-size=1440,900`；找不到则回退系统默认浏览器。
- `auto`：Windows 下优先 app 窗口；Linux 用默认浏览器。
- 打开失败不能影响服务运行；控制台打印 URL。

---

## 5. Pi 网关（`ros2_ws/src/raspbot_console/`）

### 5.1 线程模型与安全

- rclpy executor 独立线程（`MultiThreadedExecutor`，2–3 线程）；HTTP/SSE 线程池；会话监控线程。
- 发布从任意线程调用（rclpy 允许），共享状态一律加锁；SSE 每客户端独立 `queue.Queue(maxsize=200)`，丢弃最旧。
- **遥控看门狗**：收到 `/api/teleop` 记 `last_cmd=now`；20 Hz 定时器发布；`now-last_cmd>0.5 s` 时发布两次零速、`teleop.active=false`、停止发布。
- **导航中收到遥控** → 取消导航目标（避免 `twist_mux` 双源竞争与 Nav2 继续占用规划器）。
- **急停**：见 §3.3.5；订阅 `/diagnostics` 解析 `Raspbot Safety: E-Stop` 的 `estop=True/False` 作为启动期同步来源。
- **进程退出**（SIGINT/SIGTERM）：停止遥控（零速）、停止托管会话（SIGINT → 等待 → SIGKILL 进程组）、关闭 HTTP；不阻塞 >30 s。
- 网关崩溃不得影响底盘：不得打开 I2C/串口，不得成为 `/cmd_vel` 唯一链路（twist_mux/limiter 仍在）。

### 5.2 会话管理（`session.py`）

- 启动命令（统一 source 后 exec，`start_new_session=True` 便于整组停止）：

```bash
bash -lc 'source /opt/ros/lyrical/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && \
  exec ros2 launch raspbot_bringup bringup.launch.py mode:=<none|slam|nav> [map:=<abs.yaml>] [use_mock:=true]'
```

- 模式映射：`base→mode:=none`，`slam→mode:=slam`，`nav→mode:=nav map:=<path>`。
- 就绪判据（每 0.5 s 轮询，超时 120 s）：
  - 进程存活；
  - `base`：`count_publishers('/odom')>0 && count_publishers('/cmd_vel_safe')>0`；
  - `slam`：在 base 基础上 `/map` 发布者 >0 且网关收到地图；
  - `nav`：在 base 基础上收到 `/map` 且节点列表含 `amcl`（随后等待初始位姿）。
- 启动失败（进程提前退出/超时/关键节点缺失）→ `session.state=error`，`error` 带最后 30 行日志摘要；`GET /api/session/logs` 可看完整尾部（最多 500 行）。
- 停止：先停遥控，SIGINT 进程组 → 20 s 超时 SIGTERM → 5 s → SIGKILL；结束后 `mode=idle,state=idle`。
- 外部栈检测：启动时若节点列表含 `raspbot_base`/`amcl`/`slam_toolbox` 等 bringup 节点且非本网关子进程 → `session.external=true`；默认拒绝 start（409）。带 `--attach` 时进入附加模式：`managed=false`，每 2 s 根据图推断 `mode`（amcl+bt_navigator→nav；slam_toolbox→slam；有 `/cmd_vel_safe`→base），所有导航/遥控/看板功能可用，start/stop 返回 409。**部署时应二选一**：systemd `rasbot.service` 或网关托管，避免双 owner。
- 地图保存独立于会话（任何 running 模式均可），写入目录规则见 §3.3.2。

### 5.3 Nav2 初始位姿与激活（`nav2.py` 或 runtime 内）

导航模式启动后 Nav2 节点处于 `unconfigured`（`autostart:=false`）。流程：

1. 等待 `/map` 与 `amcl` active（localization 的 manager 会自动拉起）。
2. 用户 `POST /api/initialpose` → 网关发布 `/initialpose`（2 Hz 重发，最多 20 s）直到 TF `map→base_link` 可查。
3. 激活编排（等价 `.dsh/selftest/activate_nav2.sh`，用 lifecycle 服务而非 CLI）：
   - 若 `/lifecycle_manager_nav2` 存在且非 finalized，best-effort 调用 `shutdown`；
   - 依次 `controller_server`、`planner_server`：未 active 则 `configure`+`activate`，每步超时 15/30 s，失败重试 3 次；
   - 等待 action `/compute_path_to_pose` 被发现（≤10 s，替代脚本里的固定 sleep 10）；
   - `behavior_server` → `bt_navigator`（失败重试，等待 action `/navigate_to_pose` 被发现 ≤15 s）→ `collision_monitor`；
   - 全部就绪且 `/navigate_to_pose` action server `server_is_ready()` → `nav.state=ready`，推送 log/state；任一失败 → `nav.state=error` + 具体节点与原因。
4. `POST /api/nav/activate` 允许手动重试。导航中重新设置初始位姿 → 先取消目标再发布。
5. 动作/反馈/结果字段使用运行时自省（`get_fields_and_field_types()`）读取，兼容 Nav2 版本差异；结果状态码 4=SUCCEEDED / 5=CANCELED / 6=ABORTED。

### 5.4 数据面

- 订阅 QoS 必须正确：`/map` transient_local（depth 1, reliable）；`/scan` sensor_data；`/odom` reliable；`/plan`、`/diagnostics` 默认 reliable。
- 地图版本：`/map` 数据 hash 变化才递增 `version` 并推送 `map` 事件；PNG 按需生成并缓存（同一 version 只生成一次）。
- 扫描点：在 map frame 下发布；若 TF 不可用则用 odom frame 并置 `frame` 字段（前端按 `robot` 位姿兜底换算或隐藏）。降采样到 ≤360 点（5 Hz）。
- 路径：`/plan` 折线降采样 ≤200 点，节流 2 Hz。
- 速率统计：滑动窗口 2 s 计算 `/scan`、`/odom` 频率；新鲜度按消息 stamp/到达时间。
- 诊断：`/diagnostics` 1 Hz 聚合；`level` 取最严重；items 最多 12 条，非 OK 优先。
- 不发布 TF；不修改任何既有话题的发布者。

### 5.5 地图文件

- PNG：8-bit 灰度，行 0 = 世界坐标最大 y（与 OccupancyGrid 行序相反）；映射：`-1→205（未知）`、`0→254（空闲）`、`100→0（占用）`、中间值线性。
- 兜底实现必须只用标准库（zlib/struct），不得依赖 numpy/Pillow/cv2。
- PGM 保存：P5，同上映射，行序为 y 从大到小；YAML 字段按 §3.3.2。
- 地图列表解析 YAML 使用安全的最小解析器（不依赖 PyYAML），支持 `image/resolution/origin/mode/negate/thresholds`；坏文件标 `valid=false` 不得使接口 500。

### 5.6 运行入口

```bash
# 构建（Pi）
cd ~/ros2_ws && colcon build --symlink-install --packages-select raspbot_console
source install/setup.bash
ros2 run raspbot_console console_bridge --port 8080
# 或直接 python -m raspbot_console --port 8080

# 附加到 systemd 已启动的栈
ros2 run raspbot_console console_bridge --attach --port 8080
```

参数：`--host 0.0.0.0 --port 8080 --token "" --workspace ~/ros2_ws --maps-dir /mnt/ssd/maps --maps-fallback ~/ros2_ws/maps --attach --no-session-manage --max-sse 4 --log-level info`。

---

## 6. Web UI 规范（`pc_console/web/`）

### 6.1 布局（目标 1440×900，最小 1280×720）

```
┌──────────────────────────────── Header ────────────────────────────────┐
│ 品牌/模式徽章 | 连接状态 | ROS 健康 | 会话状态 | [急停] | 设置/帮助       │
├──────────┬───────────────────────────────────────┬─────────────────────┤
│ 左栏 260 │ 地图画布（自适应）                     │ 右栏 320            │
│ 会话控制 │  工具条：指针/设目的地/设初始位姿/跟随  │ 机器人遥测          │
│ 地图列表 │  右上：图层开关、缩放、坐标             │ 导航状态/取消/数值输入│
│ 外设     │                                       │ 诊断                │
│          ├───────────── 底部：遥控条（键盘/摇杆/速度）─────────────┤
│          │ 事件日志（可折叠）                                        │
└──────────┴───────────────────────────────────────┴─────────────────────┘
```

- 深色主题（背景 `#0b1220` 系，主色青 `#22d3ee`，警告琥珀，错误红），地图白/灰/黑保持高对比；系统字体栈含 `"Microsoft YaHei"`。
- 组件层级：顶栏、左栏卡片、地图浮层工具条、右栏卡片、底部遥控条与日志；不可依赖外部字体/图标库（内联 SVG）。
- 关键状态一律有中文文案 + 颜色 + 图标三重表达，不能只靠颜色。

### 6.2 交互

- **地图**：滚轮以鼠标为中心缩放；拖拽平移；`F` 跟随机器人；双击/按钮回中；显示网格与比例尺（米）；鼠标坐标实时显示。
- **工具**：
  - `V` 指针（默认，平移/查看）；
  - `G` 设目的地：单击地图 → `POST /api/goal`（需 nav ready；未就绪时按钮禁用并提示原因）；目标标记 + 路径显示；
  - `P` 设初始位姿：按下确定位置、拖拽确定朝向、松开发送；释放前显示箭头预览。
  - `Esc` 取消当前工具/预览。
- **会话**：
  - idle：三张模式卡「手动遥控 / 开始建图 / 开始导航」。导航卡需从地图列表选择（无地图时提示先建图）；点击后确认弹窗，显示「将停止当前进程并启动…」。
  - slam：显示「保存地图」「停止建图」；地图版本更新时画布自动增量刷新（底图重载 + 位姿/激光平滑过渡）。
  - nav：显示阶段（等待初始位姿 → 激活中 → 就绪 → 前往目的地 → 成功/失败/取消），进度、剩余距离、已用时间、恢复次数；提供「取消目标」「重新激活」「设置初始位姿」。
- **遥控**：WASD 前后/横移，Q/E 自转，Shift 加速（不超过 limits）；keyup/失焦/切换标签页立即停；虚拟摇杆支持鼠标/触摸，松手回中并停；速度滑杆显示 m/s；遥控仅在 running 且非急停时可用；导航中首次遥控需确认「将取消当前导航目标」。
- **急停**：顶栏常驻红色按钮；点击立即锁存（无确认），状态变为「已急停」；解除需二次确认弹窗。急停时禁用遥控/目标/会话启动，保留停止会话与解除。
- **诊断**：右栏摘要（OK/WARN/ERROR 计数，最严重 1–3 条消息），点击展开完整列表；ERROR 时顶栏横幅。
- **日志**：底部可折叠，级别过滤、暂停滚动、清空、下载 `.log`；网关 `log` 事件 + 会话日志（按需拉取 `/api/session/logs`）。
- **设置**：Pi 地址/端口/token（`/local/config`）、连接测试（`/local/ping`）、图层默认值、刷新频率跟随（10 Hz 固定）、快捷键说明。
- **连接**：EventSource 断开/`/api/health` 失败 → 顶栏「未连接」、地图/按钮禁用但保留最后画面并加时间戳水印；自动重连（EventSource retry + 指数退避 fetch）。
- **错误**：API 4xx/5xx 转为 toast + 日志条目，文案映射（如 `nav_not_ready` → 「导航尚未就绪：请先设置初始位姿」）。

### 6.3 技术约束

- 原生 ES Modules，无框架、无构建、无 CDN；`type="module"`；支持 Edge/Chrome 最近两年版本。
- 画布使用 `requestAnimationFrame` 分层重绘（底图层 1 Hz 级、动态层 10 Hz）；`devicePixelRatio` 适配；离屏 canvas 缓存底图。
- 纯函数抽到 `js/math.js`（世界↔屏幕、y 翻转、点击→目标、按键→速度、clamp、格式化），可用 `node --test` 覆盖（`web/tests/`）。
- `index.html` 中所有被 JS 引用的元素 id/class 必须存在；提供 DOM id 契约测试（Node 读文件校验）。
- 无 `alert/confirm/prompt`（统一自绘模态框）；无 `eval`；不写 `document.write`。

### 6.4 文案与状态映射

| 状态 | 文案 | 颜色 |
|---|---|---|
| 连接正常 | 已连接 / 延迟 xx ms | 青 |
| 连接失败 | 未连接：无法访问 Pi 网关 | 红 |
| session.mode=idle | 空闲：可选择运行模式 | 灰 |
| session.state=starting | 正在启动（建图/手动/导航）… | 蓝/琥珀 |
| session.state=running | 运行中 | 绿 |
| session.state=stopping | 正在停止… | 琥珀 |
| session.state=error | 启动失败/异常退出 | 红 |
| nav.state=waiting_initialpose | 请在地图上设置初始位姿 | 琥珀 |
| nav.state=activating | 正在激活导航（步骤 x/6）… | 蓝 |
| nav.state=ready | 导航就绪，点击地图设置目的地 | 绿 |
| nav.state=goal_active | 前往目的地：剩余 x.xx m / 已用 xx s | 青 |
| nav.state=succeeded | 已到达目的地 | 绿 |
| nav.state=failed/canceled | 导航失败/已取消（原因） | 红/灰 |
| estop.active | 已急停（点击解除，需确认） | 红 |

---

## 7. 目录与工作包边界

```
pc_console/                       # WP-C2（PC 程序）与 WP-C3（Web UI）
  main.py                         # C2
  console_server/                 # C2：config/http/proxy/demo
  tests/                          # C2：unittest
  web/                            # C3：index.html/css/js/tests
  start.bat / start-demo.bat      # C2
  config.example.json / README.md # C2
  demo_maps/                      # 运行时生成（git 忽略）

ros2_ws/src/raspbot_console/      # WP-C1（Pi 网关）
  package.xml setup.py setup.cfg resource/
  launch/console.launch.py
  config/console.yaml
  raspbot_console/                # 纯逻辑模块不得在导入期依赖 rclpy
  test/                           # pytest/unittest（纯逻辑可离线跑）
  README.md

tools/console/                    # WP-C4（部署与验收）
  deploy.ps1                      # Windows → Pi 部署/构建
  install_service.sh              # 可选 systemd
  acceptance_sil.sh               # Pi 台架 SIL 端到端（mock 栈 + HTTP）
  contract_check.py               # 协议一致性检查（可对 demo/真网关）

docs/
  PC上位机_设计文档_v1.md         # 本文（lead）
  PC上位机_操作手册_v1.md         # 使用手册（WP-C4）
  PC上位机_验收记录_2026-11-24.md # 测试证据（WP-C4）
```

**边界规则**：C1 不修改 `ros2_ws` 其他包；C2/C3 只写 `pc_console/`；C4 只写 `tools/console/`、`docs/`；lead 负责 `README.md` 与最终集成。任何跨边界改动先在任务板说明。

---

## 8. 测试与验收

### 8.1 PC 侧离线（无 Pi，必须全绿）

```bash
# 1) PC 程序单元/集成测试（标准库 unittest）
cd pc_console && python -m unittest discover -s tests -v

# 2) Web 纯函数与 DOM 契约测试（Node ≥18）
node --test pc_console/web/tests

# 3) 演示模式端到端（启动本地服务，脚本走 HTTP/SSE）
python tools/console/contract_check.py --base http://127.0.0.1:8090 --full \
    --nav-map demo_home --initialpose 1.5 1.5 0 --goal 2.5 1.5 0
```

说明：演示模式默认地图 `demo_home`、机器人起点约 `(1.5, 1.5)`，因此全流程检查需显式给
`--nav-map/--initialpose/--goal`；Pi 台架 SIL 用默认值（0,0,0 + 0.8,0,0）即可。`--demo`
参数仅为命令兼容保留。

`contract_check.py --full` 至少覆盖：health/state 必填字段；SSE 收到 `state/scan`；`session/start slam` → 收到地图且 PNG 可解码；保存地图 → `/api/maps` 可见；`session/stop`；`session/start nav` → initialpose → 收到 `nav.state=ready`；goal → 最终 `succeeded`；teleop → 位姿变化；estop → 速度归零/目标取消；释放；错误路径（无图导航 409、急停中遥控 409）。

### 8.2 Pi 台架 SIL（上电后执行，脚本必须可重复）

```bash
# 部署（PC 上，需 Pi 可达）：pwsh -File tools/console/deploy.ps1
# 台架验收（Pi 上）：bash ~/tools/console/acceptance_sil.sh
```

`acceptance_sil.sh` 用 `use_mock:=true` 的 mock 三件套跑完整 HTTP 流程：建图 → 保存地图 → 导航（静态 mock 地图）→ initialpose → Nav2 激活 → 目标 SUCCEEDED，并检查：
急停 0.3 s 内零速；遥控 500 ms 看门狗；SSE 频点；地图 PNG 尺寸/坐标换算；目标取消；无应用层异常退出。

### 8.3 真车验收（需落地场地，人工）

- TC-C1：建图 2 s 内首帧可见；建图过程中遥控可动；保存后地图在导航模式加载成功。
- TC-C2：在图上点目标，车到点偏差 <20 cm（空旷区，依据可行性分析 TC-H2）。
- TC-C3：急停按下 ≤0.3 s 四轮停转；解除后需二次确认。
- TC-C4：关闭 PC 程序/拔网，车按既有看门狗停车（手动模式）或继续导航（自动模式，策略见主设计文档），重连后状态恢复。
- TC-C5：初始位姿设置后 10 s 内 AMCL 收敛、导航按钮可用。

### 8.4 验收证据

- 离线：`pc_console/tests`、`web/tests`、`contract_check.py` 日志。
- 截图：演示模式主界面（建图中/导航中/急停各一张）存入 `docs/evidence/`（由 lead 用无头浏览器生成）。
- Pi：`acceptance_sil.sh` 输出日志存入 `docs/` 验收记录。

---

## 9. 已知限制与风险

1. **无编码器** → 位姿漂移，UI 必须显示定位来源（map/odom）与数据新鲜度。
2. **Nav2 生命周期** 不 autostart；激活依赖初始位姿，冷启动可能耗时 20–60 s，UI 必须给出明确进度而非「卡住」。
3. **地图坐标 y 翻转** 是最高频 bug 源（可行性分析 H-2）：必须由 Node 单测 + 已知点校验覆盖。
4. **PC 与 Pi 时钟不同**：UI 只用 PC 本地时间做超时/延迟判断，不直接比较两端时间戳；显示数据新鲜度用各自接收间隔。
5. **局域网安全**：网关无 TLS/账号体系，仅限可信内网；token 为可选的弱防护。禁止公网端口映射。
6. **不保证**在弱 Wi-Fi 下遥控手感；控制帧 20 Hz、超时 500 ms，丢包即停是设计行为。
7. **演示模式不是真车仿真**：只用于 UI/协议验收，不模拟 Nav2 调参效果、雷达噪声与打滑。

---

## 10. 变更流程

- 接口新增字段：向后兼容，追加并在本文登记；接口语义/字段改名：升 `api` 版本（2）并同步 C1/C2/C3。
- 本契约由 lead 维护；WP-C1/C2/C3 以本文为准，冲突时先停下来问 lead，不自行改协议。
