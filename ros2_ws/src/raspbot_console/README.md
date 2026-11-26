# raspbot_console —— RASPBOT-V2 Pi 端 HTTP/SSE 网关（WP-C1）

Pi 5 上的 ROS 2 Python 包（Ubuntu 26.04 / ROS 2 Lyrical / Python 3.12）。它把
ROS 侧能力包装成 **Wire API v1**（HTTP + SSE），供 PC 上位机（`pc_console/`）
与 Web UI 使用：状态看板、建图/导航会话管理、地图 PNG/保存/列表、遥控、
急停、初始位姿与 Nav2 生命周期激活、外设控制。

契约来源（冻结）：

- `docs/PC上位机_设计文档_v1.md` §1、§3、§5、§8（接口/schema/实现设计）
- `ros2_ws/CONTRACTS.md`（话题/TF 契约，本包**不新增任何 TF 发布者**）

只依赖 Python 标准库 + ROS 依赖（`rclpy`、`tf2_ros`、`nav2_msgs`、
`lifecycle_msgs`、`geometry_msgs`、`sensor_msgs`、`nav_msgs`、
`diagnostic_msgs`、`std_msgs`）。**不依赖** numpy/Pillow/cv2/PyYAML（启动流程
中的 `config/console.yaml` 由 launch 文件用 ROS 环境自带的 PyYAML 读取）。

---

## 1. 架构与线程模型

```
┌────────────────────────────────────────────────────────────────┐
│ console_bridge（ros2 run raspbot_console console_bridge）       │
│                                                                │
│  HTTP 线程池（ThreadingHTTPServer, :8080）                      │
│      │  /api/*（JSON）+ /api/events（SSE chunked）              │
│      ▼                                                          │
│  controller.py   命令编排 / 前置条件 / 错误码 / nav 状态机       │
│    ├─ state.py       线程安全状态仓 + 事件总线 + 速率/诊断聚合   │
│    ├─ session.py     会话状态机 + `ros2 launch` 子进程（进程组） │
│    └─ ros_runtime.py 唯一 rclpy 层：订阅/TF/发布/action/lifecycle│
│                                                                │
│  监控线程：10 Hz 推送 state 事件；0.5 s 推进会话状态机           │
│  ROS 线程：MultiThreadedExecutor(3) 执行全部 ROS 回调            │
└────────────────────────────────────────────────────────────────┘
```

导入期依赖约束：`config/protocol/mapio/state/session/controller/http_api`
**不导入 rclpy**，可在无 ROS 的机器上离线单测；`ros_runtime` 是唯一直接
依赖 rclpy/tf2/消息包的模块，`bridge` 负责组装与优雅退出。

### 模块职责

| 模块 | 职责 |
|---|---|
| `config.py` | 参数默认值、argparse、可选 JSON 配置（CLI > JSON > 默认） |
| `protocol.py` | 错误模型、字段校验、compact JSON、SSE 帧、yaw/四元数 |
| `mapio.py` | OccupancyGrid→PNG（纯 zlib/struct）、P5 PGM+YAML 保存、PGM→PNG、地图列表、名称校验 |
| `state.py` | 线程安全状态仓、§3.1 快照、SSE 事件总线、2 s 速率窗口、诊断聚合 |
| `session.py` | 会话状态机（idle/starting/running/stopping/error）、`ros2 launch` 命令构造、SIGINT→SIGTERM→SIGKILL 进程组停止、stdout 500 行环形缓冲、attach/外部栈 |
| `controller.py` | HTTP 命令 → 状态/会话/runtime 编排；显式 `(status, payload)` 接口 |
| `http_api.py` | ThreadingHTTPServer、路由、token/CORS、SSE（HTTP/1.1 chunked） |
| `ros_runtime.py` | 订阅（QoS）、TF 查询、遥控 20 Hz+看门狗、急停重发、initialpose 2 Hz 重发、Nav2 action 客户端 + lifecycle 激活编排、事件推送 |
| `bridge.py` | 入口：参数、rclpy init、启动线程、信号处理、退出收尾 |

### ROS 数据面（与 CONTRACTS v1.2 一致）

- **订阅**：`/map`（transient_local, depth 1, reliable）、`/scan`
  （sensor_data）、`/odom`（reliable）、`/plan`、`/diagnostics`。
- **发布**：`/cmd_vel_teleop`（20 Hz，reliable；经 `twist_mux` 优先级 50 汇入
  `/cmd_vel`，**绝不直接发 `/cmd_vel`/`/cmd_vel_safe`**）、`/estop`（Bool,
  reliable depth 1）、`/initialpose`（PoseWithCovarianceStamped）、`/buzzer`、
  `/led`、`/servo_pan`、`/servo_tilt`。
- **TF**：只读。用 `tf2_ros.Buffer + TransformListener` 查 `map→base_link`
  （10 Hz），无 map 时退化 `odom→base_link`，`state.robot.frame` 反映来源；
  **不发布任何 TF**。
- **事件**：`state` 10 Hz、`scan` 5 Hz（≤360 点，map/odom 坐标）、`path`
  2 Hz 节流（≤200 点）、`map`（内容 hash 变化时版本 +1）、`session`（状态
  变化立即）、`log`（即时）。

### 地图文件约定（§5.5）

- PNG：8-bit 灰度，`-1→205（未知）/0→254（空闲）/100→0（占用）`，中间值线性；
  **行 0 = 世界坐标最大 y**（与 OccupancyGrid 行序相反）。
- PGM：P5，映射相同、行序相同（y 由大到小）；YAML 字段
  `image/mode/resolution/origin/negate/occupied_thresh/free_thresh`。
- 保存目录：`/mnt/ssd/maps`（存在且可写）否则 `~/ros2_ws/maps`。
- 地图列表对坏文件返回 `valid=false`，不会 500。

---

## 2. 构建（Pi）

```bash
# 1) 源码就位（示例）
rsync -av raspbot_console/ ubuntu@192.168.50.98:~/ros2_ws/src/raspbot_console/

# 2) 构建（第 9 个包）
ssh ubuntu@192.168.50.98
cd ~/ros2_ws
colcon build --symlink-install --packages-select raspbot_console
source install/setup.bash
```

依赖已在 `package.xml` 声明（`rclpy/tf2_ros/nav2_msgs/lifecycle_msgs/
geometry_msgs/sensor_msgs/nav_msgs/diagnostic_msgs/std_msgs` 等），
`rosdep install --from-paths src --ignore-src -r -y` 可补齐。

---

## 3. 运行

```bash
# 方式 A：ros2 run（网关托管 bringup，默认模式）
ros2 run raspbot_console console_bridge --port 8080

# 方式 B：直接模块入口（等价）
python3 -m raspbot_console --port 8080

# 方式 C：launch（参数读 config/console.yaml，可逐项覆盖）
ros2 launch raspbot_console console.launch.py port:=8080 token:=mytoken

# 附加到 systemd 已启动的 ROS 栈（网关不管理 bringup）
ros2 run raspbot_console console_bridge --attach --port 8080
```

启动后：`http://<pi>:8080/api/health`。默认监听 `0.0.0.0:8080`，
**只应暴露在可信内网**（无 TLS；token 仅弱防护）。

### 3.1 参数（CLI；括号内为默认值）

| 参数 | 说明 |
|---|---|
| `--host`（`0.0.0.0`） | 监听地址 |
| `--port`（`8080`） | 监听端口 |
| `--token`（空） | 非空时要求 `X-Console-Token`；GET 流式接口可用 `?token=` |
| `--max-sse`（`4`） | SSE 客户端上限，超出 503 |
| `--workspace`（`~/ros2_ws`） | 会话启动时 source 的工作区 |
| `--ros-setup`（`/opt/ros/lyrical/setup.bash`） | ROS 环境脚本 |
| `--ros-log-dir DIR`（空=继承） | source 后为 bringup 子进程 `export ROS_LOG_DIR`；SSD 会掉盘的台架建议 `/tmp/console_roslog`，避免日志 I/O 阻塞节点线程 |
| `--maps-dir`（`/mnt/ssd/maps`） | 地图主目录 |
| `--maps-fallback`（`~/ros2_ws/maps`） | 地图回退目录 |
| `--attach` | 附加外部 ROS 栈：`managed=false`，start/stop 返回 409 |
| `--no-session-manage` | 不管理会话（等价只读模式） |
| `--max-linear/--max-lateral/--max-angular`（0.30/0.30/1.5） | 遥控限幅（`state.limits`） |
| `--session-timeout`（120） | 会话就绪超时 s |
| `--log-level`（info） | debug/info/warning/error |
| `--config <json>` | 可选 JSON 配置（键名同默认值，CLI 优先） |

`config/console.yaml` 是上述默认值的可读参考，`console.launch.py` 会读取它
作为 launch 参数默认值；直接 `ros2 run` 时以 `config.py` 的 `DEFAULTS` 为准。
`ros2 launch` 追加的 `--ros-args ...` 会被忽略（本节点不使用 ROS 参数/remap）。

### 3.2 会话启动命令（网关托管时）

```bash
bash -lc 'source /opt/ros/lyrical/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && \
  exec ros2 launch raspbot_bringup bringup.launch.py mode:=<none|slam|nav> [map:=<abs.yaml>] [use_mock:=true]'
```

模式映射：`base→mode:=none`、`slam→mode:=slam`、`nav→mode:=nav map:=<yaml>`；
`use_camera/use_perception` 可选。子进程 `start_new_session=True`，停止时对
**进程组** 依次 SIGINT（20 s）→ SIGTERM（5 s）→ SIGKILL。

就绪判据（每 0.5 s 轮询，超时 120 s）：`/odom` 与 `/cmd_vel_safe` 有发布者；
`/map` 有发布者且网关已收到地图；`nav` 额外要求 `amcl` 节点存在。

### 3.3 两种部署方式（二选一，避免双 owner）

| 方式 | bringup 由谁拉起 | 网关启动参数 | 说明 |
|---|---|---|---|
| A. systemd 托管栈 | `raspbot.service`（外部） | `--attach` | 开机即用；网关只读+控制，start/stop 409 |
| B. 网关托管 | 网关自己（`ros2 launch` 子进程） | 默认（不传 attach） | PC 上一键开始/停止建图/导航 |

**不要同时启用两种方式**：网关会检测到外部 bringup 节点并拒绝再次拉起
（`409 external_stack`），这就是「一个 ROS 图一个 owner」的保护。

方式 A 的 systemd unit 示例（`/etc/systemd/system/raspbot-console.service`）：

```ini
[Unit]
Description=RASPBOT-V2 Pi HTTP/SSE console gateway
After=network-online.target raspbot.service
Wants=network-online.target
Requires=raspbot.service

[Service]
Type=simple
User=ubuntu
Environment=HOME=/home/ubuntu
ExecStart=/bin/bash -lc 'source /opt/ros/lyrical/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && exec ros2 run raspbot_console console_bridge --attach --port 8080'
Restart=on-failure
RestartSec=3
# 退出时网关会先停遥控（零速）、再关闭 HTTP；托管模式下还会停止子进程
KillSignal=SIGINT
TimeoutStopSec=40

[Install]
WantedBy=multi-user.target
```

方式 B 更适合调试：把上面的 `--attach` 去掉即可（unit 的 `Requires=raspbot.service`
也删除，避免 systemd 先起 bringup 导致外部栈冲突）。

---

## 4. API 摘要（Wire API v1）

Base URL `http://<pi>:8080`，全部接口以 `/api/` 开头；时间戳为 float Unix 秒，
角度为弧度，长度为米；JSON 使用 compact separators。错误统一为
`{"error":{"code","message","detail"}}`，状态码：400/401/404/405/409/503/500。

| 方法/路径 | 说明 |
|---|---|
| `GET /api/health` | 存活探针（免 token） |
| `GET /api/state` | §3.1 完整状态快照 |
| `GET /api/events` | SSE：`state/scan/path/map/session/log` |
| `GET /api/map/meta` | 地图元数据 |
| `GET /api/map.png?v=N` | 地图 PNG（无图 404；按 version 缓存） |
| `GET /api/maps` | 已保存地图列表 + 扫描目录 |
| `GET /api/maps/<name>/preview.png` | PGM→PNG 缩略图 |
| `POST /api/map/save` | 保存当前 `/map` 为 PGM+YAML |
| `POST /api/session/start` | `{"mode":"base|slam|nav","map":null,...}` → 202 |
| `POST /api/session/stop` | 运行中 202；空闲 200 |
| `POST /api/teleop` / `POST /api/teleop/stop` | 遥控 / 立即零速退出 |
| `POST /api/estop` | `{"active":true/false}` 锁存/解除 |
| `POST /api/goal` / `POST /api/goal/cancel` | Nav2 目的地 / 取消 |
| `POST /api/initialpose` / `POST /api/nav/activate` | 初始位姿 / 重试激活 |
| `POST /api/actuator/buzzer|led|servo` | 蜂鸣器 / RGB / 云台 |
| `GET /api/logs` `GET /api/session/logs` `GET /api/ros` | 日志与 ROS 图调试 |

SSE 细节：`Content-Type: text/event-stream`、`X-Accel-Buffering: no`、
**HTTP/1.1 + `Transfer-Encoding: chunked`、逐帧 flush、无 Content-Length**；
首帧 `retry: 2000`，空闲 10 s 发 `: ping`；每客户端独立队列（200，丢最旧），
最多 4 客户端（超出 503）。

### 关键安全行为

- **遥控**：20 Hz 发布 `/cmd_vel_teleop`；500 ms 无新帧 → 自动零速并
  `teleop.active=false`；`seq` 单调递增，过期帧丢弃；导航中遥控会先取消目标。
- **急停**：锁存 `/estop=true` 并每秒重发，取消导航目标、停遥控、立即零速；
  急停期间会话启动/导航目标/遥控均 409；显式 `false` 才解除。启动后若本网关
  尚未发过急停命令，会用 `/diagnostics` 中 `Raspbot Safety: E-Stop` 的
  `estop=True/False` 同步真实状态。
- **退出/会话停止**：统一停遥控 + 零速；托管会话被 SIGINT→SIGTERM→SIGKILL
  停止；不阻塞 >30 s。
- 网关不打开 I2C/串口，不发布 TF，不直接写 `/cmd_vel`，崩溃不影响底盘
  （twist_mux/limiter/base 仍在链路上）。

---

## 5. 离线测试（本机 Windows/Linux 均可跑，无需 ROS）

```bash
cd ros2_ws/src/raspbot_console

# 1) 语法编译
python -m compileall -q .

# 2) 纯逻辑单元测试（不依赖 rclpy）
python -m unittest discover -s test -v

# 3) 导入期隔离检查（ros_runtime/bridge 之外不得 import rclpy）
python -c "import raspbot_console.mapio, raspbot_console.state, raspbot_console.protocol, raspbot_console.session, raspbot_console.controller, raspbot_console.http_api"
```

测试覆盖：`test_mapio.py`（灰度映射/PNG/PGM 行序/YAML/地图列表/保存冲突）、
`test_state.py`（§3.1 schema/版本/事件总线/遥测看门狗/诊断聚合/速率）、
`test_protocol.py`（错误模型/字段校验/JSON/SSE 帧）、`test_session.py`
（启动命令/状态机/超时/异常退出/SIGINT→SIGTERM→SIGKILL/attach）、
`test_http_api.py`（真实 HTTP 服务：health/state/SSE chunked/teleop 校验/
estop/session 409/map 404/PNG/保存/导航流程/token/CORS/405/请求体上限）。
`test_http_api.py` 用假 runtime 注入真实 controller，**不需要 ROS**。

---

## 6. 离线未验证项与真机联调状态（2026-11-26 更新）

> **2026-11-26 已在真机（Pi 5 / 192.168.50.100）完成**：部署+colcon 构建、
> 118 项纯逻辑单测、台架 SIL `acceptance_sil.sh` **PASS=44/FAIL=0**（建图→保存→
> 导航 initialpose→激活 6/6→目标 SUCCEEDED→取消→急停）、真硬件 `mode=base` 只读
> 遥测（12 节点、/scan≈10 Hz、/odom≈30 Hz、诊断新鲜）、PC 端代理+Web UI 联调
> （截图 5/5、JS 错误 0）。**仍未验证**：真车落地运动/建图/导航到达精度、真车急停
> 机械时延、带 token 的浏览器 `<img>` 场景、systemd 方式 A/B 的实际部署。
> 详细证据：`docs/PC上位机_验收记录_2026-11-24.md` §9、`docs/evidence/`。

以下为最初（Pi 离线时）列出的未验证项，保留供后续复查：

以下按“代码按 rclpy/ROS 文档与仓库既有事实编写，但只能在 Pi 上实测确认”列出：

1. `rclpy` 具体 API 行为：`ActionClient.server_is_ready()/send_goal_async`、
   `tf2_ros.Buffer.lookup_transform`、lifecycle 服务
   `/{node}/get_state`、`/{node}/change_state` 的参数/返回结构。
2. QoS 匹配：`/map` transient_local + `/scan` sensor_data 与既有发布者能否握手、
   地图首帧是否到达。
3. 会话启动命令在 Pi 上的实际行为（`source`、`ros2 launch` 参数、
   `start_new_session=True` 进程组停止）；就绪判据在真 bringup 下的时序。
4. Nav2 生命周期激活编排的实际顺序/时序（manager shutdown、5 节点
   configure/activate、`/compute_path_to_pose` 与 `/navigate_to_pose` 发现窗口、
   重试与 15/30 s 超时）。
5. `NavigateToPose` 的 feedback/result 字段在 Lyrical/Nav2 1.5.2 下的真实名称
   （代码用 `get_fields_and_field_types()` 自省，但字段存在性未实测）。
6. `/plan`、`/diagnostics`、`/scan` 的 frame_id/时间戳/频率在真实栈下的表现；
   `map→base_link` TF 出现时机是否与初始位姿重发匹配。
7. 急停诊断同步：`Raspbot Safety: E-Stop` 的 `estop` KeyValue 值是否为
   `True/False` 文本（已按 safety_node.cpp 实现解析）。
8. 遥控链路：`/cmd_vel_teleop → twist_mux → /cmd_vel → /cmd_vel_safe` 的
   20 Hz/500 ms 看门狗实测；急停 0.3 s 内四轮停转。
9. 地图：真实 SLAM 地图尺寸/分辨率下的 PNG 渲染与 `/mnt/ssd/maps` 可写性；
   `map_saver` 兼容性（本包自带保存，不依赖 nav2_map_server）。
10. SSE 在 PC 反向代理/浏览器 EventSource 下的 chunked 增量解析、断线重连、
    以及 `X-Accel-Buffering: no` 的实际效果。
11. `colcon build`/`ros2 run`/`ros2 launch` 与 systemd unit（本机无 ROS）。
12. 图片内容在 Web UI 的坐标换算（y 翻转）——本包已用已知栅格单测覆盖行序，
    但仍需与 UI 联调确认。

### 上 Pi 后的验证步骤（建议顺序）

```bash
# 0) 构建并启动网关
cd ~/ros2_ws && colcon build --symlink-install --packages-select raspbot_console
source install/setup.bash
ros2 run raspbot_console console_bridge --port 8080 &

# 1) health/state/SSE
curl -s localhost:8080/api/health
curl -s localhost:8080/api/state | python3 -m json.tool | head -40
curl -N --max-time 3 localhost:8080/api/events        # 应看到 retry + state 帧

# 2) 手动模式会话 + 遥控 + 急停
curl -s -XPOST localhost:8080/api/session/start -d '{"mode":"base"}'
#   等待 GET /api/state 中 session.state=running（约 10-30 s）
ros2 topic hz /cmd_vel_teleop                          # 遥控时应 ~20 Hz
curl -s -XPOST localhost:8080/api/teleop -d '{"vx":0.1,"vy":0,"wz":0,"seq":1}'
#   松手/停止：确认 500 ms 内 /cmd_vel_safe 归零
ros2 topic echo /cmd_vel_safe --once
curl -s -XPOST localhost:8080/api/estop -d '{"active":true}'
#   确认 0.3 s 内 /cmd_vel_safe≈0；再解除
curl -s -XPOST localhost:8080/api/estop -d '{"active":false}'
curl -s -XPOST localhost:8080/api/session/stop

# 3) 建图 + 保存
curl -s -XPOST localhost:8080/api/session/start -d '{"mode":"slam"}'
#   等 nav/map：GET /api/map/meta 有 available=true，GET /api/map.png 可解码
curl -s -XPOST localhost:8080/api/map/save -d '{"name":"room","overwrite":false}'
ls -l /mnt/ssd/maps/room.pgm /mnt/ssd/maps/room.yaml   # 或 ~/ros2_ws/maps/
curl -s localhost:8080/api/maps | python3 -m json.tool
curl -s -o /tmp/p.png localhost:8080/api/maps/room/preview.png && file /tmp/p.png
curl -s -XPOST localhost:8080/api/session/stop

# 4) 导航闭环（Nav2 不 autostart，必须先 initialpose）
curl -s -XPOST localhost:8080/api/session/start \
  -d '{"mode":"nav","map":"/mnt/ssd/maps/room.yaml"}'
curl -s -XPOST localhost:8080/api/initialpose -d '{"x":0.0,"y":0.0,"yaw":0.0}'
#   轮询 GET /api/state：nav.state waiting_initialpose → activating → ready
curl -s -XPOST localhost:8080/api/goal -d '{"x":1.5,"y":0.0,"yaw":null}'
#   观察 nav.distance_remaining/navigation_time_s，最终 succeeded
curl -s -XPOST localhost:8080/api/goal/cancel
curl -s -XPOST localhost:8080/api/session/stop

# 5) --attach 模式（systemd 已拉起 bringup 时）
sudo systemctl start raspbot.service
ros2 run raspbot_console console_bridge --attach --port 8080 &
curl -s localhost:8080/api/state      # session.managed=false, external=true
curl -s -XPOST localhost:8080/api/session/stop   # 应 409 attach_mode

# 6) 优雅退出：Ctrl-C/`kill -INT` 网关进程
#   确认零速下发、无遗留 ros2 launch 子进程（ps -ef | grep bringup）
```

---

## 7. 已知限制与实现假设

1. `state.robot.imu.yaw_deg` 使用 `/odom`（EKF，IMU 修正后）姿态的 yaw 作为
   代理，因为 §5.4 的订阅清单不含 `/imu/data`；若要真实 IMU yaw 可后续追加
   订阅（不改契约字段）。
2. SSE 采用 **HTTP/1.1 + chunked**（lead 的协议补充；设计文档 §3.2 原文写
   `Connection: close`，以补充为准）。
3. `nav.state` 进入终态（succeeded/failed/canceled）后，允许直接下发新目标，
   并在 3 s 后自动回到 `ready`（契约 goal 前置为 ready；这是为连续操作做的
   兼容处理）。
4. `POST /api/goal` 的 `yaw=null` 且机器人位姿不可用时回退 `0.0` 并写 warn 日志
   （契约要求用 robot.yaw，但无位姿时需兜底）。
5. 扫描点坐标优先 map，其次 odom；两者 TF 都不可用时以雷达自身 frame 发布
   （契约只写到 odom 兜底，此处为防御性扩展，前端可按 frame 隐藏）。
6. `session.error` 截断到 8000 字符（含最后 30 行 bringup 日志摘要）；完整尾部
   通过 `GET /api/session/logs` 获取。
7. token 的 `?token=` 形式对所有 GET 开放（契约仅要求 GET 流式接口可用，
   这里方便 `<img>` 直接加载 `map.png`）。
8. `--attach`/`--no-session-manage` 下 start/stop 返回 409（错误码
   `attach_mode`）；检测到外部栈拒绝 start 为 409 `external_stack`。
9. 生命周期激活进度 `activation.total=6` = `lifecycle_manager_nav2` + 5 个 Nav2
   节点；等待 action server 的子步骤不单独计步。
10. `/api/teleop` 要求 `vx/vy/wz` 三个字段齐全（契约示例即三字段），`seq`
    可选；越界值 clamp、非数值 400。
11. 同一模式重复 `session/start` 也执行「停止→启动」切换（最简单且幂等）。
12. 地图保存要求会话处于 running（base/slam/nav 任一；§5.2），idle → 409
    `session_idle`。

---

## 8. 目录结构

```
raspbot_console/
  package.xml setup.py setup.cfg resource/raspbot_console
  launch/console.launch.py       # 可选 launch（读 config/console.yaml）
  config/console.yaml            # 默认参数参考（与 config.py DEFAULTS 一致）
  raspbot_console/
    __init__.py __main__.py config.py protocol.py mapio.py state.py
    session.py controller.py http_api.py ros_runtime.py bridge.py
  test/
    support.py test_protocol.py test_mapio.py test_state.py
    test_session.py test_http_api.py
  README.md
```
