# tools/console —— PC 上位机部署、契约检查与台架验收（WP-C4）

本目录是 RASPBOT-V2 PC 上位机（`pc_console/` + `ros2_ws/src/raspbot_console/`）的
**部署与验收工具**，不属于运行时。所有脚本以设计文档 `docs/PC上位机_设计文档_v1.md`
（Wire API v1，冻结）为准。

| 文件 | 运行位置 | 用途 |
|---|---|---|
| `deploy.ps1` | Windows PC | 打包 `ros2_ws/src/raspbot_console` + `tools/console`，scp 到 Pi，`colcon build`，跑纯逻辑单测并自检 `ros2 pkg` |
| `install_service.sh` | Pi（sudo） | 可选：安装 `raspbot-console.service`（网关开机自启、崩溃重启），含 `--attach` 模式 |
| `acceptance_sil.sh` | Pi | 台架 SIL 端到端验收：mock 建图→保存地图→导航激活→到点→取消→急停/看门狗全流程 HTTP/SSE |
| `contract_check.py` | PC 或 Pi | 协议一致性检查：`GET /api/*` + SSE schema（只读）与 `--full` 业务全流程；可对演示后端或真网关 |
| `capture_evidence.py` | Windows PC | 无头 Edge/Chrome 截图，生成 `docs/evidence/pc_console_0*.png` 与报告（lead 集成时使用） |

---

## 1. 前置条件

### PC（Windows 10/11）
- Python 3.10+（仓库实测 3.14），**仅标准库**；Node ≥18 仅用于 Web 测试，不是运行依赖。
- 能 ssh/scp 到 Pi（`deploy.ps1` 用系统自带 `ssh.exe`/`scp.exe`/`tar.exe`，Windows 10+ 自带）。
- PC 与 Pi 同一网段，Pi 网关端口（默认 8080）可达。

### Pi（Ubuntu 26.04 / ROS 2 Lyrical）
- `~/ros2_ws` 已存在且已构建过（`install/setup.bash` 可 source）。
- 包依赖已安装（`rclpy`、`raspbot_console` 依赖的 ROS 消息包；见 `package.xml`）。
- 台架 SIL 时**不要**让真硬件栈同时运行（见 §3 双 owner）。

---

## 2. deploy.ps1 —— PC → Pi 部署

```powershell
# 仓库根目录执行；默认目标 ubuntu@192.168.50.98
pwsh -File tools\console\deploy.ps1
pwsh -File tools\console\deploy.ps1 -PiHost ubuntu@192.168.50.98 -NoBuild
pwsh -File tools\console\deploy.ps1 -SkipTests
pwsh -File tools\console\deploy.ps1 -IdentityFile ~\.ssh\id_ed25519
# 密码登录（无密钥）：用系统 askpass 包装，例如
pwsh -File tools\console\deploy.ps1 -Askpass E:\code\raspberrypi\.dsh\askpass.cmd
```

参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `-PiHost` | `ubuntu@192.168.50.98` | ssh 目标（user@host） |
| `-NoBuild` | 关 | 只同步不 `colcon build` |
| `-SkipTests` | 关 | 跳过 Pi 上纯逻辑 unittest |
| `-Askpass` | 空 | 密码方式登录时设置 `SSH_ASKPASS` 包装脚本 |
| `-IdentityFile` | 空 | 指定私钥 |
| `-TimeoutSec` | 300 | ssh/scp 超时 |

行为要点：
1. 只打包 `ros2_ws/src/raspbot_console` 和 `tools/console`（排除 `__pycache__`/`*.pyc`），
   **不碰** `ros2_ws` 其它包；解包到 `~/`（`~/ros2_ws/src/...`、`~/tools/console/...`）。
2. 远端 `source /opt/ros/lyrical/setup.bash` 后 `colcon build --symlink-install --packages-select raspbot_console`，
   构建日志 `/tmp/console_build.log`，失败非零退出。
3. 构建后 `ros2 pkg prefix/executables raspbot_console` 自检，最终打印 `DEPLOY_OK`。
4. 失败时打印阶段原因并返回非零；不会自动修改 `raspbot.service`。

部署完成后（Pi 上二选一，见 §3）：

```bash
# 方式 B：网关托管 bringup（调试推荐）
source /opt/ros/lyrical/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run raspbot_console console_bridge --port 8080
# 或开机自启：bash ~/tools/console/install_service.sh --start

# 方式 A：附加到 systemd 已启动的真硬件栈
ros2 run raspbot_console console_bridge --attach --port 8080
```

---

## 3. install_service.sh —— 可选 systemd 服务

```bash
bash ~/tools/console/install_service.sh                  # 安装并 enable，不立即启动
bash ~/tools/console/install_service.sh --start          # 安装并立即启动
bash ~/tools/console/install_service.sh --attach --start # 附加模式（外部已有 bringup）
bash ~/tools/console/install_service.sh --port 8080 --token mytoken
bash ~/tools/console/install_service.sh --uninstall
```

- 服务名 `raspbot-console.service`，`Restart=always`、`RestartSec=3`、`TimeoutStopSec=30`。
- 写入 `/etc/systemd/system/raspbot-console.service`（权限 600），`enable` 开机自启；
  `ExecStart` 内 source ROS 与工作区后 `ros2 run raspbot_console console_bridge ...`。
- `--force`：跳过「`raspbot.service` 正在运行」检查（不建议）。

**与 `raspbot.service` 的关系（双 owner 是最高频事故）**：

| 方式 | bringup 由谁拉起 | 网关参数 | 网关可否启停会话 |
|---|---|---|---|
| A. `raspbot.service` 托管真硬件栈 | systemd | `--attach` | 否（start/stop 返回 409 external_stack） |
| B. 网关托管 | 网关按会话 `ros2 launch` 子进程 | 默认（无 `--attach`） | 是（PC 上一键建图/导航） |

必须二选一：用方式 B 时先 `sudo systemctl disable --now raspbot.service`；
用方式 A 时 `install_service.sh --attach`，并删除 unit 里对 `raspbot.service` 的 `Requires`。
两种方式同时开启时，网关会检测到外部 bringup 节点并拒绝启动会话（保护机制，不是 bug）。

---

## 4. contract_check.py —— 协议一致性检查

```bash
# 只读检查（默认 http://127.0.0.1:8090；PC 演示/真车代理或直连 Pi 均可）
python tools/console/contract_check.py --base http://127.0.0.1:8090

# 业务全流程（会启停会话、保存地图、发目标；演示模式需显式给地图/起点/目标）
python tools/console/contract_check.py --base http://127.0.0.1:18099 --full \
    --nav-map demo_home --initialpose 1.5 1.5 0 --goal 2.5 1.5 0
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--base` | `http://127.0.0.1:8090` | 被测服务地址 |
| `--token` | 空 | 网关启用 token 时填（`X-Console-Token`） |
| `--full` | 关 | 跑完整业务流（建图→保存→停止→导航→initialpose→激活→目标→遥控/急停/错误路径） |
| `--nav-map` | 空 | 导航用地图名或 yaml 路径；Pi 台架默认用地图列表第一张，演示模式需 `demo_home` |
| `--initialpose X Y YAW` | 0 0 0 | 初始位姿（弧度） |
| `--goal X Y YAW` | 0.8 0 0 | 导航目标 |
| `--nav-timeout` | 180 | 导航/激活等待上限（秒） |
| `--timeout` | 10 | 单请求超时 |
| `--out` | 空 | 输出 JSON 报告路径 |

退出码：`0` 全部 PASS（允许 SKIP/WARN）；`1` 有 FAIL；`2` 参数/连接错误。
`--full` 预期（仓库实测，演示后端）：**33 PASS / 0 FAIL / 1 SKIP**。
检查项覆盖：health/state 必填字段、SSE `state/scan`、地图 PNG 可解码、遥控看门狗、
急停锁存（遥控/目标 409）、地图保存与重名 409、导航 `waiting_initialpose→ready→succeeded`、
idle 遥控/目标 409、nav 缺 map 400 等。

> `--demo` 参数仅作历史兼容保留；演示后端直接给 `--base` 即可。

---

## 5. acceptance_sil.sh —— Pi 台架 SIL 端到端验收

```bash
# Pi 上（需已 deploy + colcon build；raspbot.service 必须已停）
bash ~/tools/console/acceptance_sil.sh
bash ~/tools/console/acceptance_sil.sh --port 18099 --nav-timeout 240
TOKEN=mytoken bash ~/tools/console/acceptance_sil.sh
```

参数/环境变量：`--port`（默认 18099，临时端口，不占 8080/8090）、`--token`、
`--slam-timeout`（默认 120）、`--nav-timeout`（默认 240）、`--keep`（保留网关排障）。

流程（全部走 HTTP/SSE，与 PC 端 UI 同一契约）：
1. 前置检查：ROS 环境、`raspbot_console` 可执行、`raspbot.service` 未运行、临时端口空闲；
2. 起网关 → `/api/health`、`/api/state` schema、SSE `state` 频点（10 Hz 量级）；
3. 错误路径：idle 遥控/目标 409、nav 缺 map 400、未知端点 404 JSON；
4. 建图：`POST /api/session/start {"mode":"slam","use_mock":true}` → 网关拉起 mock bringup
   → 地图出现、`/api/map.png` 可解码且尺寸与 meta 一致；
5. 遥控 + 看门狗：位姿变化、`/cmd_vel_safe` 采样、0.5 s 自动零速；
6. 保存地图：PGM+YAML 成对落盘、PGM 尺寸/YAML resolution 与 meta 一致、`/api/maps` 可见、重名 409；
7. 导航：`{"mode":"nav","map":"<保存的yaml>","use_mock":true}` → `waiting_initialpose`
   → `initialpose` → Nav2 激活 `ready` → `goal` **SUCCEEDED**；
8. 重新设初始位姿 → 再发目标 → `goal/cancel` 取消生效；
9. 急停锁存（遥控/目标 409、`/cmd_vel_safe` 尽量实测）→ 解除；
10. 停止会话回 idle、会话日志无 Python Traceback、网关存活；
11. 清理：只停本脚本的网关 PID 与 cmdline 含 `use_mock:=true` 的 mock 栈，
    **不会** pkill 其它 ROS 栈 / 真车驱动。

退出码：`0` 全 PASS（允许 WARN/SKIP）；`1` 有 FAIL；`2` 前置条件不满足。
产物：网关日志 `/tmp/console_sil_gateway.log`；失败时脚本打印具体检查项与网关日志尾部。
未在台架自动判定的项（真车 TC-C1..C5、急停机械时延、雷达盲区、UI 截图）在脚本末尾列出。

---

## 6. capture_evidence.py —— 截图（lead 集成用）

```powershell
python tools/console/capture_evidence.py --base http://127.0.0.1:8099 --out docs/evidence
```

用本机 Edge/Chrome 无头模式依次截取空闲/建图/导航就绪/导航中/急停 5 张图 + `capture_screens_report.json`。
依赖浏览器与 UI 状态（脚本通过 API 驱动演示后端）；在 CI/无浏览器环境会失败，属预期。

---

## 7. 典型工作流

```text
PC:  pwsh -File tools\console\deploy.ps1          # 部署到 Pi（含单测）
Pi:  bash ~/tools/console/install_service.sh --start   # 或手动 ros2 run（二选一）
Pi:  bash ~/tools/console/acceptance_sil.sh       # 台架 SIL 全流程
PC:  python tools/console/contract_check.py --base http://<pi>:8080 --full  # 真网关契约
PC:  pc_console\start.bat                         # 连真车使用
```

---

## 8. 2026-11-26 真机联调结果与本目录脚本状态

在 Pi 5（`ubuntu@192.168.50.100`，Ubuntu 26.04 / ROS 2 Lyrical）上实测：

| 脚本 | 状态 | 结果 |
|---|---|---|
| `deploy.ps1` 等价流程 | ✅ 已执行 | `raspbot_console` + `raspbot_mock` colcon 构建成功；118 项纯逻辑单测 OK |
| `acceptance_sil.sh` | ✅ 已执行 | **PASS=44 / FAIL=0 / WARN=0 / SKIP=1**（建图→保存→导航激活 6/6→目标 SUCCEEDED→取消→急停），退出码 0；日志归档 `docs/evidence/acceptance_sil_2026-11-26.log` |
| `contract_check.py` | ✅ 已执行 | 直连 Pi 网关 8 PASS/0 FAIL/1 SKIP；经 PC 代理同样通过（`docs/evidence/contract_check_pc_proxy_2026-11-26.txt`） |
| `capture_screens.js` | ✅ 已执行 | 经 PC 代理连接真网关 + SIL 会话：5/5 截图、JS 错误 0（`--use-mock` 已支持，见 `--help`） |
| `install_service.sh` | ✅ 已执行（2026-11-27） | 安装 `raspbot-console.service` 并 `enable --now`；服务 `enabled+active`，开机自启、崩溃 3 s 重启；默认 `--maps-dir ~/ros2_ws/maps --ros-log-dir /tmp/console_roslog`（避开 USB SSD），已实测 `systemctl restart` 后 PC 代理自动恢复 |

台架要点（脚本已内置）：

- 地图保存到 `~/ros2_ws/maps`、ROS 日志到 `/tmp/console_sil_roslog`，**不用 USB SSD**
  （本机 `/mnt/ssd` 有 UAS 掉盘史，日志 I/O 会拖住节点线程并放大 Nav2/DDS 激活超时）。
- 网关激活 Nav2 前会提前 configure behavior/bt 并 **prewarm 20 s**；SIL 下可稳定 6/6。
- 停会话后外部栈检测有 10 s 宽限；紧接着切换模式若仍偶发 409 `external_stack`，
  重启网关重试。
- Wi-Fi 差（丢包 20–50%）会影响 SSH/HTTP；脚本的 curl 超时较长，失败时先确认网络。

真车落地运动/建图/导航（TC-C1~C5）与急停机械时延仍需场地+人工监护，**未在本次联调执行**。

