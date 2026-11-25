# RASPBOT-V2 Web 控制台（WP-C3）

`pc_console/web/` 是 PC 上位机的浏览器前端：**原生 HTML + CSS + ES Modules，无框架、无构建、无 CDN、无外部字体/图片/图标**，由 `pc_console` 本地服务以 `http://127.0.0.1:8090/` 提供，离线可用（目标浏览器 Edge/Chrome 最近两年版本）。

接口与 UI 规范严格遵循冻结契约：`docs/PC上位机_设计文档_v1.md` §3（HTTP/SSE API / state schema）、§6（UI 规范）、§8.1（离线测试）。

---

## 1. 目录与模块职责

```
pc_console/web/
  index.html              # 单页结构：顶栏 / 左栏 / 地图画布 / 右栏 / 遥控条 / 日志 / 弹窗 / 内联 SVG 图标
  css/app.css             # 深色控制台主题、三栏响应式布局（1440×900 最佳，最小 1280×720）
  js/config.js            # API 路径、文案映射（会话/导航/诊断/错误码）、限值、快捷键、本地存储 key
  js/math.js              # 纯函数：world↔screen、y 翻转、地图 origin/resolution/yaw 换算、缩放/平移、
                          #         按键/摇杆→麦轮速度、clamp、网格/比例尺、格式化
  js/api.js               # fetch 封装（超时/错误码→中文）、EventSource 封装（retry+指数退避、看门狗）
  js/mapview.js           # canvas 地图：底图离屏缓存、激光/路径/轨迹/机器人/目标/初始位姿、平移缩放跟随
  js/teleop.js            # 键盘 WASD/QE + 虚拟摇杆 + 20 Hz 节流 + 松手即停 + 导航中遥控确认
  js/ui.js                # DOM 小工具、自绘模态框（替代 confirm）、Toast、事件日志面板、地图列表
  js/app.js               # 启动、全局 store、SSE 接入、状态渲染、动作/快捷键、连接恢复
  tests/math.test.mjs     # 纯函数单测（Node ≥24）
  tests/dom_contract.test.mjs # DOM id/资源/禁止项/API 白名单/fixtures schema 静态契约测试
  tests/smoke.test.mjs    # 最小 DOM 桩下真实启动 app.js 的启动/渲染/交互链路冒烟测试（无浏览器）
  tests/fixtures/         # 符合契约的 state/scan/path/maps JSON 夹具
  README.md
```

边界：本目录**只写 `pc_console/web/**`**；`pc_console/main.py`、`console_server/` 属于 WP-C2，`ros2_ws/` 属于 WP-C1。

数据流：`GET /api/state` 快照 + `GET /api/events` SSE（`state` 10 Hz、`scan` 5 Hz、`path`、`map`、`session`、`log`）→ 全局 store → 各面板渲染。**所有状态机字段的权威值来自 state/SSE**；POST 成功仅表示「命令已受理」。

---

## 2. 如何启动与联调

### 2.1 演示模式（无 Pi / 无 ROS，推荐 UI 联调）

```bash
# 仓库根目录
python pc_console/main.py --demo
# 浏览器打开 http://127.0.0.1:8090/ （start-demo.bat 亦可）
```

演示模式（C2 实现）与 Pi 网关 API 完全一致，可走通：建图（地图逐步揭示）→ 保存地图 → 导航（设置初始位姿 → 激活 → 点目标 → 轨迹/剩余距离/成功）→ 遥控/急停/外设/诊断/日志。

> 已用 C2 `--demo` 演示服务实测（2026-11-24，临时集成探针，非交付测试）：`/`、`/css/*`、`/js/*` 静态资源 200；`/api/state` 的 UI 依赖字段齐全；SSE 收到 `state`；`session/start(base)`、`teleop`、`teleop/stop`、`estop(on/off)`、`session/stop`、`maps`/`preview.png`、`local/config`、`local/ping` 全部按契约返回（31/31 项通过）。

演示模式下：

- 顶栏显示琥珀色「演示模式」徽章；
- 设置弹窗中 **Pi 地址/端口/token/连接测试不可编辑**（`POST /local/config` 在演示模式返回 409），符合 §4.1；
- 地图上的「设目的地 / 设初始位姿」**仍可用**——这是验收演示闭环（§4.2、§8.1）所必需的，不属于禁用的「目标编辑」。

### 2.2 连接真车

```bash
python pc_console/main.py --pi 192.168.50.98:8080
```

在设置弹窗中可改 Pi 地址/端口/token（`GET/POST /local/config`）并用「连接测试」（`/local/ping`）验证连通与延迟。

### 2.3 断连容错行为

- SSE 断开：`EventSource` 关闭后按 1/2/4/8/15 s 指数退避重连；同时每 2 s 轮询 `/api/health` 测延迟并作为降级在线判据；
- 断连时**保留最后画面**，地图叠加半透明水印（「未连接 / 数据延迟 · 最后更新 HH:MM:SS」），遥控、目的地、会话启动等危险按钮禁用；
- 恢复后自动补拉 `/api/state`、`/api/maps`、`/local/config` 并 toast「连接已恢复」；
- SSE 连接存活但 20 s 无事件时主动重建（防半开连接假在线）。

---

## 3. 交互与快捷键

| 操作 | 说明 |
|---|---|
| `W / S` | 前进 / 后退（麦轮 vx） |
| `A / D` | 左 / 右横移（麦轮 vy；A=+y，D=-y） |
| `Q / E` | 逆时针 / 顺时针自转（wz） |
| `Shift` | 按住加速（1.6×，**绝不超过** `state.limits`） |
| `V` | 指针工具（拖拽平移 / 查看） |
| `G` | 设目的地工具（仅 `nav.state=ready` 可用；单击地图 → `POST /api/goal`，yaw=null 用当前朝向） |
| `P` | 设初始位姿工具（仅导航会话 running；**按下定位置、拖拽定朝向、松开发送** `POST /api/initialpose`，释放前显示箭头预览） |
| `F` | 跟随机器人 / 取消跟随（拖拽地图会自动取消跟随） |
| `R` | 回中：视角移到机器人 |
| `0` 或 `H` | 整图适配 |
| `+` / `-` | 以画布中心放大 / 缩小（滚轮以鼠标为中心缩放） |
| `Esc` | 取消当前工具 / 关闭弹窗 |
| 双击地图 | 指针工具下回中到机器人 |
| 鼠标/触摸摇杆 | 上推前进、右推右移，松手回中并立即 `POST /api/teleop/stop` |

其他交互：

- **急停**：顶栏红色按钮，点击立即锁存（无确认）；解除需二次确认弹窗。急停中遥控/目的地/会话启动禁用，保留停止会话与解除。
- **速度滑杆**：显示当前上限 `m/s`（`state.limits.max_linear × 比例`），默认 60%。
- **导航中首次遥控**：弹出「取消目标并遥控」确认（网关收到 teleop 也会取消目标，前端再确认一次防误触）。
- **保存地图**：名称弹窗，默认 `map_YYYYMMDD_HHMM`；仅允许 `^[A-Za-z0-9_.-]{1,40}$`；409 `map_exists` 时询问是否覆盖。
- **导航数值输入（备选）**：右栏折叠区可输入 x/y/yaw° 发送目标或初始位姿。
- **日志**：级别过滤、暂停滚动、清空、下载 `.log`、「会话日志」按需拉取 `/api/session/logs`。
- **状态表达**：所有关键状态均有「中文文案 + 颜色 + 状态点/图标」三重表达，不只依赖颜色。

窗口失焦、切换标签页、弹窗打开时：立即停发遥控并零速；弹窗打开时屏蔽遥控键盘。

---

## 4. 契约字段 → UI 映射（自查用）

| state 字段 | UI 位置 |
|---|---|
| `session.mode/state/progress/message/error/map` | 顶栏会话徽章、左栏会话卡、启动进度条、模式卡、地图导航名 |
| `ros.ok/ready/rates/nodes` | 顶栏 ROS 健康（悬停查看 ready 明细与速率） |
| `limits` | 遥控速度上限、速度滑杆显示、前端限幅 |
| `robot.frame/x/y/yaw/speed/imu/scan` | 右栏遥测、地图机器人箭头与轨迹；`frame!=map` 时标黄虚线显示「未定位」 |
| `nav.state/message/initialpose/goal/goal_state/distance_remaining/navigation_time_s/recoveries/activation` | 右栏导航卡、地图目标/初始位姿标记、取消/重新激活按钮 |
| `map.available/version/width/height/resolution/origin` | 底图 `/api/map.png?v=<version>`、HUD、坐标换算、适配/比例尺 |
| `estop.active` | 顶栏急停按钮锁存态、急停横幅、危险操作禁用 |
| `teleop.active` | 遥控条状态文案 |
| `peripherals.buzzer/led/servo` | 左栏外设卡回显 |
| `diagnostics.level/counts/items/age_s` | 右栏诊断计数/摘要/展开列表、顶栏 ERROR 横幅 |
| `logs[]` + SSE `log` | 底部事件日志（按 `ts+msg` 去重） |

可选/`null` 字段（`robot=null`、`map.available=false`、`nav.*=null`、`session.map=null` 等）全部容错为「—」或空态提示；不依赖契约之外的任何字段。

错误映射示例：`nav_not_ready → 「导航尚未就绪：请先设置初始位姿」`、`estop_active → 「已急停：请先解除急停」`、`map_exists → 「同名地图已存在」`；未登记错误码用服务端 `message` 兜底，全部转中文 toast + 日志条目。

---

## 5. 技术约束与性能

- 画布 `devicePixelRatio` 适配（上限 3），底图/网格/比例尺绘制到**离屏 canvas**，仅在视图或地图变化时重绘；动态层（激光/路径/轨迹/机器人/目标）由 `requestAnimationFrame` 合并重绘；主 canvas 每帧合成两层。
- 底图按 `map.version` 换 URL（长缓存）；`map` 事件或 `state.map.version` 变化才重载图片。
- SSE 10 Hz 状态更新只做必要的 DOM 文本更新（`textContent` 差异比较），诊断列表/地图列表按签名重建，避免抖动。
- 断连时 `map.png` 不重载、画布保留，仅叠加水印。
- 响应式：`1440×900` 最佳布局；`≤1400px`、`≤1320px`、`≤1280px` 逐级收窄三栏；`≤780px` 高度时压缩日志/卡片。
- 系统字体栈含 `"Microsoft YaHei"`；等宽数字用 `ui-monospace/Consolas`；无外部字体。
- 无障碍：`lang="zh-CN"`、语义化按钮、`aria-pressed/aria-expanded`、焦点可见、模态框焦点环与 Esc 关闭、`role=log/alert`。

---

## 6. 测试

在**仓库根目录**运行（Node ≥ 24）：

```bash
node --test pc_console/web/tests
```

包含：

- `math.test.mjs`：世界↔屏幕双向、y 翻转（已知点/负坐标）、地图 `origin+resolution+yaw` 换算（图片像素/ OccupancyGrid 栅格/包围盒/适配视图）、屏幕点击→世界、以鼠标为中心缩放的**不动点**、拖拽平移、按键→vx/vy/wz（WASD/QE/Shift/滑杆/对向抵消/方向键别名）、摇杆死区与右移/上推、clamp、网格/比例尺、距离/时长/角度/延迟/时间戳格式化。
- `dom_contract.test.mjs`：交付文件清单与 UTF-8；HTML 引用的本地 js/css 存在、无外部 http(s)/`@import`；HTML id 唯一、`label for`/`aria-*` 引用有效；JS 中 `getElementById('…')`/`$el('…')` 引用的 id 全部存在；禁止 `alert/confirm/prompt/eval/document.write` 与内联事件；**JS 出现的 `/api/*`、`/local/*` 端点严格属于契约白名单**；`tests/fixtures/` 的 state/scan/path/maps JSON 通过 §3.1/§3.2/§3.3.1 schema 校验。
- `smoke.test.mjs`：用最小 DOM / canvas / fetch / EventSource / Image 桩在 Node 中真实执行 `js/app.js`，验证 boot 无异常、契约 state 快照渲染到顶栏/遥测/导航/诊断/地图 HUD、SSE `state/session/map/scan/path/log` 事件驱动 UI、地图选择、急停锁存与二次确认解除、启动会话确认、保存地图弹窗、发送目的地/取消目标的请求体、断连水印与危险按钮禁用、键盘遥控 20 Hz 发送与松手即停。**无需无头浏览器**，可在离线环境回归 UI 逻辑。

语法的快速自查（无需浏览器）：

```bash
node --check pc_console/web/js/math.js
node --check pc_console/web/js/api.js
node --check pc_console/web/js/config.js
node --check pc_console/web/js/ui.js
node --check pc_console/web/js/mapview.js
node --check pc_console/web/js/teleop.js
node --check pc_console/web/js/app.js
```

> 说明：在部分受限沙箱里 Node 测试运行器无法创建子进程（`spawn EPERM`），可改用
> `node --test --experimental-test-isolation=none pc_console/web/tests/math.test.mjs` 或直接
> `node pc_console/web/tests/math.test.mjs` 在单进程内验证；无沙箱环境下 DoD 命令为准。

浏览器验收（由 lead 用无头浏览器截图）：演示模式主界面（建图中 / 导航中 / 急停各一张）存入 `docs/evidence/`。

---

## 7. 已知限制（UI 侧）

1. **无编码器**：`robot.frame=odom`（未定位）时位姿是开环积分，UI 以琥珀色虚线环与「odom 未定位」标签提示，不保证走位精度。
2. **激光帧不一致时隐藏**：`scan`/`path` 事件的 `frame` 非 `map` 且与 `robot.frame` 不一致时**不绘制**（宁可隐藏也不错误叠加坐标）。
3. **2D 雷达限制**：扫描面离地约 74 mm，低矮/玻璃/镜面障碍不可见，必须有人监护并限定场地（帮助弹窗已提示）。
4. 地图底图加载失败时保留旧画面并提示；断连不主动清空地图与激光，便于事后观察最后状态。
5. 虚拟摇杆不发送自转分量（Q/E 负责自转）；不支持多点触控双指缩放（滚轮/按钮缩放已覆盖）。
6. 前端只使用 PC 本地时间做超时/新鲜度判断（`data-age`、水印时间），不与 Pi 时钟直接比较。
