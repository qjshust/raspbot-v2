# raspbot_fusion —— WP-E 融合与里程计（C++ 重写版）

> 契约：CONTRACTS.md v1.2 §2/§3/§4。本包与 `raspbot_description`（URDF/静态 TF）
> 同属 WP-E，本包负责动态部分：`/cmd_vel_safe`、`/odom`、`odom→base_link` TF、
> `/odometry/filtered`。
>
> **v0.2.0（C++ 重写）**：原 Python 实现已归档到
> `E:\code\raspberrypi\legacy_python\raspbot_fusion\`；`tests/test_limiter.py`
> （33 用例）作为语义回归资产保留（字节不变），C++ 侧由
> `tests/test_limiter.cpp`（gtest，33 用例 1:1 映射）执行。

## 1. 节点 / 发布 / 订阅

| 节点 | 包 | 发布 | 订阅 | 频率 |
|---|---|---|---|---|
| ekf_node | robot_localization（参数本包维护） | `/odom`（remap 自 odometry/filtered）、`odom→base_link` TF | `/odom_raw`（odom0）、`/imu/data`（imu0） | 30 Hz |
| cmd_vel_limiter | raspbot_fusion（C++） | `/cmd_vel_safe`（Twist, Reliable） | `/cmd_vel`（Reliable, depth 10）、`/estop`（Bool, Reliable, depth 1）、`/imu/data`（SensorData，仅取 header.stamp） | 30 Hz |
| odom_relay | raspbot_fusion（C++） | `/odometry/filtered`（原样转发） | `/odom` | 30 Hz（跟随 ekf） |

- `/cmd_vel_safe`、`/odom`、`/odometry/filtered` 的**唯一发布者**均在本包（SW-C04）。
- `odom→base_link` TF 唯一 authority 是 ekf_node（SW-C05）。
- ekf_node 的 diagnostics 发布器重映射到 `ekf/diagnostics`（launch 内
  `remappings=[('diagnostics', 'ekf/diagnostics')]`），再由 H 的 safety_node
  聚合进 `/diagnostics`，规避单发布者冲突。
- **必须保持 `print_diagnostics: true`**（2026-11-26 真机修正）：r_l 在
  `print_diagnostics=false` 时只跳过 `freq_diag_->tick()` 与
  `force_update()`，但已在构造时注册的 `HeaderlessTopicDiagnostic` 任务仍会被
  diagnostic_updater 的 1 Hz 定时器发布，于是 `ekf/diagnostics` 每秒出现一条
  **假的 ERROR**：`odometry/filtered topic status: No events recorded.`
  （values：Events 0 / Actual frequency 0），而 `/odom` 实际稳定 30 Hz。
  改为 true 后该条目变为 OK（实测 29.9999 Hz，Events in window 150），并多一条
  "Filter diagnostic updater: ... functioning properly."，两者均由 safety_node
  聚合；不影响 `/diagnostics` 单发布者契约。
- 前置：`raspbot_description` 的 robot_state_publisher 已在运行（ekf_node 需要
  `base_link→imu_link` TF 才能消费 `/imu/data`）。

## 2. 代码结构

```
include/raspbot_fusion/limiter.hpp   纯逻辑限速器接口（无 ROS 依赖）
src/limiter.cpp                      实现（纯标准库）
src/cmd_vel_limiter.cpp              节点 cmd_vel_limiter（rclcpp）
src/odom_relay.cpp                   节点 odom_relay（rclcpp）
tests/test_limiter.cpp               gtest 33 用例（与 test_limiter.py 1:1）
tests/test_limiter.py                Python 回归资产（字节不变，不参与构建）
config/{ekf.yaml,limiter.yaml}       参数（原样 install 到 share/）
launch/fusion.launch.py              启动（原样 install 到 share/）
```

构建目标：库 `raspbot_fusion_lib`（limiter 纯函数）、可执行 `cmd_vel_limiter`、
`odom_relay`（名字与 Python 版一致）。

## 3. 参数

### 3.1 config/limiter.yaml（节点 cmd_vel_limiter）

| 参数 | 默认 | 说明 |
|---|---|---|
| control_rate | 30.0 | `/cmd_vel_safe` 发布频率 Hz |
| max_linear | 0.30 | 满速线性上限 m/s（FR-E-04：输出 ≤ max_linear） |
| max_angular | 1.5 | 满速角速度上限 rad/s |
| max_lin_accel | 0.6 | 一阶斜率限幅 m/s²（FR-E-04：阶跃无突跳） |
| max_ang_accel | 2.0 | 角加速度限幅 rad/s² |
| degraded_max_linear | 0.15 | IMU 掉线时线性上限 m/s（SYS-S03 判据 ≤0.5） |
| imu_timeout | 1.0 | IMU age > 1 s 视为掉线（设计 §7.4） |
| input_timeout | 0.4 | `/cmd_vel` 静默超时（P0，见 §4） |
| warn_interval | 2.0 | 状态转换告警限频（WARN ≤1/2 s） |

### 3.2 config/ekf.yaml（节点 ekf_node，robot_localization 3.10.0）

关键项（`*_config` 为 12 元 = [x,y,z,roll,pitch,yaw, vx,vy,vz,vroll,vpitch,vyaw]；
**协方差矩阵为 15 元** = [x,y,z,roll,pitch,yaw, vx,vy,vz,vroll,vpitch,vyaw, ax,ay,az]，与 r_l 3.10.0 状态向量一致）：

| 参数 | 值 | 理由 |
|---|---|---|
| frequency / two_d_mode / publish_tf | 30.0 / true / true | 契约 30 Hz；地面 2D 车；TF 仅 E 可发 |
| odom0: odom_raw，config 仅 vx,vy,vyaw | [F×6, T,T,F,F,F,T] | **无编码器：开环位姿不得当观测**（SW-C06），只用速度，权重由 A 消息协方差决定 |
| imu0: imu/data，config 仅 yaw,yaw_rate | [F,F,F,F,F,T,F,F,F,F,F,T] | HWT101 单轴 Z：roll/pitch 恒 0 协方差 1.0，不可用（设计 §5.1.1） |
| imu0 yaw 观测噪声 | 消息自带 | FR-E-03：直接使用 `/imu/data` 的 orientation 协方差（raspbot_imu/config/imu.yaml `orientation_yaw_variance`，10⁻³ 量级） |
| odom0_queue_size / imu0_queue_size | 10 / 10 | 默认 20 → 10 |
| odom0_sensor_timeout / imu0_sensor_timeout | 0.2 / 1.0 | 默认 1/frequency≈0.033 s 对 30 Hz odom 太紧（抖动误拒）；IMU 对齐设计 §7.4 的 1 s 掉线判定 |
| print_diagnostics | **true** | 必须为 true：r_l 在 false 时跳过 freq_diag tick 但仍注册诊断任务，会向 `ekf/diagnostics` 发布假 ERROR "No events recorded."（2026-11-26 真机实测）；true 后输出真实 30 Hz 频率诊断，配合 launch 的 diagnostics remap 仍满足 `/diagnostics` 单发布者契约 |
| process_noise_covariance | [0.05,0.05,1e-9,1e-9,1e-9,0.01, 0.05,0.05,1e-9,1e-9,1e-9,0.02, 0.1,0.1,0.1] | 无编码器 → 位置/速度不确定性靠过程噪声维持；yaw 项 < 位置项；2D 锁零轴给 1e-9；ax/ay/az 0.1 占位（未启用加速度观测） |
| initial_estimate_covariance | [0.01,0.01,1e-9,1e-9,1e-9,0.01, 0.1,0.1,0.1,1e-9,1e-9,0.1, 0.1,0.1,0.1] | 启动不确定度取小值；速度 0.1 允许快速收敛；2D 锁零轴给 1e-9；ax/ay/az 0.1 占位 |

完整注释见 `config/ekf.yaml`。

## 4. 三级停车链路（P0，J-07 / SW-F01）

`/cmd_vel`（twist_mux 输出）静默后的停车链路，从最后有效指令起算 **≤0.5 s**：

1. **第 1 级（0.4 s，本包 input_timeout）**：limiter 检测到 `/cmd_vel` 超过 0.4 s
   未更新 → 输出**瞬时归零**（安全路径，**不经斜率限幅**——否则 0.2 m/s 按
   0.6 m/s² 减速还要再花 0.33 s，SW-F01 会超 0.54 s 判据），并以 30 Hz **持续
   发布**零速 `/cmd_vel_safe`。
   为什么是 0.4 s 而不是 0.5 s：判据要求 ≤0.5 s，要留出 mux→limiter→base 两级
   各约 33 ms（一个 30 Hz 周期）的传播时间，总链 ≈0.47 s ≤ 0.54 s。
2. **第 2 级（0.5 s，A 的 cmd_timeout）**：若 limiter 或链路本身故障，A 的底盘
   看门狗在收不到 `/cmd_vel_safe` 更新 0.5 s 后四轮置零（兜底）。
3. **第 3 级（即时，/estop）**：收到 `/estop=true` → limiter 锁存，输出恒零；
   显式 `/estop=false` 解除（CONTRACTS §2 锁存语义）。

没有第 1 级时，limiter 会把最后一条非零指令永远转发下去，A 的看门狗永不触发
——这是本看门狗存在的安全原因。

斜率限幅只作用于**正常指令目标的变化**：包括超时/急停解除后的**从零恢复加速**
（无突跳，FR-E-04）。安全归零（超时/急停）一律瞬时。

## 5. 构建与测试（Pi 上）

```bash
cd ~/ros2_ws
colcon build --packages-select raspbot_fusion
colcon test --packages-select raspbot_fusion    # gtest 33 用例
colcon test-result --verbose
```

脱离真车的运行验证（mock 联调，不碰硬件）：

```bash
ros2 launch raspbot_description description.launch.py
ros2 launch raspbot_fusion fusion.launch.py
ros2 topic hz /odom /cmd_vel_safe /odometry/filtered
ros2 run tf2_tools view_frames
```

无 `/imu/data` 与 `/cmd_vel` 输入时，cmd_vel_limiter 应 30 Hz 持续发布零速
`/cmd_vel_safe`，并输出「IMU 掉线」「输入指令超时」两条限频 WARN（掉线降级路径
的正常行为）；用 fake_base/fake_imu 喂数据则无告警。

## 6. 语义差异说明（C++ vs 归档 Python 版）

接口类型映射（**语义等价**，gtest 已按同一容差逐条复刻）：

| Python | C++ | 说明 |
|---|---|---|
| `imu_age=None` / `input_age=None`（从未收到） | `kNever` = NaN | `imu_stale(NaN)`/`input_stale(NaN)` 返回 true，与 None 一致 |
| 非数值字符串 `'x'` | 无对应 | C++ 强类型接口不存在该场景；NaN 覆盖其全部行为路径（构造校验 / dt 保护 / 新鲜度判定） |
| 构造参数 `None`/`'x'` | NaN / 编译期拒绝 | 构造校验统一抛 `std::invalid_argument`（对应 ValueError） |
| `ValueError` | `std::invalid_argument` | 非法参数拒绝语义一致（SW-U15 精神） |
| `rclpy.time.Time.from_msg(..., clock_type=节点时钟)` | `rclcpp::Time(msg.header.stamp, this->now().get_clock_type())` | 时间戳统一为节点时钟类型后再相减 |
| `time.monotonic()`（dt/input_age） | `std::chrono::steady_clock` | 均不受系统时间跳变影响 |

限幅语义（**完全一致**，无行为差异）：
- 处理顺序：estop 锁存（恒零）→ 输入超时（瞬时归零）→ 斜率限幅 → 幅值夹紧；
- dt 保护：`!(0 < dt ≤ 0.5)`（含 NaN）→ 按 0 处理，dt=0.5 恰好合法；
- 目标数值防护：NaN/inf → 0（安全方向）；
- estop 锁存期间与输入超时期间内部状态清零，解除/恢复后从零平滑加速；
- IMU 掉线 → 线性幅值钳制 `degraded_max_linear`，角速度上限不变。

## 7. 已知限制

- **无编码器**：`/odom` 的平移仍来自 `/odom_raw` 的指令速度积分（会包含打滑/地面
  误差），EKF 只把 `/imu/data` 的 **yaw 与 yaw 速率**当观测——即**只有航向被 IMU
  修正**，平移精度依赖 A 的开环模型（设计 §7.3 阶段一：直线 2 m 误差 <5 cm 需
  A 的 pwm_per_mps 标定配合）。后续编码器/雷达匹配是阶段二/三的事。
- **协方差来源**：EKF 直接采用 A/B 消息内协方差作为观测噪声。当前 B 的 imu.yaml
  `orientation_yaw_variance=0.0003`（比设计 §7.3 的 0.0025 更紧，WP-B 已收紧）；
  若实测航向跳变，调 `imu0_config` 权重或让 B 放宽该值。A 的 `/odom_raw` 协方差
  需反映开环速度不确定度（建议 vx/vy 方差 0.01~0.05 量级），否则 EKF 会把开环
  速度当"准真值"。
- `/odom` 与 `/odometry/filtered` 内容相同（后者是前者的原样转发，FR-E-05 P2 的
  轻量实现，供诊断/记录；不额外滤波）。
- EKF 未配置 roll/pitch/加速度；换装 ICM-42688 时需改 `imu0_config`（WP-B 负责
  消息内容，本包只消费契约话题）。
- 本包不发布 `map→odom`（F 的 slam_toolbox/amcl 独占）；`map_frame: map` 仅用于
  命名，`world_frame: odom` 表示无需 map→odom 也可工作。
