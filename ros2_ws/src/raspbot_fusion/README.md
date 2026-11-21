# raspbot_fusion —— WP-E 融合与里程计（/cmd_vel_safe + /odom + TF）

> 契约：CONTRACTS.md v1.2 §2/§3/§4。本包与 `raspbot_description`（URDF/静态 TF）
> 同属 WP-E，本包负责动态部分：`/cmd_vel_safe`、`/odom`、`odom→base_link` TF、
> `/odometry/filtered`。

## 1. 节点 / 发布 / 订阅

| 节点 | 包 | 发布 | 订阅 | 频率 |
|---|---|---|---|---|
| ekf_node | robot_localization（参数本包维护） | `/odom`（remap 自 odometry/filtered）、`odom→base_link` TF | `/odom_raw`（odom0）、`/imu/data`（imu0） | 30 Hz |
| cmd_vel_limiter | raspbot_fusion | `/cmd_vel_safe`（Twist, Reliable） | `/cmd_vel`（Reliable, depth 10）、`/estop`（Bool, Reliable, depth 1）、`/imu/data`（SensorData，仅取 header.stamp） | 30 Hz |
| odom_relay | raspbot_fusion | `/odometry/filtered`（原样转发） | `/odom` | 30 Hz（跟随 ekf） |

- `/cmd_vel_safe`、`/odom`、`/odometry/filtered` 的**唯一发布者**均在本包（SW-C04）。
- `odom→base_link` TF 唯一 authority 是 ekf_node（SW-C05）。
- 前置：`raspbot_description` 的 robot_state_publisher 已在运行（ekf_node 需要
  `base_link→imu_link` TF 才能消费 `/imu/data`）。

## 2. 参数

### 2.1 config/limiter.yaml（节点 cmd_vel_limiter）

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

### 2.2 config/ekf.yaml（节点 ekf_node）

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
| print_diagnostics | false | 默认 true 会向 `/diagnostics` 发布；契约规定其唯一发布者是 H（SW-C04） |
| process_noise_covariance | [0.05,0.05,1e-9,1e-9,1e-9,0.01, 0.05,0.05,1e-9,1e-9,1e-9,0.02, 0.1,0.1,0.1] | 无编码器 → 位置/速度不确定性靠过程噪声维持；yaw 项 < 位置项；2D 锁零轴给 1e-9；ax/ay/az 0.1 占位（未启用加速度观测） |
| initial_estimate_covariance | [0.01,0.01,1e-9,1e-9,1e-9,0.01, 0.1,0.1,0.1,1e-9,1e-9,0.1, 0.1,0.1,0.1] | 启动不确定度取小值；速度 0.1 允许快速收敛；2D 锁零轴给 1e-9；ax/ay/az 0.1 占位 |

完整注释见 `config/ekf.yaml`。

## 3. 启动

```bash
# 1) 静态 TF（必需，先于/并行于 fusion）
ros2 launch raspbot_description description.launch.py
# 2) 融合栈（/odom + /cmd_vel_safe + /odometry/filtered）
ros2 launch raspbot_fusion fusion.launch.py
# bag 回放 / mock 联调：
ros2 launch raspbot_description description.launch.py use_sim_time:=true
ros2 launch raspbot_fusion fusion.launch.py use_sim_time:=true
```

离线 mock 联调（不碰真车）：用 H 的 `fake_base`（30 Hz 发 `/odom_raw`）+ `fake_imu`
（100 Hz 发 `/imu/data`）替代 A/B，加本包两个 launch，检查
`ros2 topic hz /odom /cmd_vel_safe /odometry/filtered`、`ros2 run tf2_tools view_frames`。

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
——这是本看门狗存在的安全原因（lead 补充 P0）。

斜率限幅只作用于**正常指令目标的变化**：包括超时/急停解除后的**从零恢复加速**
（无突跳，FR-E-04）。安全归零（超时/急停）一律瞬时。

## 5. 离线测试（脱离真车）

```bash
cd ros2_ws/src/raspbot_fusion
python3 -m pytest tests -q          # 33 项，全绿（Windows 开发机已验证）
python3 -m py_compile raspbot_fusion/*.py launch/*.launch.py tests/test_limiter.py
```

覆盖：阶跃限幅（解析断言）、幅值夹紧、estop 锁存/解除、IMU 掉线降级与恢复、
**输入超时看门狗**、dt 异常保护（SW-U02 精神）、非法参数拒绝。
`raspbot_fusion/limiter.py` 为纯逻辑（不 import rclpy/sensor_msgs），可直接离线跑。

URDF 检查：`python3 raspbot_description/tools/check_urdf.py`。

## 6. 已知限制

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
