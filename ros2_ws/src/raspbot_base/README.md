# raspbot_base — RASPBOT-V2 底盘驱动 (WP-A)

ROS 2 底盘驱动：通过 I2C `0x2B`（`/dev/i2c-1`）控制亚博 RASPBOT-V2 的四轮麦轮电机、双舵机、蜂鸣器、RGB 灯条，并读取超声波与 4 路巡线传感器。运行于 Pi 5 / Ubuntu 26.04 / ROS 2 Lyrical。

契约对齐 **CONTRACTS v1.2**：本包只消费 `/cmd_vel_safe`（由 WP-E 限幅节点发布），只发布 `/odom_raw`，**不再广播 TF**（`odom→base_link` 唯一归 EKF）。

## 1. 发布 / 订阅

| 话题 | 类型 | 方向 | 频率 | QoS |
|---|---|---|---|---|
| `/cmd_vel_safe` | `geometry_msgs/Twist` | 订阅（话题名由参数 `cmd_vel_topic` 决定） | 30 Hz | Reliable |
| `/buzzer` | `std_msgs/Bool` | 订阅 | 事件 | Reliable |
| `/led` | `std_msgs/ColorRGBA` | 订阅 | 事件 | Reliable |
| `/servo_pan` | `std_msgs/Float32` | 订阅 | 事件 | Reliable |
| `/servo_tilt` | `std_msgs/Float32` | 订阅 | 事件 | Reliable |
| `/odom_raw` | `nav_msgs/Odometry` | 发布（`frame_id=odom`, `child=base_link`） | 30 Hz | Reliable |
| `/range` | `sensor_msgs/Range` | 发布（`frame_id=ultrasonic`, ULTRASOUND, fov 0.26, 0.02~4.0 m） | 5 Hz | SensorData |
| `/line_sensor` | `std_msgs/UInt8` | 发布（原始寄存器字节，极性不翻转） | 5 Hz | Reliable |
| `/raspbot_base/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发布（`heartbeat` / `i2c_error_count` / `last_cmd_age_sec`） | 1 Hz | Reliable |

TF：本节点**不发布任何 TF**（CONTRACTS v1.2 §3，`odom→base_link` 由 E 的 EKF 发布）。

诊断话题说明：`diagnostic_updater` 默认发布 `/diagnostics`，但契约规定 `/diagnostics` 的唯一发布者是 H 的聚合节点（SW-C04 单发布者），因此 `base.launch.py` 将本节点的话题重映射为 `/raspbot_base/diagnostics`；H 的聚合器订阅该话题并合并进 `/diagnostics`。直接 `ros2 run`（不经 launch）时才会使用 updater 默认的 `/diagnostics`。

## 2. 参数（config/params.yaml）

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_linear` | `0.30` | 线速度上限 (m/s) |
| `max_angular` | `1.50` | 角速度上限 (rad/s) |
| `k_geom` | `0.1245` | 麦轮几何系数 (m)，悬空标定 |
| `pwm_per_mps` | `700.0` | PWM/速度换算系数 |
| `max_pwm` | `255` | PWM 饱和值（超过按比例缩放） |
| `deadband_pwm` | `25` | 死区：非零且 \|pwm\|<25 → ±25 |
| `wheel_signs` | `[1,1,1,1]` | 逐轮符号（L1 L2 R1 R2） |
| `control_rate` | `30.0` | 控制环频率 (Hz) |
| `sensor_rate` | `5.0` | 传感器环频率 (Hz) |
| `cmd_timeout` | `0.5` | 指令超时 (s)，超时按零速处理 (FR-A-02) |
| `cmd_vel_topic` | `cmd_vel_safe` | 速度指令订阅话题名 |
| `cmd_resend_interval` | `1.0` | PWM 重发间隔 (s)，扩展板复位后 ≤1 s 恢复 (FR-A-06) |
| `diag_rate` | `1.0` | /diagnostics 上报频率 (Hz) |
| `odom_frame` | `odom` | odom 消息 frame_id |
| `base_frame` | `base_link` | odom 消息 child_frame_id |

## 3. 启动命令

```bash
# 构建（在 Pi 上，由集成阶段统一执行）
cd ~/ros2_ws && colcon build --symlink-install --packages-select raspbot_base
source install/setup.bash

# 启动（默认加载 config/params.yaml）
ros2 launch raspbot_base base.launch.py
# 覆盖参数文件
ros2 launch raspbot_base base.launch.py params_file:=/path/to/params.yaml
# 直接运行
ros2 run raspbot_base base_node --ros-args -p cmd_vel_topic:=cmd_vel_safe
```

退出行为：SIGINT / SIGTERM 均优雅退出（退出码 0），`stop()` 四轮置零并关闭超声波使能；指令静默 0.5 s 后自动按零速处理。

## 4. 已知限制

- **开环里程计**：扩展板无编码器反馈，`/odom_raw` 是“指令速度”的积分，无打滑/负载补偿，协方差按诚实的大值上报（pose x/y 0.10、yaw 0.20；twist x/y 0.05、yaw 0.10）。**不得**直接用于 Nav2/SLAM 定位；仅供遥测、可视化与 E 的 EKF 融合输入。
- 厂商 `Raspbot_Lib` 会吞掉所有 I2C 异常（`except: print(...)`），本节点以“读取返回空”作为失败信号计数 `i2c_error_count` 并限频告警（≤1 次/5 s），**节点不退出**，总线恢复后自动继续发布。
- I2C 总线互斥：同一时刻只能有一个进程打开 `/dev/i2c-1`，不要同时跑厂商 demo 脚本与本节点。
- 舵机 2（tilt）机械限位 ~110°，本节点在驱动层强制 clamp 到 [0,110]，不依赖厂商库限位。
- 无硬件急停：扩展板会保持最后一次 PWM，`kill -9` 后轮子仍转，需依靠 systemd 重启或物理断电（SYS-S02 待 WP-H）。
- 无编码器/无速度反馈，`k_geom`、`pwm_per_mps`、`deadband_pwm` 均需按 §测试方案 CAR-C 在台架上标定。

## 5. 离线测试方法（无需 ROS、无需真车）

纯逻辑模块（`kinematics.py` / `odometry.py` / `sensors.py` / `safety.py`）不依赖 rclpy / smbus，在 Windows 开发机上可直接跑：

```bash
# 单元测试（SW-U01~U10 全跑；SW-G06~G08 组件测试需要 ROS 环境，自动跳过）
python -m pytest test -q
# 只跑纯逻辑
python -m pytest test -q -k "not sw_g"
```

组件测试（`test/test_component.py`，SW-G06/07/08）以 `pytest.importorskip('rclpy')` 开头：无 ROS 的机器上自动跳过；在 Pi 上执行时通过 `test/conftest.py` 注入假 `smbus`（`FakeSMBus`），**绝不打开 `/dev/i2c-1`**：

```bash
# Pi 上（组件测试全绿）
cd ~/ros2_ws/src/raspbot_base && python3 -m pytest test -q
```

对应测试方案：SW-U01~U10（单元）、SW-G06~G08（组件）、D-01/D-02 修复项。
