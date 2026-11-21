# raspbot_base — RASPBOT-V2 底盘驱动 (WP-A, C++)

ROS 2 (rclcpp) 底盘驱动：通过 I2C `0x2B`（`/dev/i2c-1`，原生 ioctl，无 libi2c/smbus 依赖）控制亚博 RASPBOT-V2 的四轮麦轮电机、双舵机、蜂鸣器、RGB 灯条，并读取超声波与 4 路巡线传感器。运行于 Pi 5 / Ubuntu 26.04 / ROS 2 Lyrical。

契约对齐 **CONTRACTS v1.2**：本包只消费 `/cmd_vel_safe`（WP-E 发布），只发布 `/odom_raw`，**不广播 TF**（`odom→base_link` 唯一归 EKF）。行为与已归档的 Python 版（`legacy_python/raspbot_base/`）逐项等价。

## 1. 发布 / 订阅

| 话题 | 类型 | 方向 | 频率 | QoS |
|---|---|---|---|---|
| `/cmd_vel_safe` | `geometry_msgs/Twist` | 订阅（话题名由参数 `cmd_vel_topic` 决定） | 30 Hz | Reliable depth 10 |
| `/buzzer` | `std_msgs/Bool` | 订阅 | 事件 | Reliable depth 10 |
| `/led` | `std_msgs/ColorRGBA` | 订阅 | 事件 | Reliable depth 10 |
| `/servo_pan` | `std_msgs/Float32` | 订阅 | 事件 | Reliable depth 10 |
| `/servo_tilt` | `std_msgs/Float32` | 订阅 | 事件 | Reliable depth 10 |
| `/odom_raw` | `nav_msgs/Odometry` | 发布（frame `odom`→`base_link`） | 30 Hz | Reliable depth 10 |
| `/range` | `sensor_msgs/Range` | 发布（`ultrasonic`, ULTRASOUND, fov 0.26, 0.02~4.0 m） | 5 Hz | SensorData |
| `/line_sensor` | `std_msgs/UInt8` | 发布（原始寄存器字节，极性不翻转） | 5 Hz | Reliable depth 10 |
| `/raspbot_base/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发布（`heartbeat`/`i2c_error_count`/`last_cmd_age_sec`） | 1 Hz | Reliable depth 10 |

TF：本节点**不发布任何 TF**（CONTRACTS v1.2 §3）。
诊断：节点按 Python 版 diagnostic_updater 的默认绝对名发布 `/diagnostics`，`base.launch.py` 将其重映射为 `/raspbot_base/diagnostics`，由 WP-H 聚合后合并进唯一 `/diagnostics`（SW-C04）。不经 launch 直接 `ros2 run` 时才会占用 `/diagnostics`。

## 2. 参数（config/params.yaml）

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_linear` | `0.30` | 线速度上限 (m/s) |
| `max_angular` | `1.50` | 角速度上限 (rad/s) |
| `k_geom` | `0.1245` | 麦轮几何系数 (m) |
| `pwm_per_mps` | `700.0` | PWM/速度换算系数 |
| `max_pwm` | `255` | PWM 饱和值（超限按比例缩放） |
| `deadband_pwm` | `25` | 死区：非零且 \|pwm\|<25 → ±25 |
| `wheel_signs` | `[1,1,1,1]` | 逐轮符号（L1 L2 R1 R2），必须 4 项 |
| `control_rate` | `30.0` | 控制环频率 (Hz) |
| `sensor_rate` | `5.0` | 传感器环频率 (Hz) |
| `cmd_timeout` | `0.5` | 指令超时 (s)，超时按零速处理 (FR-A-02) |
| `cmd_vel_topic` | `cmd_vel_safe` | 速度指令订阅话题名 |
| `cmd_resend_interval` | `1.0` | PWM 重发间隔 (s) (FR-A-06) |
| `diag_rate` | `1.0` | 诊断上报频率 (Hz) |
| `odom_frame` | `odom` | odom 消息 frame_id |
| `base_frame` | `base_link` | odom 消息 child_frame_id |

## 3. 构建与启动

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select raspbot_base
source install/setup.bash
ros2 launch raspbot_base base.launch.py            # 加载 config/params.yaml + 诊断重映射
ros2 launch raspbot_base base.launch.py params_file:=/path/to/params.yaml
ros2 run raspbot_base base_node
```

退出行为：SIGINT/SIGTERM 均优雅退出（退出码 0，先四轮置零+关超声）；`base.launch.py` 带 `on_exit=Shutdown()`（SW-F03：base 意外死亡 → 整个 launch 退出 → systemd Restart=always 重启 bringup）。I2C 打开失败（如 `/dev/i2c-1` 不存在）时启动报错退出（非零码，交由 systemd 重启）。

## 4. 已知限制

- **开环里程计**：无编码器，`/odom_raw` 是命令速度积分，协方差按诚实大值上报（pose x/y 0.10、yaw 0.20；twist x/y 0.05、yaw 0.10）。不得直接用于 Nav2/SLAM 定位。
- I2C 总线互斥：同一时刻只能有一个进程打开 `/dev/i2c-1`。
- 舵机 2（tilt）机械限位 ~110°，驱动层强制 clamp [0,110]，不依赖厂商库。
- 读失败：每次失败计入 `i2c_error_count`，WARN ≤1 次/5 s，节点绝不退出，总线恢复后自动续发（SW-G08/SYS-S05）。电机写失败有 5 s 节流 WARN（Python 版厂商库静默吞掉，此为诊断增强）。
- 无硬件急停：扩展板保持最后 PWM，`kill -9` 后需 systemd 重启兜底（SW-F03）。

## 5. 离线测试方法

纯逻辑库 `raspbot_base_lib`（kinematics/odometry/sensors/safety）不依赖 ROS/硬件，gtest 50 项（1:1 映射 Python 版 45 用例 + 5 项补充），Pi 上：

```bash
colcon test --packages-select raspbot_base && colcon test-result --verbose
# 或直接跑
./build/raspbot_base/test_kinematics && ./build/raspbot_base/test_odometry
./build/raspbot_base/test_safety && ./build/raspbot_base/test_sensors
./build/raspbot_base/test_component
```

历史 Python 测试（`test/*.py`，45 用例）按要求原样保留，C++ 包不运行它们；Python 源码归档于 `legacy_python/raspbot_base/`（行为参考）。
