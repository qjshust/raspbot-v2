# 工作包拆分与接口契约（CONTRACTS）

> 任何 agent 开工前先读本文件。**契约冻结，改动必须同步所有工作包。**

版本：v1.2（2026-11-20）
适用：RASPBOT-V2 / Pi 5 / Ubuntu 26.04 / ROS 2 Lyrical

变更：
- v1.1 仅调整 WP-B 路径 `src/hwt101_driver/**` → `src/raspbot_imu/**`（包同时支持 HWT101 串口与 ICM-42688 SPI），话题、TF、参数契约均未改动。
- v1.2 与设计文档 v2.4 §4.2 对齐并新增 WP-I：
  - 新增话题 `/cmd_vel_safe`（E→A）；`/cmd_vel` 消费者改为 **E**（原写 A）；
  - 新增话题 `/estop`（Bool，锁存语义：true 锁停，显式发 false 恢复）；
  - 新增 WP-I Hailo-8 感知：`src/raspbot_perception/**`、`launch/perception.launch.py`、话题 `/detections`（vision_msgs/Detection2DArray）、`/perception/image`（sensor_msgs/Image）；
  - 新增 WP-H 包归属：`src/raspbot_mock/**`、`src/raspbot_safety/**`、`launch/bringup.launch.py`；
  - 新增 WP-E 归属：`src/raspbot_description/**`（URDF/TF 唯一维护者）。
  - 其余话题、TF、参数冻结项不变。

---

## 1. 所有权（谁改哪个目录）

| 工作包 | 独占路径 | 负责人 |
|---|---|---|
| A 底盘驱动 | `src/raspbot_base/**` | 待分配 |
| B IMU | `src/raspbot_imu/**` | 待分配 |
| C 雷达 | `src/raspbot_bringup/lidar/**`、`launch/lidar.launch.py`、`udev/99-raspbot.rules`（雷达段） | 待分配 |
| D 相机 | `src/raspbot_bringup/camera/**`、`launch/camera.launch.py` | 待分配 |
| E 融合 | `src/raspbot_fusion/**`、`src/raspbot_description/**` | 待分配 |
| F 建图 | `src/raspbot_bringup/slam/**`、`launch/slam.launch.py`、`launch/localization.launch.py`、`maps/**`、`tools/slam/**` | 待分配 |
| G 导航 | `src/raspbot_bringup/nav2/**`、`launch/nav2.launch.py` | 待分配 |
| H 平台 | `src/raspbot_bringup/system/**`、`src/raspbot_mock/**`、`src/raspbot_safety/**`、`launch/bringup.launch.py`、`scripts/**`、`udev/99-raspbot.rules`（通用段） | 待分配 |
| I 感知（Hailo-8） | `src/raspbot_perception/**`、`launch/perception.launch.py` | 待分配 |

规则：

- 只能修改自己表中的路径；跨包需求写成 issue，由属主改。
- `src/raspbot_description/**`（URDF/TF）由 E 统一维护，其他人提需求。
- 任何新增依赖必须在 `package.xml` 中声明，并在 PR 说明里列出。

---

## 2. 话题契约

| 话题 | 类型 | 产生者 | 消费者 | 频率 | QoS |
|---|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | teleop / Nav2（经 twist_mux 汇入） | E | ≤30 Hz | Reliable, depth 10 |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | **E** | A | 30 Hz | Reliable |
| `/odom_raw` | `nav_msgs/Odometry` | **A** | E | 30 Hz | Reliable |
| `/imu/data` | `sensor_msgs/Imu` | **B** | E, H | 100 Hz | SensorData |
| `/scan` | `sensor_msgs/LaserScan` | **C** | F, G, H | 10 Hz | SensorData |
| `/odom` | `nav_msgs/Odometry` | **E** | G, 诊断 | 30 Hz | Reliable |
| `/odometry/filtered` | `nav_msgs/Odometry` | E | 诊断/记录 | 30 Hz | Reliable |
| `/range` | `sensor_msgs/Range` | A | 安全监控 | 5 Hz | SensorData |
| `/line_sensor` | `std_msgs/UInt8` | A | 巡线逻辑 | 5 Hz | Reliable |
| `/image_raw` | `sensor_msgs/Image` | D | I（感知） | 15 Hz | SensorData |
| `/detections` | `vision_msgs/Detection2DArray` | I | 监控/决策 | ≤15 Hz | SensorData |
| `/perception/image` | `sensor_msgs/Image` | I | 监控 | ≤15 Hz | SensorData |
| `/estop` | `std_msgs/Bool` | 任意（人工/监控） | E | 事件 | Reliable, depth 1 |
| `/buzzer` | `std_msgs/Bool` | 任意 | A | 事件 | Reliable |
| `/led` | `std_msgs/ColorRGBA` | 任意 | A | 事件 | Reliable |
| `/servo_pan` | `std_msgs/Float32` | 任意 | A | 事件 | Reliable |
| `/servo_tilt` | `std_msgs/Float32` | 任意 | A | 事件 | Reliable |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | H | 监控 | 1 Hz | Reliable |
| `/map` | `nav_msgs/OccupancyGrid` | F | G | 事件 | Transient Local |

补充约定：

- `/cmd_vel` 各轴单位：linear 为 m/s，angular 为 rad/s，正方向遵循右手定则（+x 前、+y 左、+wz 俯视逆时针）。
- `/estop` 锁存语义：收到 true 后 E 持续输出零速，直到显式收到 false。
- `/detections` 的 `header.frame_id` 为 `camera_link`。

约定：

- 一个话题**只能有一个发布者**。
- 角度单位一律弧度，长度一律米，角度正方向遵循右手定则。
- 传感器消息必须带 `header.stamp` 与 `header.frame_id`。

---

## 3. TF 契约

| 变换 | 发布者 | 说明 |
|---|---|---|
| `map → odom` | **F**（slam_toolbox 或 amcl） | 全局修正，仅 F 可发 |
| `odom → base_link` | **E**（EKF） | 连续位姿，仅 E 可发 |
| `base_link → laser` | `robot_state_publisher`（E 维护 URDF） | 静态，标定值 |
| `base_link → imu_link` | 同上 | 静态 |
| `base_link → camera_link` | 同上 | 静态，含俯角 |
| `base_link → ultrasonic` | 同上 | 静态 |
| `base_link → base_footprint` | 同上 | 静态，地面投影 |

禁止：

- 在其他节点里重复发布上表中的任何变换；
- 用 `static_transform_publisher` 发布动态变换；
- 修改 `frame_id` 命名（`base_link` 不是 `base_footprint` 的别名）。

---

## 4. 参数契约

| 文件 | 属主 | 内容 |
|---|---|---|
| `raspbot_base/config/params.yaml` | A | 轮距系数、PWM 标定、超时、速度上限 |
| `raspbot_imu/config/imu.yaml` | B | source、mode、angle_source、yaw_offset_deg、baud、协方差 |
| `raspbot_fusion/config/ekf.yaml` | E | 过程噪声、观测噪声、IMU 外参 |
| `raspbot_fusion/config/limiter.yaml` | E | 限幅、平滑、IMU 掉线降级、estop |
| `raspbot_bringup/slam/slam_params.yaml` | F | 分辨率、匹配参数、地图边界 |
| `raspbot_bringup/nav2/nav2_params.yaml` | G | 代价地图、控制器、行为树、恢复行为 |
| `raspbot_bringup/camera/camera.yaml` | D | 分辨率、帧率、内参 |

参数名一旦被其他包引用即视为契约；重命名需要同步。

---

## 5. 开发与测试隔离

1. **工作区隔离**：每个 agent 使用独立目录（`~/ws_<name>`）或 git worktree，禁止共享 `build/`、`install/`、`log/`。
2. **硬件预约**：同一时刻只允许一个 agent 上电测试；其余用 mock 或 rosbag。
3. **Mock 节点**（由 H 提供，供 E/F/G 离线开发）：
   - `fake_base`：以 30 Hz 发布 `/odom_raw`，接受 `/cmd_vel_safe` 并积分回显；
   - `fake_imu`：发布带噪声的 `/imu/data`；
   - `fake_scan`：从 `rosbag` 回放或合成圆形障碍。
4. **禁止**在车上并行运行两个会打开 `/dev/i2c-1` 的进程。

---

## 6. 完成定义（DoD）

每个工作包交付必须同时满足：

- [ ] `colcon build` 独立通过，无新增警告；
- [ ] 有脱离真车的测试（单元测试或 bag 回放），并在 README 写明运行方式；
- [ ] README 含：发布/订阅列表、参数说明、启动命令、已知限制；
- [ ] 未修改他人路径、未改动本契约；
- [ ] 至少一条对应的验收用例（对应设计文档第 8 章）可执行。

---

## 7. 依赖与波次

```
第 1 波（并行）：A  B  C  D  H  I
第 2 波：        E（依赖 A、B）
第 3 波：        F（依赖 C、E）
第 4 波：        G（依赖 F）
```

集成门禁：

- A 完成 → E 才能开始融合联调；
- E 完成且 `/odom` 指标达标（直行 2 m 误差 <5 cm）→ F 才能建图；
- F 产出可加载地图 → G 才能做定点导航。
