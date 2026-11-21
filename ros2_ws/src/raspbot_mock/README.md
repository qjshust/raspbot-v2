# raspbot_mock — mock 三件套（FR-H-04，C++ 版）

用合成数据替代 RASPBOT-V2 的 A/B/C 硬件驱动，供 E/F/G 与平台在**无真车**时做集成
（验收用例 `SW-I04`：无硬件下跑通一次建图与一次定点导航）。
本包为 C++（ament_cmake）实现，行为与归档的 Python 版
（`legacy_python/raspbot_mock/`）一致。

## 节点与话题

| 节点 | 订阅 | 发布 | 频率 | QoS |
|---|---|---|---|---|
| `fake_base` | `/cmd_vel_safe` (geometry_msgs/Twist) | `/odom_raw` (nav_msgs/Odometry, frame `odom`, child `base_link`) | 30 Hz | Reliable, depth 10 |
| `fake_imu` | — | `/imu/data` (sensor_msgs/Imu, frame `imu_link`) | 100 Hz | SensorData |
| `fake_scan` | — | `/scan` (sensor_msgs/LaserScan, frame `laser`) | 10 Hz | SensorData |

- `fake_base`：按指令速度做与 `raspbot_base` 完全一致的半隐式积分（先
  `theta += wz·dt` 再更新 x/y，见 `raspbot_mock/kinematics.hpp`——兼容接口，
  A 的 C++ kinematics 库交付后由 lead 统一替换为直接 include）；带
  `cmd_timeout`（默认 0.5 s 超时归零）；协方差与真底盘同口径。
- `fake_imu`：`yaw = yaw_rate_deg_s · t`（默认 0，静止指向 0）；四元数仅由 yaw
  生成；协方差默认 yaw 0.0025、tilt 1.0、gyro 0.0004、accel 0.04。
- `fake_scan`：合成 5×4 m 矩形房间（车在 (0,0)），360° 720 点（0.5°），
  射线-矩形求交（slab 法，`raspbot_mock/geometry.hpp`）；未命中按
  `range_max=15` 报告；命中加 `noise` 高斯扰动。**0.05 m 栅格下 5×4 m 房间
  = 100×80 格地图（07_nav2_sil 依赖）**。

## 参数（config/mock.yaml）

| 节点 | 参数 | 默认 | 说明 |
|---|---|---|---|
| fake_base | `rate` | 30.0 | 发布频率 (Hz) |
| fake_base | `noise` | 0.0 | 位姿 x/y/theta 噪声 (m/rad) |
| fake_base | `publish_tf` | false | **默认不发 TF**（`odom→base_link` 属 E）；仅独立测试设 true |
| fake_base | `cmd_timeout` | 0.5 | 无指令超时归零 (s) |
| fake_imu | `rate` | 100.0 | 发布频率 (Hz) |
| fake_imu | `yaw_rate_deg_s` | 0.0 | 恒定转向速率 (°/s) |
| fake_imu | `noise` | 0.0 | yaw 噪声 (rad) |
| fake_scan | `rate` | 10.0 | 发布频率 (Hz) |
| fake_scan | `room_x` / `room_y` | 5.0 / 4.0 | 合成房间尺寸 (m) |
| fake_scan | `noise` | 0.002 | 距离噪声 (m) |
| fake_scan | `range_min` / `range_max` | 0.2 / 15.0 | 与 C1 雷达一致 |

## 构建与启动

```bash
cd ~/ros2_ws && colcon build --packages-select raspbot_mock

# 三件套一起（SW-I04 冒烟）
ros2 launch raspbot_mock mock.launch.py

# 单节点
ros2 run raspbot_mock fake_base --ros-args -p publish_tf:=true

# 与总启动结合（bringup 会按 use_mock 自动替换硬件驱动）
ros2 launch raspbot_bringup bringup.launch.py use_mock:=true mode:=slam
```

## 测试（gtest）

```bash
colcon test --packages-select raspbot_mock
colcon test-result --verbose
# 直接运行：
#   ros2 run raspbot_mock test_geometry   （5 用例，1:1 映射 Python tests/test_geometry.py）
#   ros2 run raspbot_mock test_kinematics （4 用例，C++ 新增）
```

## 已知限制

- `/odom_raw` 是开环航位推算（与真车一致：无编码器），长期漂移不可避免；
  mock 下 EKF/建图只能证明**链路与算法**，不代表实车精度。
- 不发 `/range`、`/line_sensor`（真底盘专属）——safety 对 `/range` 无数据不告警。
- `fake_scan` 为凸房间模型，无动态障碍。
- mock 模式没有真实地图源：`use_mock:=true mode:=nav` 的 localization 需静态地图，
  详见 raspbot_bringup/system/README.md。
- 运动学接口为兼容层（`raspbot_mock::kinematics`），待 A 的 raspbot_base C++
  库交付后由 lead 统一对齐（package.xml 已声明 `<depend>raspbot_base</depend>`）。
