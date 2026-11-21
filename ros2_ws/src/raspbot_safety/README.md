# raspbot_safety — 平台安全监控与 /diagnostics 聚合（FR-H-03，C++ 版）

`safety_node` 以 1 Hz 发布 `/diagnostics`（Reliable，唯一发布者），内容 = 自身平台
状态 + 各节点诊断聚合 + 安全相关告警。对应验收用例 `SW-C11` 与 `SYS-S04`
（雷达/IMU 掉线可见性）。本包为 C++（ament_cmake）实现，行为与归档的 Python 版
（`legacy_python/raspbot_safety/`）一致。

## 发布与订阅

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 发布 | `/diagnostics` | diagnostic_msgs/DiagnosticArray | 1 Hz 聚合诊断（唯一发布者） |
| 订阅 | `raspbot_base/diagnostics` | DiagnosticArray | 聚合（条目名前缀 `/raspbot_base/diagnostics`） |
| 订阅 | `twist_mux/diagnostics` | DiagnosticArray | 聚合（前缀 `/twist_mux/diagnostics`） |
| 订阅 | `lidar_filter/diagnostics` | DiagnosticArray | 聚合（前缀 `/lidar_filter/diagnostics`） |
| 订阅 | `ekf/diagnostics` | DiagnosticArray | 聚合（前缀 `/ekf/diagnostics`） |
| 订阅 | `/estop` | std_msgs/Bool | 急停状态上报（Reliable, depth 1） |
| 订阅 | `/range` | sensor_msgs/Range | 最近 1 s 窗口最小值 <0.35 m → WARN |
| 订阅 | `/imu/data` | sensor_msgs/Imu | 失联 >1 s → WARN（FR-E-02 可见性） |
| 订阅 | `/odom_raw` | nav_msgs/Odometry | 失联 >0.5 s → WARN |
| 订阅 | `/scan` | sensor_msgs/LaserScan | 失联 >0.5 s → WARN（SYS-S04 可见性） |

诊断条目：Heartbeat / CPU（/proc/stat 差值 + getloadavg 1/5/15）/ Memory /
Temperature（thermal_zone0，≥80 °C → WARN，设计 §7.4）/ Uptime / E-Stop /
Proximity / imu·odom_raw·scan 时效监控。`DiagnosticStatus.level` 为 uint8
（C++ 原生字节，对应 Python 版的 bytes 修复）。

## 参数（config/safety.yaml）

| 参数 | 默认 | 说明 |
|---|---|---|
| `rate` | 1.0 | 发布频率 (Hz) |
| `temp_warn_c` | 80.0 | SoC 温度告警 (°C) |
| `cpu_warn_pct` / `mem_warn_pct` | 90.0 | CPU / 内存告警 (%) |
| `range_warn_m` | 0.35 | 近距离告警 (m) |
| `range_window_s` | 1.0 | 距离观察窗口 (s) |
| `imu_stale_s` | 1.0 | IMU 失联判定 (s) |
| `odom_stale_s` / `scan_stale_s` | 0.5 | 里程计 / 雷达失联判定 (s) |

## 构建与启动

```bash
cd ~/ros2_ws && colcon build --packages-select raspbot_safety

ros2 launch raspbot_safety safety.launch.py
ros2 topic echo /diagnostics --once

# 急停 / 恢复（脚本位于 ros2_ws/scripts/）
bash ros2_ws/scripts/estop.sh          # 发布 /estop true（锁停）
bash ros2_ws/scripts/estop_release.sh  # 发布 /estop false（恢复）
```

## 测试（gtest）

```bash
colcon test --packages-select raspbot_safety
colcon test-result --verbose
# 直接运行：
#   ros2 run raspbot_safety test_sysinfo      （17 用例，1:1 映射 Python tests/test_sysinfo.py）
#   ros2 run raspbot_safety test_thresholds   （8 用例，1:1 映射 Python tests/test_thresholds.py）
```

## 与 Python 版的差异（C++ 迁移说明）

- **聚合方式**：Python 版动态发现所有非 `/diagnostics` 的 DiagnosticArray 话题；
  C++ 版按契约**静态订阅四个 remap 后的诊断话题**
  （raspbot_base/twist_mux/lidar_filter/ekf 的 diagnostics）。新增诊断源需在此
  package 内加订阅（对应参数 `topic_scan_period_s` 已随动态发现一起移除）。
- `uptime_s` 值格式化从 Python `str(float)`（repr 最短表示）改为 `%.2f`，
  仅影响显示精度。
- `/range` 无数据（mock 模式或传感器空闲）**不告警**；真车模式有数据且
  <0.35 m 才 WARN（与 Python 版一致）。
- I2C 错误计数属 WP-A（SYS-S05）：A 发布诊断话题后由本节点聚合进 `/diagnostics`。
- 任何节点直接向 `/diagnostics` 发布都会破坏单发布者约定（SW-C04），
  新诊断源必须 remap 到专属话题名后由本节点聚合。
