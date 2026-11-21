# raspbot_safety — 平台安全监控与 /diagnostics 聚合（FR-H-03）

`safety_node` 以 1 Hz 发布 `/diagnostics`（Reliable），内容 = 自身平台状态 +
其他节点的诊断聚合 + 安全相关告警。对应验收用例 `SW-C11`（/diagnostics 1 Hz、
含 CPU、温度、心跳等）与 `SYS-S04`（雷达/IMU 掉线可见性）。

## 发布与订阅

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 发布 | `/diagnostics` | diagnostic_msgs/DiagnosticArray | 1 Hz 聚合诊断（唯一发布者） |
| 订阅 | 所有 `diagnostic_msgs/DiagnosticArray` 话题（除 `/diagnostics` 自身） | 动态发现并建订阅 | 子 status 合并进 `/diagnostics`（名称加话题前缀） |
| 订阅 | `/estop` | std_msgs/Bool | 急停状态上报（Reliable, depth 1） |
| 订阅 | `/range` | sensor_msgs/Range | 最近 1 s 窗口最小值 <0.35 m → WARN |
| 订阅 | `/imu/data` | sensor_msgs/Imu | 失联 >1 s → WARN（FR-E-02 可见性） |
| 订阅 | `/odom_raw` | nav_msgs/Odometry | 失联 >0.5 s → WARN |
| 订阅 | `/scan` | sensor_msgs/LaserScan | 失联 >0.5 s → WARN（SYS-S04 可见性） |

诊断条目：

- `Raspbot Platform: Heartbeat`（自增心跳）
- `Raspbot Platform: CPU`（/proc/stat 两次采样差值 + os.getloadavg 1/5/15）
- `Raspbot Platform: Memory`（/proc/meminfo，MemAvailable 口径）
- `Raspbot Platform: Temperature`（thermal_zone0，m°C→°C；≥80 °C → WARN，设计 §7.4）
- `Raspbot Platform: Uptime`
- `Raspbot Safety: E-Stop`（true → WARN "ESTOP ENGAGED"）
- `Raspbot Safety: Proximity`（窗口最小距离 <0.35 m → WARN）
- `Raspbot Sensors: imu/data | odom_raw | scan`（时效监控）

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
| `topic_scan_period_s` | 5.0 | 诊断话题动态发现周期 (s) |

## 启动

```bash
ros2 launch raspbot_safety safety.launch.py
ros2 topic echo /diagnostics --once

# 急停 / 恢复（脚本位于 ros2_ws/scripts/）
bash ros2_ws/scripts/estop.sh          # 发布 /estop true（锁停）
bash ros2_ws/scripts/estop_release.sh  # 发布 /estop false（恢复）
```

## 离线测试

```bash
cd ros2_ws/src/raspbot_safety
python3 -m pytest tests -q     # 纯函数解析 + 阈值判断（无需 ROS）
```

## 已知限制

- **Lyrical 诊断绑定（已适配）**：Pi 上 Lyrical 的新版 rosidl_generator_py 绑定里
  `DiagnosticStatus.level` 是 **bytes**（`OK=b'\x00'`），字符串字段是 str。
  发布时 convert_from_py 对 level 断言 `PyBytes_Check`、对字符串断言
  `PyUnicode_Check`——两者反着用会直接 abort。safety_node 已按此适配
  （`_norm_level()` 转 bytes、字符串保持 str），聚合子状态原样透传。
  写新诊断发布代码时不要沿用旧绑定的 int level 习惯。
- `/range` 无数据（mock 模式或传感器空闲）**不告警**，避免 SW-I04 噪声；
  真车模式有数据且 <0.35 m 才 WARN。
- I2C 错误计数属 WP-A（SYS-S05）：A 以任何诊断话题发布计数后会被自动聚合
  （前缀 `<topic>: <name>`），本节点不直接读 I2C。
- 若 Nav2/其他节点直接向 `/diagnostics` 发布（默认行为），会造成双发布者，
  违反契约单发布者约定——需在对应 launch 中把其诊断话题 remap 到
  `/nav2/diagnostics` 等名字后由本节点聚合（集成注意项）。
- 纯函数不读真实 `/proc`；`safety_node` 读取失败时对应条目为 WARN 并带
  `n/a` 值，不影响其他条目。
