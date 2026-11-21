# WP-C 雷达 bringup（lidar/）

## 1. 交付内容

| 文件 | 用途 |
|---|---|
| `lidar/rplidar.yaml` | `rplidar_node` 参数（rplidar_ros 2.1.4） |
| `lidar/rear_filter.yaml` | `scan_to_scan_filter_chain` 参数（后向立柱遮挡切除） |
| `launch/lidar.launch.py` | 组合启动；`rear_cut_enabled` / `use_sim_time` / `lidar_params_file` |

## 2. 话题拓扑与契约

```
rear_cut_enabled=true（默认）：
  rplidar_node ──scan_raw──▶ scan_to_scan_filter_chain ──scan──▶ /scan
rear_cut_enabled=false：
  rplidar_node ──scan──▶ /scan（直通，过滤链不启动）
```

- `/scan`：`sensor_msgs/LaserScan`，`frame_id=laser`，10 Hz（FR-C-01），量程 0.2~15 m；
  任一配置下发布者只有 1 个（SW-C04）。
- 原始话题 `/scan_raw` 仅在过滤开启时存在（launch 内 remap，供调试）。

## 3. 裁决依据（J-01 / J-02 / J-08）

| 裁决 | 实现位置 |
|---|---|
| J-01 `frame_id` 必须 `laser` | `rplidar.yaml` 的 `frame_id: laser`（绝不允许 `laser_link`；SW-C03） |
| J-02 驱动用 rplidar_ros 2.1.4 | `package='rplidar_ros', executable='rplidar_node'` |
| J-08 后向遮挡 ±12° → 切除 ≥15° | `rear_filter.yaml`：保留前向 ±165°（330°），切除后向 ±15° |

## 4. 参数表（rplidar_node，全部经 Pi 只读取证）

| 参数 | 值 | 证据 |
|---|---|---|
| `channel_type` | `serial` | 二进制 strings `~/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node` |
| `serial_port` | `/dev/raspbot_lidar` | 同上；udev 固定命名（FR-C-02，硬件报告 §3.3） |
| `serial_baudrate` | `460800` | 同上；C1 出厂 460800 8N1（硬件报告 §5.1） |
| `frame_id` | `laser` | 同上；J-01 |
| `inverted` | `false` | 同上；实测基线（smoke_slam.sh） |
| `flip_x_axis` | `false` | 同上（strings 含 `flip_x_axis`） |
| `angle_compensate` | `true` | 同上；实测基线（硬件报告 §5.1） |
| `scan_mode` | `DenseBoost` | 同上；C1 实测 10.0 Hz/720 点（smoke_slam.sh 已验证） |

> `scan_frequency`/`auto_standby` 在 2.1.4 中存在但本车不设（C1 串口转速由
> `scan_mode` 决定），见 `rplidar.yaml` 注释。
> rplidar_ros 版本证据：`~/ros2_ws/install/rplidar_ros/share/rplidar_ros/package.xml`
> → `<version>2.1.4</version>`。

## 5. 后向滤波（laser_filters 2.3.2）

- 节点：`package='laser_filters', executable='scan_to_scan_filter_chain'`。
- 默认输入/输出话题为 `scan` / `scan_filtered`（上游
  `src/scan_to_scan_filter_chain.cpp`），本 launch 用 remap 接成
  `scan_raw → scan`。
- 参数键格式为 2.3.2 新版**命名对象**格式，证据（Pi 实测文件）：
  `/opt/ros/lyrical/share/laser_filters/examples/angular_filter_example.yaml`：

  ```yaml
  scan_to_scan_filter_chain:
    ros__parameters:
      filter1:
        name: angle
        type: laser_filters/LaserScanAngularBoundsFilter
        params:
          lower_angle: -1.52
          upper_angle: 1.52
  ```

- 过滤器：非 InPlace 版 `laser_filters/LaserScanAngularBoundsFilter`
  （插件注册见 `/opt/ros/lyrical/share/laser_filters/laser_filters_plugins.xml`）。
- 角度：`lower_angle=-2.8797932657906435`、`upper_angle=2.8797932657906435`
  —— 精确 ±165°（任务书 -2.88/2.88 rad 的精确值），保证切除角精确 ≥15°
  （J-08 要求“过滤角取 ≥15°”，±2.88 的四舍五入会差 0.01°）。
- 行为：界外点**置为 `range_max`**（保留 720 点消息形状，自由空间语义，
  不进代价地图）——上游 `LaserScanAngularBoundsFilter` 实现如此；最终以
  SYS-M06 目视验收为准（正前方 180° 无固定短射线、后向扇区不进代价地图）。
- 前提假设：scan 角 0 = 车头正前方（装车朝向已由硬件报告 §5.1 / CAR-B13 验证）。
  若机械外参加 yaw（CAR-C05 TF 标定），需同步调整本文件角度。

## 6. 启动方式

```bash
# 默认（含后向切除）
ros2 launch raspbot_bringup lidar.launch.py
# 直通原始扫描（调试/对比）
ros2 launch raspbot_bringup lidar.launch.py rear_cut_enabled:=false
# 切换量程模板（FR-C-04）
ros2 launch raspbot_bringup lidar.launch.py \
    lidar_params_file:=<share>/raspbot_bringup/lidar/lidar_outdoor.yaml
# bag 回放
ros2 launch raspbot_bringup lidar.launch.py use_sim_time:=true
```

## 7. FR-C-04 量程模板（占位做法，P2）

室内模板 = `rplidar.yaml` 现值。户外模板按文件尾部注释操作：复制为
`lidar_outdoor.yaml`（建议 `scan_mode` 保持 DenseBoost，量程收缩 0.35~8.0 m
放到 WP-F 的 `slam_params.yaml`），经 `lidar_params_file` 切换，不改 launch 与代码。
切换生效验收：`SW-G09`（launch 参数覆盖）+ `SYS-M06`。

## 8. QoS 与已知限制

- 过滤链订阅 `scan` 用 `SensorDataQoS`；发布 `scan_filtered` 用节点默认 QoS
  （Reliable，KeepLast(1000)，上游 `scan_to_scan_filter_chain.cpp` 的
  `create_publisher` 默认值）。这与契约表“/scan SensorData”存在差异：
  **SW-C02 按 J-13 口径“以代码现值为基线，用 `ros2 topic info -v` 实测记录”**，
  是否需要改 QoS 由 lead 裁决（改需动 laser_filters 上游，不建议）。
  （rplidar_node 直通时 /scan 的 QoS 同样以实测记录为准。）
- `rear_cut_enabled=false` 时 `/scan` 含后向 ±12° 立柱短射线（预期现象，
  CAR-B13 口径），导航/建图请保持默认过滤开启。
- 热拔雷达行为（SW-F06）：rplidar_node 不掉线行为由驱动决定，本包未包装；
  复插后 `/scan` 自动恢复。
- 外参 TF（base_link→laser）由 WP-E 的 URDF 维护（CONTRACTS §3），本包不发布 TF。

## 9. 验收用例对应

| 需求 | 用例 |
|---|---|
| FR-C-01（10 Hz、frame_id、量程） | `SW-C01`、`SW-C03` |
| FR-C-02（udev 固定命名） | `CAR-B01`（udev 规则已存在，硬件报告 §3.3） |
| FR-C-03（外参入 URDF） | `CAR-C05`（属 WP-E） |
| FR-C-04（量程模板） | `SYS-M06`、`SW-G09` |
| J-08（后向遮挡） | `SYS-M06`、`CAR-B13` |
