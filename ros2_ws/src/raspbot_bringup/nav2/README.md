# WP-G 导航与避障（Nav2 1.5.2）

> 文件：`raspbot_bringup/nav2/nav2_params.yaml`、`raspbot_bringup/launch/nav2.launch.py`
> 属主：WP-G。契约路径见 CONTRACTS v1.2 §1/§4。
> 基线：Nav2 1.5.2-2resolute（Pi：`/opt/ros/lyrical/share/nav2_bringup`）。

## 1. 发布 / 订阅

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel_nav_raw` | geometry_msgs/Twist | 发（controller_server、behavior_server） | Nav2 内部未过滤指令；仅 collision_monitor 订阅，**非契约话题** |
| `/cmd_vel_nav` | geometry_msgs/Twist | 发（collision_monitor，单发布者） | **契约话题**：Nav2 最终输出 → H 的 twist_mux |
| `/collision_monitor_state` | nav2_msgs/CollisionMonitorState | 发（collision_monitor） | 触发状态（供 H/监控） |
| `/nav2/diagnostics` | diagnostic_msgs/DiagnosticArray | 发（Nav2 各节点） | Nav2 诊断（已从 /diagnostics remap 走，契约 /diagnostics 单发布者属 H） |
| `/scan` | sensor_msgs/LaserScan | 订（local_costmap、collision_monitor） | C 提供，10 Hz |
| `/odom` | nav_msgs/Odometry | 订（controller_server、bt_navigator） | E 的 EKF 提供，30 Hz |
| `/map` | nav_msgs/OccupancyGrid | 订（global_costmap static_layer） | F 提供，Transient Local |
| `/tf`、`/tf_static` | — | 订 | E/F 提供（唯一发布者，本包不广播任何 TF） |
| `navigate_to_pose`（action） | nav2_msgs/action/NavigateToPose | 服务 | bt_navigator 动作服务器 |
| `/local_costmap/costmap`、`/global_costmap/costmap` | nav_msgs/OccupancyGrid | 发 | 可视化（rviz 可选） |

**不发布**：`/cmd_vel`（属 twist_mux）、`/diagnostics`（属 H）、任何 TF（属 E/F）。

## 2. 参数说明（nav2_params.yaml 关键项）

| 段 | 关键参数 | 值 | 依据 |
|---|---|---|---|
| bt_navigator | default_nav_to_pose/through_poses_bt_xml | `$(find-pkg-share nav2_bt_navigator)/behavior_trees/...` **全路径** | 官方默认行为树；**裸文件名会被 1.5.2 当 XML 内容解析导致启动失败（集成实测）** |
| controller_server | controller_frequency | 20.0 Hz | J-06 / NFR-01 |
| controller_server/general_goal_checker | xy_goal_tolerance / yaw_goal_tolerance | 0.15 m / 0.3 rad | 任务指定（SYS-N02 到达精度） |
| controller_server/FollowPath | plugin | nav2_mppi_controller::MPPIController | **弃 DWB 用 MPPI-Omni**（集成实测 DWB 零速 + 官方默认 MPPI + Omni 匹配麦轮） |
| FollowPath | motion_model / 插件 | `"Omni"` / mppi::OmniMotionModel | 块名须与 motion_model 值一致；libmppi_motion_models.so strings 已核实 |
| FollowPath | vx_max / vx_min / vy_max / wz_max | 0.25 / −0.15 / 0.25 / 1.0 | 任务指定（≤0.30 m/s 实车限速）；1.5.2 无 vy_min 键，vy 按 ±vy_max 对称 |
| FollowPath | ax_max/min、ay_max/min、az_max | ±0.6、±0.6、2.0 | 与 E 的 limiter 一致 |
| FollowPath | batch_size / time_steps / model_dt | 500 / 40 / 0.05 | Pi 4 核降载；2.0 s 时域；model_dt 必须 == 控制周期（20 Hz） |
| controller_server | min_y_velocity_threshold | 0.001（官方 0.5） | Omni 需要 vy 反馈；0.5 是差速车噪声门限，会把 vy 清零导致横移被钳在 ~0.03 m/s |
| controller_server / behavior_server / collision_monitor | enable_stamped_cmd_vel | **false**（1.5.2 默认 true） | 默认 true → cmd 输出 TwistStamped，契约订阅方全是 Twist → DDS 类型不匹配静默丢弃；**注意键名是 enable_stamped_cmd_vel，不是 twist_mux 的 use_stamped** |
| planner_server/GridBased | plugin | nav2_navfn_planner::NavfnPlanner | 官方默认 NavFn（20 m² 小场景、CPU 低；不用 Smac） |
| planner_server/GridBased | allow_unknown / use_final_approach_orientation | false / false | 任务指定 |
| behavior_server | behavior_plugins | spin / backup / wait | 官方名（recoveries_server 为旧版名） |
| behavior_server | cycle_frequency | 20.0 Hz | 任务指定（官方默认 10） |
| local_costmap | rolling/width×height/resolution/update | true / 4×4 m / 0.05 / 5 Hz | 任务指定 + J-11 |
| global_costmap | update / resolution / track_unknown_space | 1 Hz / 0.05 / false | J-11 / 任务指定 |
| 两代价图 | footprint | `[[0.119,0.09],[0.119,-0.09],[-0.119,-0.09],[-0.119,0.09]]` | J-05：238×180 mm 外廓含前伸，x 前向 |
| inflation_layer | inflation_radius / cost_scaling_factor | 0.30 / 2.0 | FR-G-02 / SYS-N03：障碍前 0.3 m 停 |
| obstacle_layer scan | topic / sensor_frame / 高度 | /scan / laser / 0.05–0.3 m | 契约 + J-01 |
| obstacle_layer scan | obstacle_max_range / raytrace_max_range | 2.5 / 3.0 | 任务值（1.5.2 官方键名） |
| collision_monitor | cmd_vel_in/out | cmd_vel_nav_raw → cmd_vel_nav | 契约输出单发布者 |
| collision_monitor | base_frame_id / transform_tolerance / source_timeout | base_link / 0.5 / 1.0 s | 任务指定；断流 1 s 强制停（SYS-S04 兜底） |
| PolygonSlow / PolygonStop | 前边界 / action / ratio | 0.72 m（slowdown, ×0.2）/ 0.42 m（stop） | = 距最前缘 0.6 m 降速 / 0.3 m 急停（J-05 起算） |

完整键值以 `nav2_params.yaml` 内注释为准（每个非官方默认值都注明了来源）。

## 3. 启动命令

```bash
# 前置：F 的定位（map_server+amcl 或 slam_toolbox）、E 的 EKF/TF、C 的 /scan 已在运行
ros2 launch raspbot_bringup nav2.launch.py
# 或显式参数
ros2 launch raspbot_bringup nav2.launch.py use_sim_time:=false autostart:=true \
    params_file:=/path/to/nav2_params.yaml

# 验证：全部 5 个节点 active
ros2 lifecycle get /controller_server /planner_server /behavior_server \
    /bt_navigator /collision_monitor
# 话题拓扑（/cmd_vel_nav 发布者数必须为 1）
ros2 topic info -v /cmd_vel_nav
# 发一个目标（rviz 或命令行）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

rviz 可选，本 launch 不启动；需要时另开：
`ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup --share)/rviz/nav2_default_view.rviz`

## 4. 与验收用例的对应

| 用例 | 落实位置 |
|---|---|
| FR-G-01 定点导航 / SYS-N01 | bt_navigator（官方重规划+恢复行为树）+ NavFn + MPPI-Omni |
| FR-G-02 停障 0.3 m / SYS-N03 | inflation_radius=0.30 + cost_scaling_factor=2.0 + J-05 足印（从最前缘起算）；collision_monitor PolygonStop（0.42 m 前边界）冗余兜底 |
| FR-G-03 恢复行为 / SYS-N05 | behavior_server spin/backup/wait + 官方行为树 Recovery 子树 |
| FR-G-04 不依赖硬件 / SW-I04/I05 | 本包只订阅契约话题（/scan /odom /map /tf），mock/bag 可直接驱动；SW-I05 空跑用上述 action 发目标观察 /cmd_vel_nav |
| SYS-N06 取消/重规划 | 官方行为树 replanning 逻辑（默认） |
| NFR-01 / J-06 Nav2 控制 20 Hz | controller_frequency=20.0；抖动 p99<10% 用 SYS-T03 实测 |
| J-05 足印 | 两代价图 footprint（238×180 外廓，x 前向） |
| J-11 地图 0.05 m | 两代价图 resolution=0.05、全局 update 1 Hz |
| SYS-S04 雷达掉线退出导航 | collision_monitor source_timeout=1.0 s → 强制 STOP；controller costmap_update_timeout=0.3 s → 控制失败退出 |

## 5. 离线测试方法（不占真车）

1. **参数/launch 静态验证**（开发机）：
   `python -m py_compile launch/nav2.launch.py`；yaml 用 PyYAML 解析。
2. **SIL 集成（SW-I04/I05）**：H 的 `fake_base`/`fake_imu`/`fake_scan` + F 的静态地图
   （`map_server` 加载测试地图 + 占位 `map→odom` 静态 TF 仅限台架）→ 启动本 launch →
   `ros2 action send_goal` 发目标，观察：规划成功、`/cmd_vel_nav` 有输出且无 NaN、
   取消后速度归零。
3. **collision_monitor 台架验证（S0 悬空，落地前必做）**：
   看 `/collision_monitor_state` 与 `polygon_slowdown/polygon_stop` 可视化——
   静止时**不得**有 scan 点落入 PolygonStop（防自遮挡误停）；用手/挡板伸入车前
   0.3 m，确认 state 依次 SLOWDOWN → STOP。

## 6. 已知限制

1. **弃 DWB、用 MPPI-Omni 的原因**：集成实测 DWB 在本车只加载 3/7 个 critics、
   输出恒为零速（"Failed to make progress"）；Nav2 1.5.2 官方默认控制器即 MPPI，
   且 MPPI 的 Omni 运动模型与四轮麦轮匹配（直接输出 vy 横移分量）。
   FollowPath 段按官方 MPPI 全套照抄后仅改运动限幅/批大小/时域/PreferForward。
2. **无编码器对 MPPI 的影响**：本车 /odom 为指令速度开环积分；MPPI 用它做
   状态反馈与加速度钳制，标定偏差会体现在 ax/ay/az 生效上。ax/ay（±0.6）、
   az（2.0）、vx_std/vy_std/wz_std、temperature/gamma 均按车估，需在
   CAR-C01/C02 标定后用 CAR-M 与 SYS-N01 实测微调。
3. **限速 0.25 m/s**：按实车 ≤0.30 m/s 规定留 20% 余量；vy 对称 ±0.25。
4. **低速下限**：底盘 PWM 死区（J-12）决定 <0.05 m/s 的爬行不承诺精确速度，
   MPPI 无最小速度参数，目标附近的小速度指令精度同样受限。
5. **collision_monitor 误停风险**：PolygonStop 依赖"正前方 180° 无固定短射线"
   （CAR-B13/J-08 前置）；若台架发现自遮挡误停，把 PolygonStop.enabled 置 false
   或提高 min_points（降速层仍生效，停障由代价图兜底）。
6. **代价图不覆盖的高度**：obstacle 层只取 0.05–0.3 m 高度内的扫描点；
   <5 cm 或悬空障碍不可见（R-02，超声不在 Nav2 链路内）。
7. **无 smoother**：1.5.2 默认行为树不含路径平滑（controller 经 FollowPath action
   直接收 NavFn 路径）；20 m² 场景无碍，若实车出现路径抖动再评估启用。
8. **生命周期管理器自管**：1.5.2 的 navigation_launch.py 不带 lifecycle_manager
   （bringup_launch.py 才有），本 launch 自启并只管理 5 个节点；include 顺带启动的
   smoother/route/waypoint/docking/following 节点保持 unconfigured（空转，无害）。
9. **BT XML 必须带 `$(find-pkg-share)` 全路径**：`default_nav_to_pose_bt_xml` /
   `default_nav_through_poses_bt_xml` 若写裸文件名，Nav2 1.5.2 会将其当作 XML
   内容解析（报 Could not parse / Failed to extract ID，目标被拒）；必须写成
   `$(find-pkg-share nav2_bt_navigator)/behavior_trees/<文件>.xml`（参数文件内由
   bt_navigator 内部解析，与 `bt_search_directories` 同款写法）。集成实测已确认。
10. **cmd 消息类型坑（Twist vs TwistStamped）**：Nav2 1.5.2 的 TwistPublisher 用
    `enable_stamped_cmd_vel` 参数（默认 **true** → 发 TwistStamped；twist_mux 的
    `use_stamped` 是另一个组件的参数名，**不要套用到 nav2**——集成实测 use_stamped 键
    无效）。契约订阅方全是 Twist，类型不匹配时 DDS 静默丢弃、无任何报错
    （表象：控制器 20 Hz "Publishing velocity" 但订阅者收不到）。
    已对 controller_server / behavior_server / collision_monitor 三段显式置 false。
11. **remap 依赖 launch 实现细节**：`cmd_vel→cmd_vel_nav_raw` 依赖 GroupAction+SetRemap
   先于节点自身 remappings 生效（launch_ros 源码已核实）；M3 集成窗口首轮务必先
   `ros2 topic info /cmd_vel_nav` 确认发布者=1 且为 collision_monitor（SW-C04）。
