# WP-F · 建图与定位（slam_toolbox + nav2 AMCL）

本目录（`ros2_ws/src/raspbot_bringup/slam/`）属于 WP-F（建图与定位），与 `launch/` 下的
`slam.launch.py`、`localization.launch.py` 配套。契约依据：`ros2_ws/CONTRACTS.md` v1.2。

## 五要素

| # | 要素 | 对应需求/用例 | 实现 |
|---|---|---|---|
| 1 | 在线建图 | FR-F-01 / SYS-M01 | `slam.launch.py` + `slam_params.yaml`（slam_toolbox 2.10.0 online_async，mapping 模式） |
| 2 | 地图保存 | FR-F-02 / SYS-M03 / SYS-D03 | `tools/slam/save_map.sh <名>`（map_saver_cli 1.5.2，pgm+yaml 成对校验） |
| 3 | 地图加载 | FR-F-02 / SYS-M03 | `localization.launch.py map:=...`（map_server 以 Transient Local 发布 /map） |
| 4 | 重定位 | FR-F-03 / SYS-M04 | AMCL（`amcl.yaml`）：给定初始位姿后 10 s 内收敛，误差 <0.1 m |
| 5 | 建图/定位互斥 | FR-F-04 / SW-I06 / SYS-F04 | 模式由启动哪个 launch 唯一决定：建图=slam.launch.py，定位=localization.launch.py；两者不同时启动（同一时刻只有一个 map→odom 发布者） |

## TF 契约（本包绝不重复发布）

| 变换 | 发布者 | 说明 |
|---|---|---|
| `map → odom` | **F**（slam_toolbox 或 amcl） | 本包两个 launch 各自发布其一 |
| `odom → base_link` | E（EKF） | 本包不发布、不占位 |
| `base_link → laser` 等静态外参 | robot_state_publisher（E 维护 URDF） | 本包不发布、不占位 |

> 例外：`tools/slam/smoke_slam.sh` 是脱离系统的诊断工具，内含静态 TF 占位，
> 仅用于单独验证 SLAM 链路，不代表真实外参。

## 建图流程（FR-F-01 / SYS-M01）

1. E/C 节点已运行（`/odom`、`/scan`、`odom→base_link`、静态外参 TF 就绪）；
2. 启动建图（不启动任何 TF）：

   ```bash
   ros2 launch raspbot_bringup slam.launch.py   # autostart=true，use_sim_time 默认 false
   ```

3. 以 0.12–0.15 m/s 遥控绕场一圈（SYS-M01 测量方法见下）；
4. `ros2 topic hz /map` 应约 1 Hz（J-11：map_update_interval 1.0 s），分辨率 0.05 m（J-11）；
5. 保存地图：`bash tools/slam/save_map.sh room`（见 `ros2_ws/maps/README.md`）。

## 加载与定位流程（FR-F-02 / SYS-M03）

```bash
ros2 launch raspbot_bringup localization.launch.py map:=/home/ubuntu/ros2_ws/maps/room.yaml
```

- map 默认 `/home/ubuntu/ros2_ws/maps/room.yaml`，不存在时报错退出；
- nav2 1.5.2 的 `localization_launch.py` 不含 lifecycle manager、amcl/map_server 不支持
  autostart 参数（Pi 实测停在 unconfigured），本包 launch 已补 `nav2_lifecycle_manager`
  自动 configure+activate；
- AMCL 参数见 `amcl.yaml`：`base_link` / `odom` / `map`、`scan`、粒子 200~1000、
  重采样约 1 Hz、`OmniMotionModel`（麦轮，理由见文件头注释）。

## 重定位流程（FR-F-03 / SYS-M04）

1. 启动 `localization.launch.py`（同加载）；
2. 把车放到任意已知位置，在 RViz 用 `2D Pose Estimate` 给定初始位姿
   （或 `ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped ...`，frame_id 必须为 `map`）；
3. 判据：初始偏差 0.3 m / 10° 时，10 s 内收敛，最终误差 <0.1 m；n=5 成功 ≥4；
4. 失败时可调：`amcl.yaml` 的 `recovery_alpha_slow/fast`（默认 0 关闭随机恢复）、
   或调大 `max_particles`（当前 1000）。

## 模式互斥（FR-F-04 / SW-I06 / SYS-F04）

- 建图、定位同一时刻只启动其一（它们都发布 `map→odom`，契约规定唯一发布者）；
- 切换 = 停掉旧 launch、启动新 launch，切换后 5 s 内 `/map` 或 `/amcl_pose` 恢复；
- 建图前无需先停 E/C；定位前必须已有地图文件。

## SYS-M01 测量方法摘要（必须按此执行，否则结论不可比）

1. 地面贴 T 形标记：起点车头位置 + 车体轴线方向各一条胶带；
2. 建图开始前记录 `map→base_link`（或 `/odom`）初值；
3. 匀速绕场一圈（20 m²），**回到起点时车身重新对准 T 形标记**（目视对齐，横向误差 ≤2 cm）；
4. 记录此刻 `map→base_link` 与初值的差（x、y、yaw）——判据：**<0.2 m**；
5. 导出 pgm 检查起点附近墙体是否"双线"（重影）；两者矛盾时以重影为准；
6. 补充质量判据（SYS-M02）：墙厚 ≤2 格（0.1 m）、无悬浮点簇、地面占用率 <5%。

## 冒烟自检（不依赖 E/F，雷达会转、轮子不动）

```bash
bash tools/slam/smoke_slam.sh   # 断言：/scan 10±1 Hz、/imu/data 100±10 Hz、
                                # /map Publisher=1、odom pose 失败=0；末尾 [PASS]/[FAIL] 汇总
```

## 已知限制

- 建图外参（`base_link→laser` 占位 0/0.107/0.034）未实测前，地图不能作为 SYS-M01 验收结果（D-04）；
- `/mnt/ssd` 挂载失败时地图落在 SD 卡 `~/ros2_ws/maps`（冒烟报告遗留问题 #1）；
- nav2_amcl 1.5.2 无独立 update_rate/resample_rate 参数，更新节奏由
  `update_min_d/a`（0.2 m / 0.2 rad）与 `resample_interval`（1）控制（见 amcl.yaml 注释）。
