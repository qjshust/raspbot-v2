# raspbot_description —— 全车 URDF 与静态 TF（WP-E 唯一维护）

> 契约：CONTRACTS.md v1.2 §1/§3。本包是 `base_link → laser / imu_link /
> camera_link / ultrasonic / base_footprint` 全部静态变换的**唯一来源**。
> `odom → base_link` 由 `raspbot_fusion` 的 ekf_node 发布，不在本包。

## 1. 外参初值表（与 `urdf/raspbot.urdf` 头部注释表一致）

坐标约定：ROS REP-103（x 前 / y 左 / z 上），长度米、角度弧度。所有 joint 均为
`fixed`，外参数值直接写在 URDF 各 `<joint>` 的 `<origin>` 中（不使用 xacro）。

| joint | child | x (m) | y (m) | z (m) | roll/pitch/yaw | 状态 |
|---|---|---|---|---|---|---|
| base_footprint_joint | base_footprint | 0.000 | 0.000 | 0.000 | 0 / 0 / 0 | base_link 即地面轴距中点，投影重合（终值） |
| laser_joint | laser | **+0.107** | 0.000 | **+0.034** | 0 / 0 / 0 | J-04 契约值（雷达在车头前/上方），**待 CAR-C05 卷尺复核** |
| imu_joint | imu_link | 0.000 | 0.000 | **+0.080** | 0 / 0 / 0 | 设计 §5.5.5：IMU 约在甲板上方 40 mm、接近几何中心，初值待卷尺复核 |
| camera_joint | camera_link | **+0.077** | 0.000 | **+0.095** | 0 / 0 / 0 | **初值占位**（云台/甲板前缘），pitch=0.0 待 CAR-C05/C06 标定 |
| ultrasonic_joint | ultrasonic | **+0.070** | 0.000 | **+0.030** | 0 / 0 / 0 | 车头超声，初值占位 |

## 2. 卷尺复核方法（对应 CAR-C05）

1. 找 `base_link` 标记点：**轴距中点**（左右轮轴连线的中点）在**地面**的投影
   （mech/README.md §7.4：甲板前缘后方 77 mm）。在甲板/地面上做十字标记。
2. 对每个传感器，卷尺量其**敏感元件中心**（C1 的旋转中心、HWT101 模块中心、
   相机光心、超声探头端面中心）相对标记点的三轴偏移，目标精度 **±2 mm / ±0.5°**
   （mech/README.md §7.4）。
3. 高度量法：传感器安装面到地面减去甲板安装面高度（C1 扫描面在甲板 +34 mm）。
4. 相机 pitch 需要棋盘格标定（CAR-C06）得到后再填入 `camera_joint` 的 rpy。
5. 更新流程：改 `urdf/raspbot.urdf` 头部注释表 + 对应 `<origin>` + 本文档表格，
   重启 `description.launch.py` 即生效（无需重新 colcon build，URDF 是运行时读取
   的数据文件；仅当文件清单变化时才需要 build）。

## 3. TF 树

```
map ──(F: slam_toolbox/amcl)──> odom ──(E: ekf_node)──> base_link
                                                          ├─ base_footprint  (0,0,0)
                                                          ├─ laser           (+0.107, 0, +0.034)
                                                          ├─ imu_link        (0, 0, +0.080)
                                                          ├─ camera_link     (+0.077, 0, +0.095)
                                                          └─ ultrasonic      (+0.070, 0, +0.030)
```

- 每帧唯一父节点（SW-C05 判据）；本包发布的 5 条边全部来自 `robot_state_publisher`。
- **禁止**在其他节点重复发布本表任何变换；**禁止** `static_transform_publisher`
  发布动态变换（CONTRACTS §3）。
- frame 命名固定：`base_link` / `odom` / `map` / `laser` / `imu_link` /
  `camera_link` / `ultrasonic`。

## 4. 启动

```bash
ros2 launch raspbot_description description.launch.py                # 真机
ros2 launch raspbot_description description.launch.py use_sim_time:=true   # bag 回放
```

`use_sim_time` 默认 `false`。URDF 路径由 `get_package_share_directory` 从包内
`share/raspbot_description/urdf/raspbot.urdf` 加载。

## 5. 依赖与说明

- package.xml：`exec_depend` = `robot_state_publisher`、`launch_ros`
  （launch 文件运行时经 launch_ros 传递获得 `ament_index_python`）。
- 本包无节点、无 console_scripts，纯资源文件（URDF + launch）。
