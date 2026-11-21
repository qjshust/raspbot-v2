# system/ — 平台工程（WP-H）系统级文件

本目录内容随 `raspbot_bringup` 包一起安装到
`~/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/system/`，
也可直接从源码树使用（脚本均以自身目录定位资源）。

| 文件 | 用途 | 对应验收 |
|---|---|---|
| `twist_mux.yaml` + `twist_mux.launch.py` | 速度汇合：`/cmd_vel_nav`(10) + `/cmd_vel_teleop`(50) → `/cmd_vel` | FR-H-06、SW-I06 |
| `raspbot.service` | systemd 一键 bringup（mode:=nav），崩溃 5 s 内重启，日志落 SSD | FR-H-01、SYS-F01、SYS-R02/R03 |
| `install_systemd.sh` | 幂等安装 systemd unit | SYS-F01 |
| `99-raspbot-vcio.rules` | udev：`/dev/vcio` 归 video 组，修 `vcgencmd` 权限 | D-09 |
| `performance.sh` | CPU governor → performance | D-08、SW-P03 |

## 1. systemd（SYS-F01 / SYS-R02 / SYS-R03）

```bash
# 安装（幂等；首次建议 --start）
bash ~/ros2_ws/src/raspbot_bringup/system/install_systemd.sh --start

# 日常
systemctl status raspbot.service
journalctl -u raspbot.service -n 100 -f
systemctl restart raspbot.service
```

- unit 以 `User=ubuntu` 运行，`ExecStart` 里 `mode:=nav`（上电即可导航，SYS-R03 冷启动
  60 s 预算含 systemd 拉起 + 节点启动）。
- `Environment=ROS_LOG_DIR=/mnt/ssd/roslog`（设计 §5.4.2）：日志落 SSD。`/mnt/ssd`
  不可用时（fstab 为 `nofail`，`ExecStartPre` 的 mkdir 失败被忽略）**systemd 仍会正常
  启动**，ROS 日志自动回退默认目录（`~/.ros/log`），不影响运行。
- `Restart=always; RestartSec=5`：节点崩溃 5 s 内重启（设计 §7.4）。
  注意：SYS-R02 判据“≤5 s 恢复”按重启延迟计算，节点自身启动约需数秒，总恢复时间
  略超 5 s 时属预期，建议与 lead 确认口径。

## 2. udev（D-09：vcgencmd 读不到 /dev/vcio）

```bash
sudo cp ~/ros2_ws/src/raspbot_bringup/system/99-raspbot-vcio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/vcio            # 期望 root:video 0660
vcgencmd get_throttled     # 不再 Permission denied（用户需在 video 组：groups）
```

温度监控本身不依赖 vcgencmd：`safety_node` 已用 sysfs
（`/sys/class/thermal/thermal_zone0/temp`）兜底（D-09 处理项）。

## 3. CPU governor（D-08）

```bash
sudo bash ~/ros2_ws/src/raspbot_bringup/system/performance.sh
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor   # 期望 performance
```

幂等可重复执行；**重启后复位**，SW-P03 等性能基线前必须重新执行（或按脚本内注释
持久化到 rc.local）。

## 4. bag 录放（FR-H-05 / SYS-D06 / SW-I03）

脚本位于 `ros2_ws/scripts/`：

```bash
bash ros2_ws/scripts/bag_record.sh my_run        # 白名单录制
bash ros2_ws/scripts/bag_play.sh /mnt/ssd/bags/xxx --clock   # 回放（可加 --clock）
```

- 白名单：`/scan /imu/data /odom /odom_raw /cmd_vel /cmd_vel_safe /tf /tf_static
  /range /line_sensor /map /diagnostics /estop`。
- 输出优先 `/mnt/ssd/bags/<时间戳>_<后缀>`；`/mnt/ssd` 未挂载或不可写时回退
  `~/bags/<时间戳>_<后缀>` 并打印 WARN（SYS-D06 降级路径，不静默丢数据）。

## 5. 急停现状（D-07）

- 软件急停：`/estop` 已实现（`ros2_ws/scripts/estop.sh` 发布 true / `estop_release.sh`
  发布 false；E 的 `cmd_vel_limiter` 锁存，true 后持续零速直至显式 false）。
  `safety_node` 会把 estop 状态报进 `/diagnostics`。
- 硬件急停：本轮**未提供物理急停按钮**，按测试方案 D-07 处置为**操作要求**——
  实车测试必须两人作业、安全员手握断电开关（电池开关/主电源）。SYS-S02 在物理
  按钮落地前标 ⛔（仅软件路径可测）。

## 6. powerguard 部署（J-10 / SYS-S06）

`tools/powerguard/powerguard.py` 已存在（INA219 6.6 V×15 次优雅关机、6.2 V×5 次
立即关机）。**`tools/powerguard/powerguard.json` 默认 `dry_run=true`，测试前必须
改 false**（TOOL-02），并在测试记录中注明：

```bash
python3 tools/powerguard/powerguard.py --status            # 读一次
python3 tools/powerguard/powerguard.py --simulate 6.5 --dry-run   # 策略演练
# 部署：编辑 tools/powerguard/powerguard.json 将 dry_run 改为 false，
#       再安装 tools/powerguard/powerguard.service（systemctl enable --now）
```

## 7. mock 导航（SW-I04）

`use_mock:=true` 时 bringup 用 `fake_base/fake_imu/fake_scan` 替换硬件驱动，
无 `/range`/`/line_sensor`。mock 模式**没有真车建图能力**：

- `use_mock:=true mode:=slam`：冒烟可用（合成 5×4 m 房间，验证 E/F 链路）。
- `use_mock:=true mode:=nav`：localization 需要一张已保存的静态地图（`map:=` 指向
  真车先前建好并保存的 yaml），否则 AMCL 无图可定位。
- 建议的 mock 导航流程：真车 `mode:=slam` 建图保存到 `/mnt/ssd/maps` →
  之后任意机器 `use_mock:=true mode:=nav map:=/mnt/ssd/maps/<图>.yaml` 空跑 Nav2
  （配合 SW-I05：只看 `/cmd_vel` 输出，不发真车指令）。

## 8. 已知集成注意

- **Nav2 诊断双发布者**：Nav2 节点默认直接发布 `/diagnostics`。按契约
  “一个话题只能有一个发布者”，需要在 `nav2.launch.py`（属 G）里把其诊断话题
  remap 到其他名字（如 `/nav2/diagnostics`），`safety_node` 会自动发现并聚合。
  在 G 交付前，`ros2 topic info /diagnostics` 若显示 >1 publisher 即为该问题。
- I2C 错误计数属 WP-A（SYS-S05）：A 发布任意诊断话题后会被自动聚合进
  `/diagnostics`（条目名前缀话题名）。
- 本 launch 不发布任何静态 TF（禁止 `static_transform_publisher`），静态外参由
  `raspbot_description`（E）的 robot_state_publisher 负责。
