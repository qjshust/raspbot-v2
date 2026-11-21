# maps/ — RASPBOT-V2 地图存放与存取流程（WP-F）

## 目录用途

本目录是**开发机**上的地图仓库（用于版本管理、备份、golden 地图归档）。
**运行时的地图在 Pi 上**，路径二选一（由 `tools/slam/save_map.sh` 自动决定）：

| 优先级 | 路径 | 条件 |
|---|---|---|
| 1 | `/mnt/ssd/maps/<地图名>` | `/mnt/ssd/maps` 存在且可写（SSD 挂载正常时） |
| 2 | `~/ros2_ws/maps/<地图名>` | SSD 不可用时的回退（如冒烟报告所述 `/mnt/ssd` 挂载失败的场景） |

Pi 上的地图可以用 `scp ubuntu@192.168.50.98:~/ros2_ws/maps/* .` 拉回本目录归档（若走 SSD 路径同理改前缀）。

## 地图文件形态

每次保存产生**成对的两个文件**，必须一起拷贝/一起删除，否则加载失败：

- `<名>.pgm` — 占用栅格灰度图（P5 二进制 PGM）
- `<名>.yaml` — 地图元数据（`image:` 相对路径、`resolution: 0.05`（裁决 J-11）、`origin`、`negate`、阈值）

## 保存流程（FR-F-02 / SYS-M03 / SYS-D03）

1. 先按 `slam/README.md` 完成建图（slam_toolbox mapping 运行中，`/map` 持续发布）；
2. 在 Pi 上执行：

   ```bash
   bash tools/slam/save_map.sh room        # 自动落到 /mnt/ssd/maps 或 ~/ros2_ws/maps
   ```

3. 脚本会校验 `pgm` + `yaml` 都存在才算成功；
4. 复测要求（SYS-D03 / SYS-M03）：`md5sum <名>.pgm <名>.yaml` 留档，同数据两次保存 md5 应一致；
   保存后重启、重新加载，障碍物位置差 ≤2 格。

## 加载流程（FR-F-02 / SYS-M03）

```bash
# 在 Pi 上（地图默认路径可被 map:= 覆盖）
ros2 launch raspbot_bringup localization.launch.py \
    map:=/home/ubuntu/ros2_ws/maps/room.yaml
# 或加载 SSD 上的地图
ros2 launch raspbot_bringup localization.launch.py \
    map:=/mnt/ssd/maps/room.yaml
```

- 地图文件不存在时 launch 会给出明确报错并退出（见 `localization.launch.py`）；
- 加载后 `/map` 由 map_server 以 **Transient Local** 发布（契约 §2），AMCL 订阅后开始定位并发布 `map → odom`。

## 命名建议

- 每个场地一张地图，用场地名：`room.yaml` / `lab.yaml` / `arena.yaml`；
- 别用日期做后缀做版本，版本靠 git 或 md5 留档；
- `.pgm/.yaml` 成对提交；大图（>2 MB pgm）建议只放 Pi 侧 + 本目录 README 记录。
