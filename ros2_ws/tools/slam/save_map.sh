#!/usr/bin/env bash
# RASPBOT-V2 · WP-F · 地图保存脚本（FR-F-02 / SYS-M03 / SYS-D03）
#
# 用法（在 Pi 上，需 slam_toolbox mapping 运行中、/map 持续发布）:
#     bash tools/slam/save_map.sh <地图名>
# 例:
#     bash tools/slam/save_map.sh room
#
# 保存路径优先级:
#   1) /mnt/ssd/maps/<名>     （SSD 挂载正常且可写）
#   2) ~/ros2_ws/maps/<名>    （回退；冒烟报告已知 /mnt/ssd 挂载可能失败）
#
# 命令依据（nav2_map_server 1.5.2）:
#   - `map_saver_cli --help` 只列出 -t/-f/--occ/--free/--fmt/--mode；
#   - 1.5.2 的 map_saver_cli 自身创建 MapSaver 节点直接订阅 /map 保存（不依赖 map_saver
#     server 节点），其节点声明参数 save_map_timeout（默认 2.0 s）；
#   - 因此用 -f 指定路径，并在 --ros-args 里把 save_map_timeout 放宽到 8.0 s
#     （map_update_interval 1.0 s 时首帧地图 ~1 s 内可达，8 s 冗余防低负载卡顿）。
#   - map_saver_cli 以 LatchedSubscriptionQoS(3) 订阅 /map，与契约 /map Transient Local 匹配。
#
# 不要在这里开 set -u：ROS 的 setup.bash 会引用未定义变量，source 时直接退出。
LC_ALL=C

. /opt/ros/lyrical/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && . "$HOME/ros2_ws/install/setup.bash"

NAME="${1:-}"
if [ -z "$NAME" ]; then
  echo "用法: bash $0 <地图名>"
  echo "示例: bash $0 room   # 保存为 room.pgm + room.yaml"
  exit 2
fi
case "$NAME" in
  */*|*\\*|*\ *)
    echo "[FAIL] 地图名不能包含路径分隔符或空格: '$NAME'"
    exit 2
    ;;
esac

if [ -d /mnt/ssd/maps ] && [ -w /mnt/ssd/maps ]; then
  DEST="/mnt/ssd/maps/$NAME"
else
  echo "[info] /mnt/ssd/maps 不存在或不可写（SSD 挂载失败?），回退到 ~/ros2_ws/maps"
  DEST="$HOME/ros2_ws/maps/$NAME"
fi

mkdir -p "$(dirname "$DEST")" || { echo "[FAIL] 无法创建目录 $(dirname "$DEST")"; exit 1; }

echo "== 保存地图到 $DEST =="
echo "   若长时间无输出，请确认 slam_toolbox mapping 正在运行且 /map 有发布者。"
timeout -s KILL 90 ros2 run nav2_map_server map_saver_cli \
  -f "$DEST" --ros-args -p save_map_timeout:=8.0
RC=$?
if [ $RC -ne 0 ]; then
  echo "[FAIL] map_saver_cli 退出码 $RC（检查 /map 是否有发布者、路径是否可写）"
  exit 1
fi

echo
echo "== 校验产物（pgm + yaml 必须成对存在） =="
if ls -l "$DEST.pgm" "$DEST.yaml" >/dev/null 2>&1; then
  ls -l "$DEST.pgm" "$DEST.yaml"
  echo "[PASS] 地图已保存: $DEST.pgm / $DEST.yaml"
  echo "      归档 md5（SYS-D03 要求）:"
  md5sum "$DEST.pgm" "$DEST.yaml"
else
  echo "[FAIL] 产物缺失，请检查 map_saver_cli 输出"
  exit 1
fi
