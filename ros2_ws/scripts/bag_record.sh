#!/usr/bin/env bash
# bag_record.sh — ros2 bag 白名单录制（FR-H-05 / SYS-D06）。
#
# 用法：
#   bash ros2_ws/scripts/bag_record.sh <名称后缀>
#
# 话题白名单见 TOPICS 数组。输出目录：
#   优先  /mnt/ssd/bags/<时间戳>_<后缀>
#   回退  ~/bags/<时间戳>_<后缀>（/mnt/ssd 未挂载或不可写时，打印 WARN）
# 不静默丢数据：回退一定打 WARN（SYS-D06 判据）。
set -euo pipefail

SUFFIX="${1:-run}"
TS="$(date +%Y%m%d_%H%M%S)"
TOPICS=(/scan /imu/data /odom /odom_raw /cmd_vel /cmd_vel_safe /tf /tf_static \
        /range /line_sensor /map /diagnostics /estop)

command -v ros2 >/dev/null 2>&1 || source /opt/ros/lyrical/setup.bash

if mountpoint -q /mnt/ssd && mkdir -p /mnt/ssd/bags 2>/dev/null; then
    OUT="/mnt/ssd/bags/${TS}_${SUFFIX}"
else
    mkdir -p "${HOME}/bags"
    OUT="${HOME}/bags/${TS}_${SUFFIX}"
    echo "WARN: /mnt/ssd 不可用，bag 回退到 ${OUT}（SYS-D06 降级路径）" >&2
fi

echo "recording -> ${OUT}"
echo "topics: ${TOPICS[*]}"
exec ros2 bag record -o "${OUT}" "${TOPICS[@]}"
