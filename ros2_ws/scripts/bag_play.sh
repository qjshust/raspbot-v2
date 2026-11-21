#!/usr/bin/env bash
# bag_play.sh — ros2 bag play 封装（SW-I03 / FR-H-05）。
#
# 用法：
#   bash ros2_ws/scripts/bag_play.sh <bag目录>
#   bash ros2_ws/scripts/bag_play.sh <bag目录> --clock   # 发布 /clock（配合 use_sim_time）
set -euo pipefail

BAG=""
CLOCK=0
for arg in "$@"; do
    case "${arg}" in
        --clock) CLOCK=1 ;;
        -*) echo "未知选项: ${arg}" >&2; exit 2 ;;
        *) BAG="${arg}" ;;
    esac
done

if [[ -z "${BAG}" ]]; then
    echo "用法: $0 <bag目录> [--clock]" >&2
    exit 2
fi

command -v ros2 >/dev/null 2>&1 || source /opt/ros/lyrical/setup.bash

ARGS=()
if [[ ${CLOCK} -eq 1 ]]; then
    ARGS+=(--clock)
fi
exec ros2 bag play "${BAG}" "${ARGS[@]}"
