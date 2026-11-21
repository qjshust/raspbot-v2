#!/usr/bin/env bash
# estop.sh — 发布 /estop true（锁停，SYS-S02 / D-07）。
#
# 锁存语义（CONTRACTS §2）：E 的 cmd_vel_limiter 收到 true 后持续输出零速，
# 直到显式收到 false。恢复用 estop_release.sh。
#
# 用法：
#   bash ros2_ws/scripts/estop.sh           # 等价 estop.sh true
#   bash ros2_ws/scripts/estop.sh false     # 等价 estop_release.sh
set -euo pipefail

# 未 source 环境时自动加载 ROS
command -v ros2 >/dev/null 2>&1 || source /opt/ros/lyrical/setup.bash

VAL="${1:-true}"
case "${VAL}" in
    true|false) ;;
    *) echo "用法: $0 [true|false]" >&2; exit 2 ;;
esac

ros2 topic pub --once /estop std_msgs/msg/Bool "{data: ${VAL}}"
echo "estop ${VAL} sent"
