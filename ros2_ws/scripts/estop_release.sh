#!/usr/bin/env bash
# estop_release.sh — 发布 /estop false，解除急停锁停（显式确认恢复，SYS-S02）。
set -euo pipefail

command -v ros2 >/dev/null 2>&1 || source /opt/ros/lyrical/setup.bash

ros2 topic pub --once /estop std_msgs/msg/Bool "{data: false}"
echo "estop released"
