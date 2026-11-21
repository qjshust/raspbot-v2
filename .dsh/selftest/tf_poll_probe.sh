#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
echo "== 原样复现轮询命令 =="
timeout -s KILL 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -5
echo "rc=$?"
echo "== 计数 =="
timeout -s KILL 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -cE '^At time' | head -1
