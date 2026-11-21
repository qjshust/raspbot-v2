#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
echo "== /cmd_vel_safe 全部发布者 =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel_safe 2>&1 | grep -E 'Node name|Publisher count' | sed 's/^/  /'
echo
echo "== /diagnostics 全部发布者 =="
timeout -s KILL 8 ros2 topic info -v /diagnostics 2>&1 | grep -E 'Node name|Publisher count' | sed 's/^/  /'
echo
echo "== /tf 全部发布者 =="
timeout -s KILL 8 ros2 topic info -v /tf 2>&1 | grep -E 'Node name|Publisher count' | sed 's/^/  /'
