#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
echo "== cmd 相关主题 =="
timeout -s KILL 8 ros2 topic list 2>/dev/null | grep -E 'cmd' | sed 's/^/  /'
echo "== /cmd_vel 发布者 =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel 2>&1 | grep -E 'Publisher count|Node name|Reliability' | head -8 | sed 's/^/  /'
echo "== /cmd_vel_nav_raw 发布者 =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel_nav_raw 2>&1 | grep -E 'Publisher count|Node name|Reliability' | head -8 | sed 's/^/  /'
echo "== /cmd_vel_nav 发布者 =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel_nav 2>&1 | grep -E 'Publisher count|Node name|Reliability' | head -8 | sed 's/^/  /'
echo "== 直接 echo /cmd_vel（若有）=="
timeout -s KILL 4 ros2 topic echo /cmd_vel --once 2>&1 | grep -A3 linear | sed 's/^/  /'
