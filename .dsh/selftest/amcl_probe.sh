#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
echo "== /tf 发布者 =="
timeout -s KILL 8 ros2 topic info /tf 2>&1 | grep -E 'Publisher|Node name' | sed 's/^/  /'
echo "== tf2_echo map base_link =="
timeout -s KILL 10 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -8 | sed 's/^/  /'
echo "== amcl_pose =="
timeout -s KILL 8 ros2 topic echo /amcl_pose --field pose.pose.position.x --once 2>&1 | head -3 | sed 's/^/  /'
echo "== amcl 日志 =="
grep -iE 'amcl' /tmp/st_nav_sil.log | tail -8 | sed 's/^/  /'
