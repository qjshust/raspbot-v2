#!/usr/bin/env bash
# 07_mppi_topics.sh —— 目标运行中列出 MPPI/轨迹相关话题并查看内容
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
echo "== 话题 =="
timeout -s KILL 10 ros2 topic list 2>/dev/null | grep -iE 'traj|mppi|local|optimal|marker' | sed 's/^/  /'
echo "== /optimal_trajectory =="
timeout -s KILL 6 ros2 topic echo /optimal_trajectory --once 2>&1 | head -6 | sed 's/^/  /'
echo "== /local_plan =="
timeout -s KILL 6 ros2 topic echo /local_plan --once 2>&1 | head -6 | sed 's/^/  /'
echo "== controller 参数抽查 =="
timeout -s KILL 8 ros2 param get /controller_server FollowPath.motion_model 2>&1 | sed 's/^/  /'
timeout -s KILL 8 ros2 param get /controller_server FollowPath.vx_max 2>&1 | sed 's/^/  /'
