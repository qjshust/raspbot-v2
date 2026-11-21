#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
echo "== /cmd_vel_nav_raw 完整 QoS =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel_nav_raw 2>&1 | sed -n '1,40p' | sed 's/^/  /'
