#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
for t in /cmd_vel_nav /cmd_vel /cmd_vel_safe; do
  echo "== $t =="
  timeout -s KILL 8 ros2 topic info -v "$t" 2>&1 | grep -E '^  Type:|Topic type:' | head -6 | sed 's/^/  /'
done
