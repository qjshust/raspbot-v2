#!/usr/bin/env bash
# 04_diag2.sh —— 定位多发布者来源
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none use_camera:=true > /tmp/st_bringup3.log 2>&1 &
sleep 28
echo "== 进程清单 =="
ps -eo pid,args | grep -E 'raspbot|rplidar|v4l2|twist|ekf|safety|robot_state|filter|relay|limiter' | grep -v grep | awk '{print "  " $1, $2, $3, $4, $5}' | sed 's/^/  /'
echo
echo "== /cmd_vel_safe 发布者详情 =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel_safe 2>&1 | grep -A2 'Publisher' | head -12 | sed 's/^/  /'
echo
echo "== /tf 发布者详情 =="
timeout -s KILL 8 ros2 topic info -v /tf 2>&1 | grep -A2 'Publisher' | head -12 | sed 's/^/  /'
echo
echo "== /diagnostics 发布者详情 =="
timeout -s KILL 8 ros2 topic info -v /diagnostics 2>&1 | grep -A2 'Publisher' | head -16 | sed 's/^/  /'
echo
echo "== /cmd_vel 发布者详情 =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel 2>&1 | grep -A2 'Publisher' | head -12 | sed 's/^/  /'
