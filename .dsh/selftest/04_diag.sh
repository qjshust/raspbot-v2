#!/usr/bin/env bash
# 04_diag.sh —— 诊断 bringup 全栈启动状态
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
pkill -f 'bringup.launch.py' 2>/dev/null
pkill -f 'raspbot_' 2>/dev/null
pkill -f rplidar_node 2>/dev/null
pkill -f component_container 2>/dev/null
pkill -f 'cmd_injector' 2>/dev/null
sleep 2
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none use_camera:=true > /tmp/st_bringup2.log 2>&1 &
sleep 30
echo "== 进程 =="
ps -eo pid,pcpu,args | grep -E 'raspbot|rplidar|v4l2|twist|ekf|safety|robot_state' | grep -v grep | sed 's/^/  /'
echo
echo "== 节点 =="
timeout -s KILL 10 ros2 node list 2>&1 | sed 's/^/  /'
echo
echo "== 话题 =="
timeout -s KILL 10 ros2 topic list 2>&1 | sed 's/^/  /'
echo
echo "== bringup 日志（ERROR/关键行）=="
grep -E 'ERROR|error|Exception|Traceback|not found|No such|died|exited|Failed' /tmp/st_bringup2.log | head -30 | sed 's/^/  /'
echo
echo "== 日志尾部 =="
tail -25 /tmp/st_bringup2.log | sed 's/^/  /'
