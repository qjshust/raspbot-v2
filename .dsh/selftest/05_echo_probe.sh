#!/usr/bin/env bash
# 05_echo_probe.sh —— echo 可用性探针 + ros2 daemon 状态
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_probe.log 2>&1 &
sleep 22
echo "== 停止 ros2 daemon =="
ros2 daemon stop 2>&1 | sed 's/^/  /'
sleep 2
echo "== echo 全量 --once =="
timeout -s KILL 8 ros2 topic echo /cmd_vel_safe --once 2>&1 | head -5 | sed 's/^/  /'
echo "== echo --field linear.x --once =="
timeout -s KILL 8 ros2 topic echo /cmd_vel_safe --field linear.x --once 2>&1 | head -5 | sed 's/^/  /'
