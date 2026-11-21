#!/usr/bin/env bash
# rate_probe2.sh —— 起 bringup → 稳定 20s → 30s 长窗口测 /imu/data /odom + CPU
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_rate.log 2>&1 &
sleep 25
echo "== /imu/data 30s =="
timeout -s KILL 30 ros2 topic hz /imu/data 2>&1 | grep -E 'average rate' | tail -2 | sed 's/^/  /'
echo "== /odom 30s =="
timeout -s KILL 30 ros2 topic hz /odom 2>&1 | grep -E 'average rate' | tail -2 | sed 's/^/  /'
echo "== /odom_raw 30s =="
timeout -s KILL 30 ros2 topic hz /odom_raw 2>&1 | grep -E 'average rate' | tail -2 | sed 's/^/  /'
echo "== 相关进程 CPU =="
ps -eo pid,pcpu,comm --sort=-pcpu | grep -E 'imu_node|ekf_node|base_node|cmd_vel' | grep -v grep | head -5 | sed 's/^/  /'
echo "== imu 日志尾 =="
grep 'imu_node' /tmp/st_rate.log | tail -3 | sed 's/^/  /'
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
echo "[DONE]"
