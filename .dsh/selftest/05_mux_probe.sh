#!/usr/bin/env bash
# 05_mux_probe.sh —— twist_mux 独立转发测试
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 run twist_mux twist_mux --ros-args \
  --params-file "$HOME/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/system/twist_mux.yaml" \
  -r /cmd_vel_out:=/cmd_vel > /tmp/st_mux.log 2>&1 &
sleep 4
echo "== 发布 teleop 0.2 ×3 s =="
timeout -s KILL 3 ros2 topic pub -r 20 /cmd_vel_teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 &
P=$!
sleep 1
echo "== /cmd_vel hz =="
timeout -s KILL 3 ros2 topic hz /cmd_vel 2>&1 | grep -E 'average rate|no new' | sed 's/^/  /'
echo "== /cmd_vel 值 =="
timeout -s KILL 4 ros2 topic echo /cmd_vel --once 2>&1 | grep -A3 linear | sed 's/^/  /'
kill $P 2>/dev/null
echo "== mux 日志 =="
cat /tmp/st_mux.log | sed 's/^/  /'
echo "== mux 参数 =="
timeout -s KILL 6 ros2 param dump /twist_mux 2>/dev/null | head -30 | sed 's/^/  /'
