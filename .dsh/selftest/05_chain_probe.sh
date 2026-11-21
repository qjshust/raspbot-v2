#!/usr/bin/env bash
# 05_chain_probe.sh —— 定位 cmd 链断点：teleop → mux(/cmd_vel) → limiter(/cmd_vel_safe)
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_chain.log 2>&1 &
sleep 22
echo "== 节点 =="
timeout -s KILL 6 ros2 node list 2>&1 | sed 's/^/  /'
echo "== 发 teleop 0.2（持续 5 s）=="
timeout -s KILL 6 ros2 topic pub -r 30 /cmd_vel_teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 &
P=$!
sleep 2.5
echo "== /cmd_vel（mux 输出）=="
timeout -s KILL 5 ros2 topic echo /cmd_vel --once 2>&1 | grep -A3 linear | sed 's/^/  /'
echo "== /cmd_vel_safe（limiter 输出）=="
timeout -s KILL 5 ros2 topic echo /cmd_vel_safe --once 2>&1 | grep -A3 linear | sed 's/^/  /'
echo "== /cmd_vel 发布者 =="
timeout -s KILL 5 ros2 topic info /cmd_vel 2>&1 | grep -E 'Publisher|Sub' | sed 's/^/  /'
echo "== /cmd_vel_safe 发布者 =="
timeout -s KILL 5 ros2 topic info /cmd_vel_safe 2>&1 | grep -E 'Publisher|Sub' | sed 's/^/  /'
kill $P 2>/dev/null
echo "== limiter/mux 日志 =="
grep -iE 'limiter|twist_mux|estop|IMU|输入' /tmp/st_chain.log | tail -12 | sed 's/^/  /'
