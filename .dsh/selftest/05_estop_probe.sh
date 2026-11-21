#!/usr/bin/env bash
# 05_estop_probe.sh —— 手动探针：estop 锁存/解除行为
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_probe.log 2>&1 &
sleep 22
echo "== 初始 /cmd_vel_safe（应 0）=="
timeout -s KILL 4 ros2 topic echo /cmd_vel_safe --field linear.x --once 2>/dev/null | grep data | sed 's/^/  /'
echo "== 发 estop=true =="
timeout -s KILL 4 ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}" 2>&1 | sed 's/^/  /'
sleep 1
echo "== 发 cmd 0.2（estop 锁存应保持 0）=="
timeout -s KILL 4 ros2 topic pub -r 30 /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 &
P=$!
sleep 1
timeout -s KILL 4 ros2 topic echo /cmd_vel_safe --field linear.x --once 2>/dev/null | grep data | sed 's/^/  /'
kill $P 2>/dev/null
echo "== 发 estop=false =="
timeout -s KILL 4 ros2 topic pub --once /estop std_msgs/msg/Bool "{data: false}" 2>&1 | sed 's/^/  /'
sleep 1
echo "== 再发 cmd 0.2（应恢复输出）=="
timeout -s KILL 4 ros2 topic pub -r 30 /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 &
P=$!
sleep 1.5
timeout -s KILL 4 ros2 topic echo /cmd_vel_safe --field linear.x --once 2>/dev/null | grep data | sed 's/^/  /'
kill $P 2>/dev/null
sleep 0.5
echo "== limiter 日志尾部 =="
grep -iE 'estop|limiter' /tmp/st_probe.log | tail -8 | sed 's/^/  /'
