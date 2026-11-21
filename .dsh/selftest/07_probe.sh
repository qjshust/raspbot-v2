#!/usr/bin/env bash
# 07_probe.sh —— SIL 导航逐环观测：目标 → cmd 链 → odom 移动 → 碰撞状态
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
MAP="/mnt/ssd/maps/sil_mock_room.yaml"
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=nav use_mock:=true map:="$MAP" > /tmp/st_probe.log 2>&1 &
sleep 45
echo "== 等待 amcl active =="
for i in $(seq 1 60); do
  ST=$(timeout -s KILL 8 ros2 lifecycle get /amcl 2>/dev/null | grep -oE 'active \[3\]')
  if [ -n "$ST" ]; then echo "  amcl active（第 ${i} 轮）"; break; fi
  sleep 2
done
echo "== 初始位姿 =="
timeout -s KILL 8 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}" >/dev/null 2>&1
echo "== 等待 map→base_link（AMCL 定位收敛）=="
for i in $(seq 1 30); do
  TF=$(timeout -s KILL 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -cE '^At time' | head -1)
  if [ "${TF:-0}" -ge 1 ]; then echo "  map→base_link 存在（第 ${i} 轮）"; break; fi
  sleep 2
done
echo "== 等待 bt_navigator active =="
for i in $(seq 1 60); do
  ST=$(timeout -s KILL 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -oE 'active \[3\]')
  if [ -n "$ST" ]; then echo "  bt_navigator active（第 ${i} 轮）"; break; fi
  sleep 2
done
echo "== 发目标（0.5, 0.3）并后台采样（每 2 s 一轮）=="
( for i in $(seq 1 15); do
    echo "--- t=$i ---"
    timeout -s KILL 6 ros2 topic echo /odom --field pose.pose.position.x --once 2>/dev/null | grep -oE '^-?[0-9.]+' | head -1 | sed 's/^/  odom_x: /'
    timeout -s KILL 6 ros2 topic echo /cmd_vel_nav --field linear.x --once 2>/dev/null | grep -oE '^-?[0-9.]+' | head -1 | sed 's/^/  cmd_vel_nav: /'
    timeout -s KILL 6 ros2 topic echo /cmd_vel_safe --field linear.x --once 2>/dev/null | grep -oE '^-?[0-9.]+' | head -1 | sed 's/^/  cmd_vel_safe: /'
    sleep 2
  done ) &
SAMPLER=$!
GOAL_OUT=$(timeout -s KILL 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.3, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | tail -8)
echo "$GOAL_OUT" | sed 's/^/  /'
wait $SAMPLER 2>/dev/null
echo "== collision_monitor 状态 =="
timeout -s KILL 6 ros2 topic echo /collision_monitor_state --once 2>/dev/null | grep -E 'action_type|polygon_name' | head -6 | sed 's/^/  /'
echo "== 日志 =="
grep -iE 'collision|STOP|SLOWDOWN|progress|missed|make progress' /tmp/st_probe.log | tail -12 | sed 's/^/  /'
