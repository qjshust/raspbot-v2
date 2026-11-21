#!/usr/bin/env bash
# 07_raw_values.sh —— 目标期间连续抓 /cmd_vel_nav_raw 的实际值
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
MAP="/mnt/ssd/maps/sil_mock_room.yaml"
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=nav use_mock:=true map:="$MAP" > /tmp/st_probe.log 2>&1 &
sleep 45
for i in $(seq 1 60); do
  ST=$(timeout -s KILL 8 ros2 lifecycle get /amcl 2>/dev/null | grep -oE 'active \[3\]')
  [ -n "$ST" ] && break
  sleep 2
done
bash "$HOME/tools/selftest/initial_pose.sh" 2>&1 | sed 's/^/  /'
bash "$HOME/tools/selftest/activate_nav2.sh" 2>&1 | tail -6 | sed 's/^/  /'
echo "== 发目标（后台 60 s）+ 每 5 s 抓 raw 值/odom =="
timeout -s KILL 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  > /tmp/goal.log 2>&1 &
G=$!
for i in $(seq 1 8); do
  sleep 4
  RAW=$(timeout -s KILL 4 ros2 topic echo /cmd_vel_nav_raw --field linear.x --once 2>/dev/null | grep -oE '^-?[0-9.]+' | head -1)
  ODOM=$(timeout -s KILL 4 ros2 topic echo /odom --field pose.pose.position.x --once 2>/dev/null | grep -oE '^-?[0-9.]+' | head -1)
  echo "  t+$((i*4))s  raw=${RAW:-<无消息>}  odom_x=${ODOM:-<无消息>}"
done
wait $G 2>/dev/null
echo "== 目标结果 =="
tail -4 /tmp/goal.log | sed 's/^/  /'
echo "== 控制器日志 =="
grep -E 'controller_server' /tmp/st_probe.log | grep -viE 'lifecycle|bond|Creating|Configuring|Activating|Deactivating|Critic|initialized|frequency|getting|Created|Loaded|costmap' | tail -8 | sed 's/^/  /'
