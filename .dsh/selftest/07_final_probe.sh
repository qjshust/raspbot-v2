#!/usr/bin/env bash
# 07_final_probe.sh —— 目标期间 /cmd_vel_nav_raw 发布者与 QoS 详查
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
timeout -s KILL 55 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  > /tmp/goal.log 2>&1 &
G=$!
sleep 12
echo "== /cmd_vel_nav_raw 发布者与 QoS =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel_nav_raw 2>&1 | grep -E 'Publisher count|Node name|QoS profile|Reliability|Durability' | head -12 | sed 's/^/  /'
echo "== /cmd_vel_nav 发布者与 QoS =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel_nav 2>&1 | grep -E 'Publisher count|Node name|QoS profile|Reliability|Durability' | head -12 | sed 's/^/  /'
echo "== /cmd_vel 发布者与订阅 QoS =="
timeout -s KILL 8 ros2 topic info -v /cmd_vel 2>&1 | grep -E 'Publisher count|Node name|QoS profile|Reliability|Durability' | head -12 | sed 's/^/  /'
wait $G 2>/dev/null
tail -4 /tmp/goal.log | sed 's/^/  /'
