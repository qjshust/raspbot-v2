#!/usr/bin/env bash
# 07_topic_probe.sh —— 目标运行中检查 /plan /local_plan /cmd_vel* 主题与发布者
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
timeout -s KILL 8 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}" >/dev/null 2>&1
for i in $(seq 1 30); do
  python3 "$HOME/tools/selftest/tf_poll.py" map base_link --timeout 8 2>/dev/null | grep -q FOUND && break
  sleep 2
done
bash "$HOME/tools/selftest/activate_nav2.sh" 2>&1 | tail -8 | sed 's/^/  /'
echo "== 发目标（后台）=="
timeout -s KILL 75 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  > /tmp/st_goal.log 2>&1 &
G=$!
sleep 12
echo "== 目标运行中：相关话题与发布者 =="
for t in /plan /local_plan /cmd_vel /cmd_vel_nav_raw /cmd_vel_nav /cmd_vel_safe; do
  echo "-- $t"
  timeout -s KILL 8 ros2 topic info "$t" 2>&1 | grep -E 'Publisher count|Node name' | head -4 | sed 's/^/   /'
done
echo "== /local_plan 内容 =="
timeout -s KILL 8 ros2 topic echo /local_plan --field poses --once 2>/dev/null | head -8 | sed 's/^/   /'
echo "== /plan 内容 =="
timeout -s KILL 8 ros2 topic echo /plan --field poses --once 2>/dev/null | head -8 | sed 's/^/   /'
echo "== /cmd_vel_nav_raw 值 =="
timeout -s KILL 8 ros2 topic echo /cmd_vel_nav_raw --once 2>&1 | grep -A3 linear | sed 's/^/   /'
wait $G 2>/dev/null
tail -4 /tmp/st_goal.log | sed 's/^/  /'
