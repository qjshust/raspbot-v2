#!/usr/bin/env bash
# 07_mppi_diag.sh —— 目标运行中抓 MPPI 内部轨迹
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
bash "$HOME/tools/selftest/activate_nav2.sh" 2>&1 | tail -7 | sed 's/^/  /'
echo "== 发目标（后台 60 s）=="
timeout -s KILL 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  > /tmp/goal.log 2>&1 &
G=$!
sleep 15
echo "== /controller_server/optimal_path（姿态数）=="
timeout -s KILL 6 ros2 topic echo /controller_server/optimal_path --once 2>&1 | grep -c 'position:' | sed 's/^/  poses: /'
echo "== /controller_server/optimal_trajectory =="
timeout -s KILL 6 ros2 topic echo /controller_server/optimal_trajectory --once 2>&1 | head -12 | sed 's/^/  /'
echo "== /cmd_vel_nav_raw hz =="
timeout -s KILL 6 ros2 topic hz /cmd_vel_nav_raw 2>&1 | grep -E 'average rate|no new' | sed 's/^/  /'
echo "== /local_costmap/costmap 大小 =="
timeout -s KILL 6 ros2 topic echo /local_costmap/costmap --once --field metadata 2>&1 | head -6 | sed 's/^/  /'
wait $G 2>/dev/null
tail -4 /tmp/goal.log | sed 's/^/  /'
