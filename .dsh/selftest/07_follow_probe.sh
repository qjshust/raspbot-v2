#!/usr/bin/env bash
# 07_follow_probe.sh —— 直接对 /follow_path 发手工路径，隔离控制器与执行链
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
sleep 8
bash "$HOME/tools/selftest/activate_nav2.sh" 2>&1 | tail -6 | sed 's/^/  /'
echo "== 直接发 /follow_path 手工路径（odom 系 0→1.0m）=="
python3 -u "$HOME/tools/selftest/chain_sampler.py" --seconds 40 > /tmp/sampler.log 2>&1 &
SAMP=$!
sleep 2
GOAL_OUT=$(timeout -s KILL 45 ros2 action send_goal /follow_path nav2_msgs/action/FollowPath \
  "{path: {header: {frame_id: 'odom', stamp: {sec: 0, nanosec: 0}}, poses: [{header: {frame_id: 'odom'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}]}}" \
  --feedback 2>&1 | tail -6)
echo "$GOAL_OUT" | sed 's/^/  /'
wait $SAMP 2>/dev/null
echo "== 采样 =="
cat /tmp/sampler.log | sed 's/^/  /'
