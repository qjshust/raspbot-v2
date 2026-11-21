#!/usr/bin/env bash
# 07_chain_probe.sh —— 目标驱动下的 cmd 链逐环采样
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
bash "$HOME/tools/selftest/activate_nav2.sh" 2>&1 | sed 's/^/  /'
echo "== 启动采样器 + 发目标 =="
python3 "$HOME/tools/selftest/chain_sampler.py" --seconds 75 2>&1 | sed 's/^/  /' &
SAMP=$!
sleep 3
GOAL_OUT=$(timeout -s KILL 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.6, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | tail -6)
echo "$GOAL_OUT" | sed 's/^/  /'
wait $SAMP 2>/dev/null
echo "== 相关日志 =="
grep -iE 'collision|STOP|SLOWDOWN|missed rate|progress|DWB|critic|NaN|nan' /tmp/st_probe.log | tail -15 | sed 's/^/  /'
