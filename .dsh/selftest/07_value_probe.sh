#!/usr/bin/env bash
# 07_value_probe.sh —— 对照实验：CLI echo 与 python 采样器同测 + 已知值自检
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

echo "== A. 采样器自检：CLI 发 0.2 到 /cmd_vel_safe，采样器应显示 0.200 =="
python3 -u "$HOME/tools/selftest/chain_sampler.py" --seconds 8 > /tmp/sampler.log 2>&1 &
SAMP=$!
sleep 2
timeout -s KILL 4 ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
wait $SAMP 2>/dev/null
cat /tmp/sampler.log | sed 's/^/  /'

echo "== B. 发目标（后台 60 s），10 s 后 CLI echo raw 值（对照采样器）=="
timeout -s KILL 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  > /tmp/goal.log 2>&1 &
G=$!
python3 -u "$HOME/tools/selftest/chain_sampler.py" --seconds 30 > /tmp/sampler2.log 2>&1 &
SAMP2=$!
sleep 12
echo "-- ros2 topic echo /cmd_vel_nav_raw --once:"
timeout -s KILL 6 ros2 topic echo /cmd_vel_nav_raw --once 2>&1 | grep -A3 'linear:' | sed 's/^/   /'
echo "-- ros2 topic echo /cmd_vel_safe --once:"
timeout -s KILL 6 ros2 topic echo /cmd_vel_safe --once 2>&1 | grep -A3 'linear:' | sed 's/^/   /'
wait $SAMP2 2>/dev/null
echo "-- 采样器（30 s）："
cat /tmp/sampler2.log | sed 's/^/  /'
wait $G 2>/dev/null
tail -4 /tmp/goal.log | sed 's/^/  /'
