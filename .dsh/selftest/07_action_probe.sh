#!/usr/bin/env bash
# 07_action_probe.sh —— 验证 planner action server 可达性 + bt 激活行为
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
echo "== ros2 action list =="
timeout -s KILL 10 ros2 action list 2>&1 | sed 's/^/  /'
echo "== python action client 探针（wait 30 s）=="
python3 - <<'PYEOF'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
rclpy.init()
n = Node('action_probe')
c = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
ok = c.wait_for_server(30.0)
print(f'  wait_for_server(30s) -> {ok}')
if ok:
    print('  [PASS] python 客户端可连 planner action server')
else:
    print('  [FAIL] python 客户端 30 s 内连不上 action server')
n.destroy_node(); rclpy.shutdown()
PYEOF
echo "== 再试激活 bt_navigator =="
timeout -s KILL 12 ros2 lifecycle set /bt_navigator configure >/dev/null 2>&1
timeout -s KILL 15 ros2 lifecycle set /bt_navigator activate >/dev/null 2>&1
sleep 2
timeout -s KILL 8 ros2 lifecycle get /bt_navigator 2>&1 | sed 's/^/  /'
grep -E 'bt_navigator' /tmp/st_probe.log | tail -5 | sed 's/^/  /'
