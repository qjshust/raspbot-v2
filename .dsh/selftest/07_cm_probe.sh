#!/usr/bin/env bash
# 07_cm_probe.sh —— 决定性实验：initialpose 事件驱动 + raw→nav 继电器旁路碰撞监视器
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
echo "== 启动 raw→nav 继电器（旁路碰撞监视器）=="
python3 -u - <<'PYEOF' > /tmp/relay.log 2>&1 &
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init()
n = Node('raw_nav_relay')
p = n.create_publisher(Twist, '/cmd_vel_nav', 10)
def cb(m):
    p.publish(m)
n.create_subscription(Twist, '/cmd_vel_nav_raw', cb, 10)
rclpy.spin(n)
PYEOF
RELAY=$!
sleep 3
echo "== 采样器 + BT 目标（1.5, 0）=="
python3 -u "$HOME/tools/selftest/chain_sampler.py" --seconds 75 > /tmp/sampler.log 2>&1 &
SAMP=$!
sleep 2
GOAL_OUT=$(timeout -s KILL 70 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | tail -6)
echo "$GOAL_OUT" | sed 's/^/  /'
wait $SAMP 2>/dev/null
kill $RELAY 2>/dev/null
echo "== 采样 =="
cat /tmp/sampler.log | sed 's/^/  /'
