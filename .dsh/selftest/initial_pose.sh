#!/usr/bin/env bash
# initial_pose.sh —— 反复发布 /initialpose 直到 map→base_link 出现（事件驱动，防丢消息）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
for i in $(seq 1 40); do
  timeout -s KILL 6 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}" >/dev/null 2>&1
  sleep 2
  if python3 "$HOME/tools/selftest/tf_poll.py" map base_link --timeout 6 2>/dev/null | grep -q FOUND; then
    echo "  [OK] initialpose 生效，map→base_link 存在（第 ${i} 轮）"
    exit 0
  fi
done
echo "  [FAIL] initialpose 未能生效（40 轮）"
exit 1
