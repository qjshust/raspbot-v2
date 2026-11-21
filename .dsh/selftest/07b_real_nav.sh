#!/usr/bin/env bash
# 07b_real_nav.sh —— 硬件在环导航冒烟（真底盘，四轮悬空；可选执行）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

cleanup() {
  timeout -s KILL 4 ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}" 2>/dev/null
  bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1

if [ -f "/mnt/ssd/maps/sil_mock_room.yaml" ]; then
  MAP="/mnt/ssd/maps/sil_mock_room.yaml"
else
  MAP="$HOME/ros2_ws/maps/sil_mock_room.yaml"
fi
[ -f "$MAP" ] || { echo "[BLOCKED] 需要先跑 07_nav2_sil 生成地图（$MAP）"; exit 2; }

echo "== 启动 bringup（mode=nav，真硬件，静态地图）=="
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=nav map:="$MAP" > /tmp/st_nav_real.log 2>&1 &
sleep 35

echo "== 等待 amcl active + 反复发布初始位姿 =="
for i in $(seq 1 60); do
  ST=$(timeout -s KILL 8 ros2 lifecycle get /amcl 2>/dev/null | grep -oE 'active \[3\]')
  if [ -n "$ST" ]; then echo "  amcl active（第 ${i} 轮）"; break; fi
  sleep 2
done
bash "$HOME/tools/selftest/initial_pose.sh" 2>&1 | sed 's/^/  /'
echo "== 补激活 Nav2 节点 =="
bash "$HOME/tools/selftest/activate_nav2.sh" 2>&1 | sed 's/^/  /'

echo "== 发导航目标（1.5, 0.0，注意 ≥ prune_distance 0.5 避免短路径被剪空）=="
GOAL_OUT=$(timeout -s KILL 80 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | tail -6)
echo "$GOAL_OUT" | sed 's/^/  /'

echo "== 过程中 cmd 链采样 =="
timeout -s KILL 5 ros2 topic echo /cmd_vel_safe --field linear --once 2>/dev/null | sed 's/^/  /'

echo "== estop 收尾 =="
timeout -s KILL 4 ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}" 2>&1 | sed 's/^/  /'
SAFE_V=""
for i in 1 2 3; do
  SAFE_V=$(timeout -s KILL 6 ros2 topic echo /cmd_vel_safe --field linear.x --once 2>/dev/null | grep -oE '^-?[0-9.]+' | head -1)
  [ -n "$SAFE_V" ] && break
  sleep 2
done
echo "  estop 后 /cmd_vel_safe.linear.x = ${SAFE_V:-<无消息>}"
if [ -n "$SAFE_V" ] && awk "BEGIN{exit !($SAFE_V <= 0.01)}"; then pass "estop 后速度归零"; else fail "estop 后速度未归零"; fi

echo
echo "== 日志 =="
tail -8 /tmp/st_nav_real.log | sed 's/^/  /'
echo
if [ $FAILS -eq 0 ]; then echo "[PASS] 07b_real_nav 完成（悬空 HIL 冒烟，结果仅作链路证据）"; else echo "[FAIL] 07b_real_nav 有 $FAILS 项失败"; exit 1; fi
