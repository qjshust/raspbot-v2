#!/usr/bin/env bash
# 07_nav2_sil.sh —— SIL 导航闭环（SW-I04 mock 三件套 + SW-I05 Nav2 静态地图空跑）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

cleanup() { bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1

echo "===== 阶段 1：mock 三件套 + fusion + slam（SW-I04）====="
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=slam use_mock:=true > /tmp/st_mock_slam.log 2>&1 &
sleep 30

echo "== 节点 =="
timeout -s KILL 8 ros2 node list 2>&1 | sed 's/^/  /'
echo "== /scan（合成房间）=="
timeout -s KILL 6 ros2 topic hz /scan 2>&1 | grep 'average rate' | sed 's/^/  /'
echo "== /map =="
MAP_PUBS=$(timeout -s KILL 8 ros2 topic info /map 2>&1 | grep 'Publisher count' | awk '{print $3}')
echo "  /map Publisher count = ${MAP_PUBS:-0}"
timeout -s KILL 8 ros2 topic echo /map --once --field info 2>&1 | head -8 | sed 's/^/  /'
ODOM_FAILS=$(grep -c 'Failed to compute odom pose' /tmp/st_mock_slam.log 2>/dev/null)
ODOM_FAILS=${ODOM_FAILS:-0}
echo "  odom pose 失败次数: $ODOM_FAILS"

[ "$MAP_PUBS" = "1" ] && pass "mock 建图：/map 发布者=1" || fail "mock 建图失败"
[ "$ODOM_FAILS" = "0" ] && pass "mock 建图：odom pose 失败 0" || fail "mock 建图：odom pose 失败 $ODOM_FAILS"

echo
echo "== 保存 mock 地图 =="
bash "$HOME/tools/slam/save_map.sh" sil_mock_room 2>&1 | sed 's/^/  /' || fail "mock 地图保存"
# save_map.sh 平面落盘：SSD 可用时在 /mnt/ssd/maps/<名>.{pgm,yaml}，否则 ~/ros2_ws/maps/
if [ -f "/mnt/ssd/maps/sil_mock_room.yaml" ]; then
  MOCK_MAP="/mnt/ssd/maps/sil_mock_room.yaml"
else
  MOCK_MAP="$HOME/ros2_ws/maps/sil_mock_room.yaml"
fi
[ -f "$MOCK_MAP" ] && pass "mock 地图已保存" || fail "mock 地图文件缺失"
ls -l "$(dirname "$MOCK_MAP")"/sil_mock_room.* 2>/dev/null | sed 's/^/  /'

echo
echo "===== 阶段 2：静态地图 + Nav2（SW-I05）====="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=nav use_mock:=true map:="$MOCK_MAP" > /tmp/st_nav_sil.log 2>&1 &
sleep 40

echo "== 节点 =="
timeout -s KILL 8 ros2 node list 2>&1 | sed 's/^/  /'
echo "== 等待 amcl active =="
for i in $(seq 1 60); do
  ST=$(timeout -s KILL 8 ros2 lifecycle get /amcl 2>/dev/null | grep -oE 'active \[3\]')
  if [ -n "$ST" ]; then echo "  amcl active（第 ${i} 轮）"; break; fi
  sleep 2
done
echo "== 发布初始位姿（反复发布直到 map→base_link 出现，防消息丢失）=="
bash "$HOME/tools/selftest/initial_pose.sh" 2>&1 | sed 's/^/  /'
echo "== 补激活 Nav2 节点（manager 不 autostart）=="
bash "$HOME/tools/selftest/activate_nav2.sh" 2>&1 | sed 's/^/  /'
echo "== /cmd_vel（twist_mux 输出）=="
timeout -s KILL 5 ros2 topic info /cmd_vel 2>&1 | grep -E 'Publisher|Sub' | sed 's/^/  /'

echo "== 等待 bt_navigator active =="
for i in $(seq 1 60); do
  ST=$(timeout -s KILL 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -oE 'active \[3\]')
  if [ -n "$ST" ]; then echo "  bt_navigator active（第 ${i} 轮）"; break; fi
  sleep 2
done
echo "== 发送导航目标（1.5, 0.0, 0°）=="
GOAL_OUT=$(timeout -s KILL 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | tail -6)
echo "$GOAL_OUT" | sed 's/^/  /'
echo "$GOAL_OUT" | grep -q 'SUCCEEDED' && pass "导航目标 SUCCEEDED" || fail "导航目标未成功（见上）"

echo
echo "== 到达后速度归零（SW-N06 前半：任务结束后 /cmd_vel_safe ≤0.05）=="
sleep 2
SAFE_V=$(timeout -s KILL 4 ros2 topic echo /cmd_vel_safe --field linear.x --once 2>/dev/null | grep -oE '^-?[0-9.]+' | head -1)
echo "  /cmd_vel_safe.linear.x = ${SAFE_V:-?}"
if [ -n "$SAFE_V" ] && awk "BEGIN{exit !($SAFE_V <= 0.05)}"; then pass "到达后速度归零（≤0.05）"; else fail "到达后速度未归零"; fi

echo
echo "== Nav2 日志 =="
tail -10 /tmp/st_nav_sil.log | sed 's/^/  /'

echo
if [ $FAILS -eq 0 ]; then echo "[PASS] 07_nav2_sil 全部通过"; else echo "[FAIL] 07_nav2_sil 有 $FAILS 项失败"; exit 1; fi
