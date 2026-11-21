#!/usr/bin/env bash
# 06_slam.sh —— 全栈 SLAM 冒烟（SW-I02 增强版，真雷达+真IMU+真底盘，轮子不动）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

cleanup() { bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1

echo "== 启动 bringup（mode=slam）=="
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=slam > /tmp/st_slam.log 2>&1 &
sleep 30

echo "== /scan =="
SCAN_OUT=$(timeout -s KILL 6 ros2 topic hz /scan 2>&1)
echo "$SCAN_OUT" | grep -E 'average rate' | sed 's/^/  /'
SCAN_RATE=$(echo "$SCAN_OUT" | grep 'average rate' | tail -1 | awk '{print $3}')
echo "== /imu/data =="
IMU_OUT=$(timeout -s KILL 6 ros2 topic hz /imu/data 2>&1)
echo "$IMU_OUT" | grep -E 'average rate' | sed 's/^/  /'
IMU_RATE=$(echo "$IMU_OUT" | grep 'average rate' | tail -1 | awk '{print $3}')

echo "== /map =="
timeout -s KILL 8 ros2 topic info /map 2>&1 | grep -E 'Type|Publisher count' | sed 's/^/  /'
MAP_PUBS=$(timeout -s KILL 8 ros2 topic info /map 2>&1 | grep 'Publisher count' | awk '{print $3}')
timeout -s KILL 8 ros2 topic echo /map --once --field info 2>&1 | head -8 | sed 's/^/  /'

echo "== map→base_link TF =="
timeout -s KILL 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -E 'At time|Translation|Invalid' | head -3 | sed 's/^/  /'

echo "== slam 日志 =="
tail -12 /tmp/st_slam.log | sed 's/^/  /'
ODOM_FAILS=$(grep -c 'Failed to compute odom pose' /tmp/st_slam.log 2>/dev/null)
ODOM_FAILS=${ODOM_FAILS:-0}

echo
echo "== 判定 =="
if [ -n "$SCAN_RATE" ] && awk "BEGIN{exit !($SCAN_RATE >= 9 && $SCAN_RATE <= 11)}"; then pass "/scan 10±1 Hz（$SCAN_RATE）"; else fail "/scan 频率 ${SCAN_RATE:-?}"; fi
if [ -n "$IMU_RATE" ] && awk "BEGIN{exit !($IMU_RATE >= 90 && $IMU_RATE <= 110)}"; then pass "/imu/data 100±10 Hz（$IMU_RATE）"; else fail "/imu/data 频率 ${IMU_RATE:-?}"; fi
[ "$MAP_PUBS" = "1" ] && pass "/map 发布者=1" || fail "/map 发布者=${MAP_PUBS:-0}"
[ "$ODOM_FAILS" = "0" ] && pass "Failed to compute odom pose = 0" || fail "odom pose 失败 $ODOM_FAILS 次"

echo
echo "== 保存地图（静态弧线图，作 SIL 素材）=="
MAP_DIR="$HOME/ros2_ws/maps/20261120_self_test_static"
bash "$HOME/tools/slam/save_map.sh" self_test_static 2>&1 | sed 's/^/  /' || fail "地图保存"
ls -l "$MAP_DIR" 2>/dev/null | sed 's/^/  /'

echo
if [ $FAILS -eq 0 ]; then echo "[PASS] 06_slam 全部通过"; else echo "[FAIL] 06_slam 有 $FAILS 项失败"; exit 1; fi
