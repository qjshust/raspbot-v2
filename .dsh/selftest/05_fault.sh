#!/usr/bin/env bash
# 05_fault.sh —— 故障注入（SW-F01 / SYS-S02 estop / SYS-S03 IMU 掉线 / SW-F06 雷达）
# 安全前提：四轮悬空。全程不落地。
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

cleanup() { bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1

echo "== 启动 bringup（mode=none）=="
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_bringup.log 2>&1 &
sleep 22

echo
echo "== SW-F01 超时停车（0.2 m/s × 2 s → 停发）=="
python3 "$HOME/tools/selftest/swf01_timeout.py" 2>&1 | sed 's/^/  /'
[ ${PIPESTATUS[0]} -eq 0 ] && pass "SW-F01 超时停车" || fail "SW-F01 超时停车"

echo
echo "== SYS-S02 estop 锁存 =="
python3 "$HOME/tools/selftest/estop_check.py" 2>&1 | sed 's/^/  /'
[ ${PIPESTATUS[0]} -eq 0 ] && pass "estop 锁存/恢复" || fail "estop 锁存/恢复"

echo
echo "== SYS-S03 IMU 掉线降级（kill imu_node）=="
IMU_PID=$(pgrep -f imu_node | head -1)
if [ -n "$IMU_PID" ]; then
  kill "$IMU_PID"
  sleep 3
  python3 "$HOME/tools/selftest/degrade_check.py" 2>&1 | sed 's/^/  /'
  [ ${PIPESTATUS[0]} -eq 0 ] && pass "IMU 掉线降级（≤0.15 m/s）" || fail "IMU 掉线降级"
else
  fail "找不到 imu_node"
fi

echo
echo "== /diagnostics 中 IMU 告警 =="
timeout -s KILL 6 ros2 topic echo /diagnostics --once 2>/dev/null | grep -E 'imu|IMU|stale|陈旧|失联' | head -4 | sed 's/^/  /'

echo
echo "== SW-F06 雷达掉线（kill rplidar_node）=="
LIDAR_PID=$(pgrep -f rplidar_node | head -1)
if [ -n "$LIDAR_PID" ]; then
  kill "$LIDAR_PID"
  sleep 2
  if pgrep -f rplidar_node >/dev/null; then fail "rplidar_node 未退出"; else pass "rplidar_node 干净退出"; fi
  echo "  /scan 停止（3 s 窗口应无消息）："
  timeout -s KILL 3 ros2 topic hz /scan 2>&1 | grep -E 'average rate|no new messages' | sed 's/^/    /'
else
  fail "找不到 rplidar_node"
fi

echo
echo "== SW-F02 SIGTERM 优雅退出（单独起 base_node）=="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 run raspbot_base base_node > /tmp/st_base_solo.log 2>&1 &
SOLO=$!
sleep 5
kill -TERM $SOLO
wait $SOLO
RC=$?
if [ $RC -eq 0 ]; then pass "SIGTERM 退出码 0"; else fail "SIGTERM 退出码 $RC"; fi
grep -iE 'stop|停车|shutdown' /tmp/st_base_solo.log | tail -2 | sed 's/^/  /'
if grep -qiE 'stop|停车' /tmp/st_base_solo.log; then pass "日志有 stop 记录"; else fail "日志无 stop 记录"; fi

echo
echo "== 结论 =="
if [ $FAILS -eq 0 ]; then echo "[PASS] 05_fault 全部通过"; else echo "[FAIL] 05_fault 有 $FAILS 项失败"; exit 1; fi
