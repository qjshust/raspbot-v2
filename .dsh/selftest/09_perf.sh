#!/usr/bin/env bash
# 09_perf.sh —— 性能与资源（SW-P01 部分 / SW-P03 / SW-P04 / SW-P07）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

cleanup() { bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1

T0=$(date +%s)
echo "== 启动 bringup（mode=none）=="
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_perf.log 2>&1 &
while [ $(( $(date +%s) - T0 )) -lt 40 ]; do
  if timeout -s KILL 4 ros2 topic list 2>/dev/null | grep -q '/odom'; then break; fi
  sleep 2
done
sleep 5
echo "  冷启动到 /odom 就绪耗时 ≈ $(( $(date +%s) - T0 )) s"

echo
echo "== SW-P03 周期抖动（/odom_raw，1000 样本）=="
python3 "$HOME/tools/selftest/jitter_check.py" 2>&1 | sed 's/^/  /'
[ ${PIPESTATUS[0]} -eq 0 ] && pass "SW-P03 抖动" || fail "SW-P03 抖动"

echo
echo "== SW-P04 端到端时延（5 Hz 方波，30 s）=="
python3 "$HOME/tools/selftest/e2e_latency.py" 2>&1 | sed 's/^/  /'
[ ${PIPESTATUS[0]} -eq 0 ] && pass "SW-P04 时延" || fail "SW-P04 时延"

echo
echo "== SW-P01 资源占用（各节点 CPU/RSS）=="
ps -eo pid,rss,pcpu,comm,args --sort=-rss | grep -E 'raspbot|rplidar|component_container|twist_mux|robot_state|v4l2|ekf|safety' \
  | grep -v grep | awk '{printf "  pid=%s rss=%.0fMB cpu=%s%% %s\n", $1, $2/1024, $3, $5}' | sed 's/^/  /'
TOTAL_RSS=$(ps -eo rss,args | grep -E 'raspbot|rplidar|component_container|twist_mux|robot_state|ekf|safety' | grep -v grep \
  | awk '{s+=$1} END {printf "%.0f", s/1024}')
echo "  节点 RSS 合计 ≈ ${TOTAL_RSS:-0} MB（判据：整机 <3 GB）"
[ "${TOTAL_RSS:-0}" -lt 3072 ] && pass "内存 <3 GB（节点合计 ${TOTAL_RSS:-0} MB）" || fail "内存超标"
CPU_TOTAL=$(ps -eo pcpu,args | grep -E 'raspbot|rplidar|component_container|twist_mux|robot_state|ekf|safety' | grep -v grep \
  | awk '{s+=$1} END {printf "%.0f", s}')
echo "  节点 CPU 合计 ≈ ${CPU_TOTAL:-0}%（4 核 → 判据均值 <280%）"
[ "${CPU_TOTAL:-0}" -lt 280 ] && pass "CPU <280%（合计 ${CPU_TOTAL:-0}%）" || fail "CPU 超标 ${CPU_TOTAL:-0}%"

echo
echo "== 温度 =="
echo "  SoC: $(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 )) C"

echo
if [ $FAILS -eq 0 ]; then echo "[PASS] 09_perf 全部通过"; else echo "[FAIL] 09_perf 有 $FAILS 项失败"; exit 1; fi
