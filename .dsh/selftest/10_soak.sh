#!/usr/bin/env bash
# 10_soak.sh —— 短时浸泡（SYS-R01 的部分执行：15 min 采样，替代完整 2 h）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"

cleanup() { bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1

echo "== 启动 bringup（mode=none）=="
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_soak.log 2>&1 &
sleep 25

echo "time,rss_mb,cpu_pct,soc_c,i2c_errors"
for i in $(seq 1 30); do
  RSS=$(ps -eo rss,args | grep -E 'raspbot|rplidar|component_container|twist_mux|robot_state|ekf|safety' | grep -v grep | awk '{s+=$1} END {printf "%.0f", s/1024}')
  CPU=$(ps -eo pcpu,args | grep -E 'raspbot|rplidar|component_container|twist_mux|robot_state|ekf|safety' | grep -v grep | awk '{s+=$1} END {printf "%.1f", s}')
  SOC=$(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 ))
  I2C_ERR=$(timeout -s KILL 6 ros2 topic echo /diagnostics --once 2>/dev/null | grep -iE 'i2c_error|i2c 错误' | grep -oE '[0-9]+' | head -1)
  echo "$i,${RSS:-0},${CPU:-0},$SOC,${I2C_ERR:-0}"
  if ! pgrep -f 'bringup.launch.py' >/dev/null; then echo "!! bringup 进程退出于第 $i 次采样"; break; fi
  sleep 30
done

echo
echo "== 结束状态 =="
tail -5 /tmp/st_soak.log | sed 's/^/  /'
echo "[DONE] 10_soak 结束（完整 2 h 长稳 SYS-R01 另排）"
