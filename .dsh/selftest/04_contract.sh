#!/usr/bin/env bash
# 04_contract.sh —— 全栈契约测试（SW-C01~C07 / SW-I01 三节点并发）
# 前置：已 colcon build。起 bringup（mode=none + 相机），跑 contract_check.py。
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"

cleanup() { bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1

echo "== 启动 bringup（mode=none, 含相机）=="
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none use_camera:=true > /tmp/st_bringup.log 2>&1 &
sleep 25

echo "== 节点列表 =="
timeout -s KILL 8 ros2 node list 2>&1 | sed 's/^/  /'

echo
echo "== 契约检查（含相机，6 s 窗口）=="
python3 "$HOME/tools/selftest/contract_check.py" --with-camera --window 6 2>&1 | sed 's/^/  /'
RC=${PIPESTATUS[0]}

echo
echo "== TF 树（view_frames）=="
timeout -s KILL 12 ros2 run tf2_tools view_frames 2>&1 | tail -2 | sed 's/^/  /'
if [ -f frames.gv ]; then cp frames.gv /tmp/st_frames.gv; echo "  frames.gv 已存 /tmp/st_frames.gv"; fi

echo
echo "== tf2_monitor（唯一权威检查）=="
timeout -s KILL 4 ros2 run tf2_ros tf2_monitor 2>&1 | grep -E 'Frame:|Authority|No authority|duplicate' | head -30 | sed 's/^/  /'

echo
echo "== 参数 dump /raspbot_base =="
timeout -s KILL 8 ros2 param dump /raspbot_base 2>/dev/null | head -30 | sed 's/^/  /'

echo
echo "== bringup 日志尾部 =="
tail -8 /tmp/st_bringup.log | sed 's/^/  /'

echo
if [ $RC -eq 0 ]; then echo "[PASS] 04_contract 全部通过"; else echo "[FAIL] 04_contract 失败（见上）"; fi
exit $RC
