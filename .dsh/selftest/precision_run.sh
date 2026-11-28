#!/usr/bin/env bash
# precision_run.sh —— 落地旋转/横移精度测试（C++ 节点）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_prec.log 2>&1 &
sleep 20
python3 -u "$HOME/tools/selftest/precision_test.py" 2>&1 | sed 's/^/  /'
RC=${PIPESTATUS[0]}
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
exit $RC
