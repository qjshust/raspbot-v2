#!/usr/bin/env bash
# straight_run.sh —— 落地直线精度测试（C++ 节点）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_ground.log 2>&1 &
sleep 22
echo "== 直行测试（0.15 m/s × 1.2 m，前向近距保护 0.15 m）=="
python3 -u "$HOME/tools/selftest/straight_line_test.py" --speed 0.15 --dist 1.2 --abort-range 0.15 2>&1 | sed 's/^/  /'
RC=${PIPESTATUS[0]}
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
exit $RC
