#!/usr/bin/env bash
# visual_demo_run.sh —— 起栈 + 目视方向确认序列
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_demo.log 2>&1 &
sleep 20
python3 -u "$HOME/tools/selftest/visual_demo.py" --speed 0.10 --yaw 0.30 --sec 1.0 2>&1 | sed 's/^/  /'
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
