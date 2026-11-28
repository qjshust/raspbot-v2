#!/usr/bin/env bash
# visual_demo3_run.sh —— 起栈 + 编号动作序列 v3（前进/后退 30 cm）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_demo3.log 2>&1 &
sleep 20
python3 -u "$HOME/tools/selftest/visual_demo3.py" --speed 0.08 --yaw 0.20 \
  --dist-fb 0.30 --dist-lr 0.15 2>&1 | sed 's/^/  /'
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
