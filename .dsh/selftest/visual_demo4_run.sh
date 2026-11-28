#!/usr/bin/env bash
# visual_demo4_run.sh —— 标定后的编号动作序列（前进/后退 30cm，旋转 90°）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_demo4.log 2>&1 &
sleep 20
python3 -u "$HOME/tools/selftest/visual_demo4.py" --dist-fb 0.30 --dist-lr 0.15 --deg 90 \
  2>&1 | sed 's/^/  /'
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
