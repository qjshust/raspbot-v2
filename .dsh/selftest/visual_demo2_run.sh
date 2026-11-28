#!/usr/bin/env bash
# visual_demo2_run.sh —— 起栈 + 编号动作序列 v2（哔声编号）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_demo2.log 2>&1 &
sleep 20
# 先确认蜂鸣器/LED 话题有订阅者（base_node 在跑）
echo "== 控制话题订阅者 =="
timeout -s KILL 6 ros2 topic info /buzzer 2>&1 | grep -E 'Subscription count' | sed 's/^/  /'
timeout -s KILL 6 ros2 topic info /led 2>&1 | grep -E 'Subscription count' | sed 's/^/  /'
python3 -u "$HOME/tools/selftest/visual_demo2.py" --lin 0.08 --yaw 0.20 2>&1 | sed 's/^/  /'
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
