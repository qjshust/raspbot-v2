#!/usr/bin/env bash
# slip_quantify_run.sh —— 起栈 + 瓷砖打滑量化
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_slip.log 2>&1 &
sleep 20
python3 -u "$HOME/tools/selftest/slip_quantify.py" --pwm-per-mps 120 --deadband 25 \
  2>&1 | sed 's/^/  /'
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
