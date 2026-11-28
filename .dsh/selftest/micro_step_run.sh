#!/usr/bin/env bash
# micro_step_run.sh —— 起栈 + 10cm 微量步进自测（雷达测距）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_micro.log 2>&1 &
sleep 20
echo "== 生效参数（应与 120 一致）=="
timeout -s KILL 8 ros2 param get /raspbot_base pwm_per_mps 2>&1 | sed 's/^/  /'
timeout -s KILL 8 ros2 param get /raspbot_base max_linear 2>&1 | sed 's/^/  /'
timeout -s KILL 8 ros2 param get /cmd_vel_limiter max_linear 2>&1 | sed 's/^/  /'
echo
echo "== 10cm 微量步进（单次脉冲，测后立即停）=="
python3 -u "$HOME/tools/selftest/micro_step.py" --dist 0.10 --cmd 0.05 \
  --pwm-per-mps 120 --deadband 25 2>&1 | sed 's/^/  /'
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
