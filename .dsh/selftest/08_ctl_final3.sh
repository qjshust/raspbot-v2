#!/usr/bin/env bash
# 08_ctl_final3.sh —— 线级验证：CLI echo + hz + 全 cmd 主题订阅
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
echo "env: ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} RMW=${RMW_IMPLEMENTATION:-<default>}"
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none use_mock:=true > /tmp/st_mock.log 2>&1 &
sleep 25
PARAMS="$HOME/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/nav2/nav2_params.yaml"
nohup ros2 run nav2_controller controller_server --ros-args \
  --params-file "$PARAMS" -r cmd_vel:=cmd_vel_nav_raw \
  --log-level debug > /tmp/st_ctl.log 2>&1 &
sleep 8
timeout -s KILL 12 ros2 lifecycle set /controller_server configure >/dev/null 2>&1
timeout -s KILL 15 ros2 lifecycle set /controller_server activate >/dev/null 2>&1
sleep 2
# 目标期间并发观测
python3 -u "$HOME/tools/selftest/send_follow_path.py" --x 1.0 --n 40 --duration 30 > /tmp/sender.log 2>&1 &
SENDER=$!
sleep 6
echo "== 目标期间（t+6s）=="
echo "-- hz /cmd_vel_nav_raw:"
timeout -s KILL 10 ros2 topic hz /cmd_vel_nav_raw 2>&1 | grep -E 'average rate|no new' | tail -2 | sed 's/^/   /'
echo "-- echo /cmd_vel_nav_raw --once:"
timeout -s KILL 10 ros2 topic echo /cmd_vel_nav_raw --once 2>&1 | grep -A3 'linear:' | sed 's/^/   /'
echo "-- echo /cmd_vel --once:"
timeout -s KILL 6 ros2 topic echo /cmd_vel --once 2>&1 | grep -A3 'linear:' | sed 's/^/   /'
echo "-- echo /cmd_vel_nav --once:"
timeout -s KILL 6 ros2 topic echo /cmd_vel_nav --once 2>&1 | grep -A3 'linear:' | sed 's/^/   /'
wait $SENDER 2>/dev/null
echo "== 发送器 =="
cat /tmp/sender.log | tail -4 | sed 's/^/  /'
echo "== 控制器日志（Publishing velocity 行数）=="
grep -c 'Publishing velocity' /tmp/st_ctl.log | sed 's/^/  count: /'
