#!/usr/bin/env bash
# 08_ctl_standalone2.sh —— 独立 controller + mppi_probe（无 sed 缓冲、无阻塞结果等待）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none use_mock:=true > /tmp/st_mock.log 2>&1 &
sleep 25
PARAMS="$HOME/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/nav2/nav2_params.yaml"
nohup ros2 run nav2_controller controller_server --ros-args \
  --params-file "$PARAMS" \
  -r cmd_vel:=cmd_vel_nav_raw \
  --log-level debug > /tmp/st_ctl.log 2>&1 &
sleep 8
timeout -s KILL 12 ros2 lifecycle set /controller_server configure >/dev/null 2>&1
timeout -s KILL 15 ros2 lifecycle set /controller_server activate >/dev/null 2>&1
sleep 2
echo "== 控制器生命周期 =="
timeout -s KILL 8 ros2 lifecycle get /controller_server 2>&1 | sed 's/^/  /'
echo "== mppi_probe =="
python3 -u "$HOME/tools/selftest/mppi_probe.py" --x 1.0 --n 40 --duration 40 > /tmp/probe.log 2>&1
RC=$?
cat /tmp/probe.log | sed 's/^/  /'
echo "  RC=$RC"
echo "== 控制器关键日志 =="
grep -iE 'MPPIController|mppi::|critic|Failed to make progress|Publishing velocity' /tmp/st_ctl.log | grep -viE 'ClassLoader|rmw|pluginlib' | tail -15 | sed 's/^/  /'
exit $RC
