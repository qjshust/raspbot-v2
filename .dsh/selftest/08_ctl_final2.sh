#!/usr/bin/env bash
# 08_ctl_final2.sh —— final_probe 观测（无 CLI、无 bag、无阻塞）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
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
echo "== final_probe =="
python3 -u "$HOME/tools/selftest/final_probe.py" --x 1.0 --n 40 --duration 40 > /tmp/final_probe.log 2>&1
RC=$?
cat /tmp/final_probe.log | sed 's/^/  /'
echo "  RC=$RC"
echo "== 控制器关键日志 =="
grep -iE 'Publishing velocity|Failed to make progress|Received a goal|computeControl' /tmp/st_ctl.log | tail -8 | sed 's/^/  /'
exit $RC
