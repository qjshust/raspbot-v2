#!/usr/bin/env bash
# 08_ctl_standalone.sh —— 独立 controller_server 实验（DEBUG 日志）
# 环境：mock 三件套 + fusion + mux + safety + description（无 Nav2 其余节点）
# 手工起 controller_server（MPPI-Omni 参数 + debug 日志），发带时间戳的手工路径。
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
echo "== 起 mock 环境（mode=none, use_mock）=="
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none use_mock:=true > /tmp/st_mock.log 2>&1 &
sleep 25
PARAMS="$HOME/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/nav2/nav2_params.yaml"
echo "== 起独立 controller_server（debug）=="
nohup ros2 run nav2_controller controller_server --ros-args \
  --params-file "$PARAMS" \
  -r cmd_vel:=cmd_vel_nav_raw \
  --log-level debug > /tmp/st_ctl.log 2>&1 &
sleep 8
echo "== 生命周期 =="
timeout -s KILL 12 ros2 lifecycle set /controller_server configure 2>&1 | sed 's/^/  /'
timeout -s KILL 15 ros2 lifecycle set /controller_server activate 2>&1 | sed 's/^/  /'
sleep 2
timeout -s KILL 8 ros2 lifecycle get /controller_server 2>&1 | sed 's/^/  /'
echo "== 发手工路径（odom 系 0→1.0 m，40 位姿，带时间戳）=="
python3 -u "$HOME/tools/selftest/send_follow_path.py" --x 1.0 --n 40 --duration 45 2>&1 | sed 's/^/  /'
RC=${PIPESTATUS[0]}
echo "== controller debug 日志（关键行）=="
grep -iE 'mppi|optimizer|critic|path|trajector|control|compute|goal|receive' /tmp/st_ctl.log | grep -viE 'get_parameters|parameter|plugin_lib' | tail -40 | sed 's/^/  /'
echo "RC=$RC"
exit $RC
