#!/usr/bin/env bash
# 08_ctl_ab.sh —— A/B 实验：官方 nav2 参数 vs 本工程参数的独立控制器
# 用法: bash 08_ctl_ab.sh <params_file> <label>
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
PARAMS="$1"
LABEL="$2"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none use_mock:=true > /tmp/st_mock.log 2>&1 &
sleep 25
nohup ros2 run nav2_controller controller_server --ros-args \
  --params-file "$PARAMS" \
  -r cmd_vel:=cmd_vel_nav_raw \
  --log-level info > /tmp/st_ctl.log 2>&1 &
sleep 8
timeout -s KILL 12 ros2 lifecycle set /controller_server configure >/dev/null 2>&1
timeout -s KILL 15 ros2 lifecycle set /controller_server activate >/dev/null 2>&1
sleep 2
ST=$(timeout -s KILL 8 ros2 lifecycle get /controller_server 2>&1 | grep -oE 'active \[3\]')
echo "[$LABEL] controller lifecycle: ${ST:-未激活}"
python3 -u "$HOME/tools/selftest/mppi_probe.py" --x 1.0 --n 40 --duration 40 > /tmp/probe.log 2>&1
RC=$?
echo "[$LABEL] 探针输出："
cat /tmp/probe.log | sed 's/^/  /'
echo "[$LABEL] 控制器日志："
grep -iE 'error|warn|invalid|failed|MPPIController' /tmp/st_ctl.log | grep -viE 'lifecycle|bond' | tail -8 | sed 's/^/  /'
exit $RC
