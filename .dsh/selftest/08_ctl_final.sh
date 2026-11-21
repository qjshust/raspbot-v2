#!/usr/bin/env bash
# 08_ctl_final.sh —— 决定性观测：目标期间 CLI echo 值 + hz 计数 + bag 录制
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
timeout -s KILL 8 ros2 lifecycle get /controller_server | sed 's/^/  生命周期: /'

echo "== 起 bag 录制（后台）=="
rm -rf /tmp/ab_bag
timeout -s KILL 45 ros2 bag record -o /tmp/ab_bag \
  /cmd_vel_nav_raw /odom /controller_server/optimal_trajectory > /tmp/bag.log 2>&1 &
BAG=$!
sleep 3

echo "== 发目标（后台）=="
python3 -u "$HOME/tools/selftest/send_follow_path.py" --x 1.0 --n 40 --duration 30 > /tmp/sender.log 2>&1 &
SENDER=$!
sleep 8

echo "== 目标期间观测 =="
echo "-- ros2 topic hz /cmd_vel_nav_raw:"
timeout -s KILL 8 ros2 topic hz /cmd_vel_nav_raw 2>&1 | grep -E 'average rate|no new' | tail -2 | sed 's/^/   /'
echo "-- ros2 topic echo /cmd_vel_nav_raw --once:"
timeout -s KILL 8 ros2 topic echo /cmd_vel_nav_raw --once 2>&1 | grep -A3 'linear:' | sed 's/^/   /' || echo "   (无消息)"
echo "-- ros2 topic info -v /cmd_vel_nav_raw:"
timeout -s KILL 8 ros2 topic info -v /cmd_vel_nav_raw 2>&1 | grep -E 'Publisher count|Node name|Reliability' | head -10 | sed 's/^/   /'

wait $SENDER 2>/dev/null
cat /tmp/sender.log | sed 's/^/  发送器: /'
wait $BAG 2>/dev/null

echo "== bag 内容 =="
ros2 bag info /tmp/ab_bag 2>&1 | grep -E 'Duration|Messages|Topic information|/cmd_vel_nav_raw|/odom|optimal' | sed 's/^/  /'
