#!/usr/bin/env bash
# lidar_yaw_run.sh —— 雷达互相关独立验证旋转角（左转、右转各一次）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_ly.log 2>&1 &
sleep 20
echo "########## 左转 wz=+0.2 × 2 s ##########"
python3 -u "$HOME/tools/selftest/lidar_yaw_check.py" --wz 0.2 --sec 2.0 --sign 1 2>&1 | sed 's/^/  /'
sleep 2
echo "########## 右转 wz=-0.2 × 2 s ##########"
python3 -u "$HOME/tools/selftest/lidar_yaw_check.py" --wz 0.2 --sec 2.0 --sign -1 2>&1 | sed 's/^/  /'
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
