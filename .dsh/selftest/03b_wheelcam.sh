#!/usr/bin/env bash
# 03b_wheelcam.sh —— 云台相机俯拍车轮画面采集（寻找能看到轮子的视角）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
cleanup() {
  pkill -f base_node 2>/dev/null
  pkill -f wheel_cam 2>/dev/null
  sleep 1
}
trap cleanup EXIT INT TERM
cleanup
nohup ros2 launch raspbot_base base.launch.py > /tmp/st_base.log 2>&1 &
sleep 6
python3 "$HOME/tools/selftest/wheel_cam.py" --tilt 110 --pans 0 60 120 180 --outdir /tmp/wheelcam 2>&1 | sed 's/^/  /'
ls -l /tmp/wheelcam/ | sed 's/^/  /'
