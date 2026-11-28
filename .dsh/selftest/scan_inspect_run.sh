#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_scan.log 2>&1 &
sleep 20
python3 -u "$HOME/tools/selftest/scan_inspect.py" 2>&1 | sed 's/^/  /'
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
