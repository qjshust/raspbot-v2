#!/usr/bin/env bash
# rot_trace_run.sh —— 起栈 + 旋转轨迹测试（左转、右转各一段）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_rot.log 2>&1 &
sleep 20
echo "########## 左转 wz=+0.5 × 2 s ##########"
python3 -u "$HOME/tools/selftest/rot_trace.py" --wz 0.5 --sec 2.0 --sign 1 2>&1 | sed 's/^/  /'
sleep 2
echo "########## 右转 wz=-0.5 × 2 s ##########"
python3 -u "$HOME/tools/selftest/rot_trace.py" --wz 0.5 --sec 2.0 --sign -1 2>&1 | sed 's/^/  /'
echo "== 收尾 =="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
