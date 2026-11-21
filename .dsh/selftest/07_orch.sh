#!/usr/bin/env bash
# 07_orch.sh —— 用 sil_orchestrator.py 跑一体化 SIL 导航诊断（远程落盘日志）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
MAP="/mnt/ssd/maps/sil_mock_room.yaml"
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=nav use_mock:=true map:="$MAP" > /tmp/st_probe.log 2>&1 &
sleep 45
python3 -u "$HOME/tools/selftest/sil_orchestrator.py" --goal-x 1.5 --duration 100 > /tmp/orch.log 2>&1
RC=$?
cat /tmp/orch.log | sed 's/^/  /'
echo "  RC=$RC"
echo "== 关键日志 =="
grep -iE 'Failed to make progress|Inconsistent|collision' /tmp/st_probe.log | grep -viE 'Creating|enabled at startup|Critic|collision checking' | tail -8 | sed 's/^/  /'
exit $RC
