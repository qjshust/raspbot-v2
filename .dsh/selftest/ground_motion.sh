#!/usr/bin/env bash
# ground_motion.sh —— 落地运动测试（C++ 版，低速安全档）
# 阶段 1：6 段低速运动探针（IMU/scan 物理证据）
# 阶段 2：指令链一致性（/odom_raw.twist 跟随 /cmd_vel_safe）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
echo "== 启动 bringup（mode=none，C++ 节点）=="
nohup ros2 launch raspbot_bringup bringup.launch.py mode:=none > /tmp/st_ground.log 2>&1 &
sleep 22
echo "== 节点 =="
timeout -s KILL 8 ros2 node list 2>/dev/null | sed 's/^/  /'
echo "== 数据流频率 =="
for t in /odom_raw /imu/data /scan /cmd_vel_safe; do
  HZ=$(timeout -s KILL 6 ros2 topic hz "$t" 2>&1 | grep -oE 'average rate: [0-9.]+' | tail -1)
  echo "  $t: ${HZ:-<无消息>}"
done
echo
echo "== 运动探针（低速：0.10 m/s × 1.5 s 等）=="
python3 -u "$HOME/tools/selftest/ground_move_probe.py" 2>&1 | sed 's/^/  /'
RC=$?
echo "探针 RC=$RC"
echo "== 收尾（安全态）=="
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1
exit $RC
