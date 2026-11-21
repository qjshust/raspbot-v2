#!/usr/bin/env bash
# 05b_systemd.sh —— SW-F03 / SYS-R02 部分：systemd 守护 + kill -9 自动恢复
# 前置：H 的 install_systemd.sh 已执行；测试用 drop-in 覆盖为 mode:=none。
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

cleanup() {
  sudo systemctl stop raspbot.service 2>/dev/null
  sudo rm -rf /etc/systemd/system/raspbot.service.d 2>/dev/null
  sudo systemctl daemon-reload
  sleep 1
}
trap cleanup EXIT INT TERM

echo "== 安装 drop-in 覆盖（mode:=none 用于台架测试）=="
sudo mkdir -p /etc/systemd/system/raspbot.service.d
sudo tee /etc/systemd/system/raspbot.service.d/override.conf >/dev/null <<EOF
[Service]
ExecStart=
ExecStart=/bin/bash -lc 'source /opt/ros/lyrical/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && exec ros2 launch raspbot_bringup bringup.launch.py mode:=none'
EOF
sudo systemctl daemon-reload
echo "  [OK] override 写入"

echo "== 启动 raspbot.service =="
sudo systemctl restart raspbot.service
sleep 25
systemctl is-active raspbot.service | sed 's/^/  状态: /'
pgrep -f base_node >/dev/null && pass "服务化 base_node 在跑" || fail "base_node 不在跑"

echo
echo "== 注入 0.15 m/s 后 kill -9 base_node（SW-F03；imu 用服务自带的）=="
sleep 3
python3 - "$HOME/tools/selftest/vibration_probe.py" <<'PYEOF' &
import sys, time, subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init()
n = Node('inj')
p = n.create_publisher(Twist, '/cmd_vel_safe', 10)
m = Twist(); m.linear.x = 0.15
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:
    p.publish(m)
    time.sleep(1.0/30.0)
PYEOF
INJ=$!
sleep 0.5
python3 "$HOME/tools/selftest/vibration_probe.py" --seconds 1 --label pre_kill 2>&1 | sed 's/^/  /'
BASE_PID=$(pgrep -f 'base_node' | head -1)
echo "  base_node PID=$BASE_PID，kill -9"
T_KILL=$(date +%s.%N)
sudo kill -9 "$BASE_PID" 2>/dev/null || kill -9 "$BASE_PID" 2>/dev/null
# 振动探针持续采样，直到 systemd 重启完成（imu 被连带重启则探针自然退出）
python3 "$HOME/tools/selftest/vibration_probe.py" --seconds 20 --label post_kill 2>&1 | sed 's/^/  /' || true
wait $INJ 2>/dev/null
T_RECOVER=$(date +%s.%N)
echo "  kill 时刻后已过 $(( $(date +%s) - ${T_KILL%.*} )) s"
sleep 8
systemctl is-active raspbot.service | sed 's/^/  服务状态: /'
if pgrep -f base_node >/dev/null; then pass "systemd 自动重启，base_node 恢复"; else fail "base_node 未恢复"; fi
T2=$(pgrep -f base_node | head -1)
echo "  新 base_node PID=$T2（与 $BASE_PID 不同=确实重启）"

echo
echo "== 恢复后发指令验证仍可用 =="
python3 - <<'PYEOF'
import time, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init()
n = Node('inj2')
p = n.create_publisher(Twist, '/cmd_vel_safe', 10)
m = Twist(); m.linear.x = 0.10
t0 = time.monotonic()
while time.monotonic() - t0 < 1.5:
    p.publish(m)
    time.sleep(1.0/30.0)
m.linear.x = 0.0
for _ in range(5): p.publish(m); time.sleep(0.1)
n.destroy_node(); rclpy.shutdown()
print('  指令注入完成')
PYEOF
timeout -s KILL 4 ros2 topic echo /odom_raw --field twist.twist --once 2>/dev/null | sed 's/^/  /'

echo
if [ $FAILS -eq 0 ]; then echo "[PASS] 05b_systemd 全部通过"; else echo "[FAIL] 05b_systemd 有 $FAILS 项失败"; exit 1; fi
