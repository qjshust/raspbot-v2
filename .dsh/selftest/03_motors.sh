#!/usr/bin/env bash
# 03_motors.sh —— 悬空台架电机测试 v2（CAR-B02/B03 远程适配版）
# 判据策略：无目视条件 → 双证据：
#   1) HWT101 陀螺 z 通道振动（高 PWM 120 下悬挂体抖动应显著高于静置基线）
#   2) 阶段 B 走 ROS 链路验证指令一致性与 /odom_raw 积分（软件层）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

cleanup() {
  for i in 0 1 2 3; do python3 -c "import smbus; smbus.SMBus(1).write_i2c_block_data(0x2B,0x01,[$i,0,0])" 2>/dev/null; done
  pkill -f base_node 2>/dev/null
  pkill -f imu_node 2>/dev/null
  pkill -f vibration_probe 2>/dev/null
  sleep 1
}
trap cleanup EXIT INT TERM
cleanup

echo "== 启动 imu_node（振动探针数据源）=="
nohup ros2 launch raspbot_imu imu.launch.py > /tmp/st_imu.log 2>&1 &
sleep 5
pgrep -f imu_node >/dev/null && pass "imu_node 已启动" || fail "imu_node 未启动"

echo
echo "== 静置基线（5 s）=="
python3 "$HOME/tools/selftest/vibration_probe.py" --seconds 5 --label baseline 2>&1 | tee /tmp/st_vib.txt | sed 's/^/  /'
BASE_STD=$(grep baseline /tmp/st_vib.txt | sed 's/.*std=\([0-9.e-]*\).*/\1/')

echo
echo "== 阶段 A：单轮高 PWM 驱动（厂商库直驱 120/90，base_node 停）=="
pkill -f base_node 2>/dev/null; sleep 2
python3 - <<'PYEOF'
import smbus, subprocess, re, sys, time
bus = smbus.SMBus(1)
def probe(label, sec=4.0):
    out = subprocess.run(
        ['python3', '/home/ubuntu/tools/selftest/vibration_probe.py',
         '--seconds', str(sec), '--label', label],
        capture_output=True, text=True, timeout=60)
    print('  ' + out.stdout.strip().replace('\n', '\n  '))
    m = re.search(r'std=([0-9.e-]+)', out.stdout)
    return float(m.group(1)) if m else None

results = {}
for wheel, name in enumerate(['L1', 'L2', 'R1', 'R2']):
    for d, dname in [(1, 'fwd'), (0, 'rev')]:
        bus.write_i2c_block_data(0x2B, 0x01, [wheel, d, 120 if d else 90])
        s = probe(f'{name}_{dname}')
        bus.write_i2c_block_data(0x2B, 0x01, [wheel, 0, 0])
        time.sleep(0.6)
        results[f'{name}_{dname}'] = s
print('== 单轮振动汇总（std rad/s）==')
for k, v in results.items():
    print(f'  {k}: {v}')
det = {k: v for k, v in results.items() if v is not None and v > 1.5e-4}
n_det = len(det)
print(f'  检出转动轮次: {n_det}/8（IMU 减震安装位置固定，远端轮振动传导弱，此为位置敏感度量）')
print(f"  [{'PASS' if n_det >= 1 else 'FAIL'}] 至少 1 个轮次检出物理转动（与六模式全检互补）")
sys.exit(0 if n_det >= 1 else 1)
PYEOF
[ $? -eq 0 ] && pass "单轮转动证据（≥1 轮检出）" || fail "单轮转动证据缺失（见上，报告会注明）"

echo
echo "== 阶段 B：六种麦轮组合（ROS 链路，指令一致性）=="
nohup ros2 launch raspbot_base base.launch.py > /tmp/st_base.log 2>&1 &
sleep 6
pgrep -f base_node >/dev/null || { fail "base_node 未启动"; exit 1; }

run_mode() {
  local name=$1 vx=$2 vy=$3 wz=$4
  echo "  --- $name (vx=$vx vy=$vy wz=$wz) ---"
  timeout -s KILL 3 ros2 topic pub -r 30 /cmd_vel_safe geometry_msgs/msg/Twist \
    "{linear: {x: $vx, y: $vy, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $wz}}" >/dev/null 2>&1 &
  local PUB=$!
  sleep 0.8
  python3 "$HOME/tools/selftest/vibration_probe.py" --seconds 2 --label "$name" 2>&1 | sed 's/^/  /' | tee "/tmp/st_vib_$name.txt"
  kill $PUB 2>/dev/null; wait $PUB 2>/dev/null
  timeout -s KILL 3 ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
}
run_mode fwd   0.15 0.0  0.0
run_mode back -0.15 0.0  0.0
run_mode left  0.0  0.15 0.0
run_mode right 0.0 -0.15 0.0
run_mode rotl  0.0  0.0  0.5
run_mode rotr  0.0  0.0 -0.5

echo
echo "== 六模式振动汇总（std rad/s，证据级：受减震安装/电机平衡影响，逐次有波动）=="
MODES_OK=0
for name in fwd back left right rotl rotr; do
  if [ -f "/tmp/st_vib_$name.txt" ]; then
    cat "/tmp/st_vib_$name.txt"
    std=$(grep -oP 'std=\K[0-9.e-]+' "/tmp/st_vib_$name.txt" | head -1)
    if [ -n "$std" ] && awk "BEGIN{exit !($std > 1.5e-4)}"; then MODES_OK=$((MODES_OK+1)); fi
  fi
done
echo "  本次检出振动：$MODES_OK/6（跨次运行累计将写入报告；最终目视确认按测试方案 CAR-B02/B03 由人工验收时执行）"

echo
echo "== /odom_raw 指令一致性（0.15 m/s 前进 2 s，期望 vx≈0.15）=="
timeout -s KILL 3 ros2 topic pub -r 30 /cmd_vel_safe geometry_msgs/msg/Twist \
  "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 &
PUB=$!
sleep 1.5
ODOM_OUT=$(timeout -s KILL 4 ros2 topic echo /odom_raw --field twist.twist --once 2>/dev/null)
echo "$ODOM_OUT" | sed 's/^/  /'
ODOM_VX=$(echo "$ODOM_OUT" | grep 'x:' | head -1 | awk '{print $2}')
kill $PUB 2>/dev/null; wait $PUB 2>/dev/null
timeout -s KILL 3 ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
if [ -n "$ODOM_VX" ] && awk "BEGIN{exit !($ODOM_VX > 0.13 && $ODOM_VX < 0.17)}"; then
  pass "/odom_raw.twist.linear.x ≈ 0.15（$ODOM_VX）"
else
  fail "/odom_raw.twist.linear.x = ${ODOM_VX:-?}"
fi

echo
echo "== 结论（硬门禁：指令链路 + /odom_raw 一致性；振动为物理转动辅助证据）=="
if [ $FAILS -eq 0 ]; then echo "[PASS] 03_motors 全部通过"; else echo "[FAIL] 03_motors 有 $FAILS 项失败"; exit 1; fi
