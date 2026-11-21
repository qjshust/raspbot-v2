#!/usr/bin/env bash
# 02_actuators.sh —— 执行器与板载传感器台架测试（CAR-B04~B07 远程适配版）
# 前置：已 colcon build；车四轮悬空；摄像头在云台上。
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

cleanup() {
  pkill -f 'base_node' 2>/dev/null
  sleep 1
}
trap cleanup EXIT INT TERM
cleanup

echo "== 启动 base_node =="
nohup ros2 launch raspbot_base base.launch.py > /tmp/st_base.log 2>&1 &
BASE_PID=$!
sleep 6
if pgrep -f base_node >/dev/null; then pass "base_node 已启动"; else fail "base_node 未启动"; tail -5 /tmp/st_base.log; exit 1; fi

echo
echo "== 1. 超声波 /range（悬空基线，前 3 帧丢弃）=="
python3 - <<'PYEOF'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
rclpy.init()
node = Node('range_check')
vals = []
frames = []
node.create_subscription(Range, '/range',
                         lambda m: (vals.append(m.range), frames.append(m.header.frame_id)),
                         qos_profile_sensor_data)
import time
deadline = time.monotonic() + 6.0
while time.monotonic() < deadline and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.2)
n = len(vals)
if n < 5:
    print(f"  [FAIL] /range 样本不足 n={n}")
else:
    body = vals[3:]
    sat = sum(1 for v in body if v >= 3.99)
    real = [v for v in body if v < 3.99]
    ok = all(v >= 0.02 for v in body) and len(real) > 0
    print(f"  n={n} 前3帧={vals[:3]} 实测段={real[:6]} 饱和(无回波)帧={sat}/{len(body)}  frame={frames[0]}")
    if ok:
        print(f"  [PASS] 解码与链路正常（环境无固定挡板，回波断续；精度测试 CAR-B06 需人工挡板，另行执行）")
    else:
        print(f"  [FAIL] 解码异常")
node.destroy_node(); rclpy.shutdown()
PYEOF

echo
echo "== 2. 巡线 /line_sensor =="
timeout -s KILL 8 ros2 topic echo /line_sensor --once 2>/dev/null | sed 's/^/  /'

echo
echo "== 3. 舵机 pan（云台相机验证画面随动）=="
python3 "$HOME/tools/selftest/servo_camera_check.py" --joint pan --poses 0 90 180 90 --outdir /tmp/servo 2>&1 | sed 's/^/  /'
PAN_RC=${PIPESTATUS[0]}
[ $PAN_RC -eq 0 ] && pass "pan 随动验证通过" || fail "pan 随动验证失败"

echo
echo "== 4. 舵机 tilt =="
python3 "$HOME/tools/selftest/servo_camera_check.py" --joint tilt --poses 0 60 110 60 --outdir /tmp/servo 2>&1 | sed 's/^/  /'
TILT_RC=${PIPESTATUS[0]}
[ $TILT_RC -eq 0 ] && pass "tilt 随动验证通过" || fail "tilt 随动验证失败"

echo
echo "== 5. RGB 7 色 + 灭灯 =="
for c in '0.0 0.0 0.0' '1.0 0.0 0.0' '0.0 1.0 0.0' '0.0 0.0 1.0' '1.0 1.0 0.0' '0.6 0.0 1.0' '0.0 1.0 1.0' '1.0 1.0 1.0'; do
  set -- $c
  timeout -s KILL 5 ros2 topic pub --once /led std_msgs/msg/ColorRGBA "{r: $1, g: $2, b: $3, a: 1.0}" >/dev/null 2>&1
  sleep 0.4
done
timeout -s KILL 5 ros2 topic pub --once /led std_msgs/msg/ColorRGBA "{r: 0.0, g: 0.0, b: 0.0, a: 1.0}" >/dev/null 2>&1
pass "RGB 8 次指令下发完成（ACK 路径，无读回）"

echo
echo "== 6. 蜂鸣器 3 次 =="
for i in 1 2 3; do
  timeout -s KILL 5 ros2 topic pub --once /buzzer std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
  sleep 0.3
  timeout -s KILL 5 ros2 topic pub --once /buzzer std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1
  sleep 0.3
done
pass "蜂鸣器 3 次指令下发完成（ACK 路径，无读回）"

echo
echo "== 7. I2C 错误计数（/raspbot_base/diagnostics）=="
timeout -s KILL 8 ros2 topic echo /raspbot_base/diagnostics --once 2>/dev/null | grep -E 'name:|message:|i2c' | sed 's/^/  /' || echo "  （无诊断话题，跳过）"

echo
echo "== 8. base 日志尾部 =="
tail -5 /tmp/st_base.log | sed 's/^/  /'

echo
echo "== 结论 =="
if [ $FAILS -eq 0 ]; then echo "[PASS] 02_actuators 全部通过"; else echo "[FAIL] 02_actuators 有 $FAILS 项失败"; exit 1; fi
