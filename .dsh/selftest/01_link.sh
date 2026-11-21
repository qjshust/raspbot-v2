#!/usr/bin/env bash
# 01_link.sh —— 链路与 I2C 健康（CAR-B01 / SW-P05 部分 / SW-I09 部分）
LC_ALL=C
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

echo "== 1. I2C 设备在位 =="
for addr in 2b 3c; do
  if i2cdetect -y 1 2>/dev/null | grep -q "$addr"; then pass "0x$addr 在线"; else fail "0x$addr 不在线"; fi
done

echo
echo "== 2. udev 固定命名（软链）=="
[ -e /dev/raspbot_lidar ] && [ -e /dev/raspbot_imu ] && pass "两个软链存在" || fail "软链缺失"

echo
echo "== 3. I2C 事务计时与错误率（100 次写 0x06）=="
python3 - <<'PYEOF'
import smbus, time
bus = smbus.SMBus(1)
N = 100
err = 0
t0 = time.perf_counter()
for _ in range(N):
    try:
        bus.write_i2c_block_data(0x2B, 0x06, [0])
    except Exception:
        err += 1
dt = (time.perf_counter() - t0) / N * 1000
print(f"  {N} 次写：err={err}，单次 {dt:.3f} ms")
print(f"  [{'PASS' if err==0 and dt<=1.0 else 'FAIL'}] 错误率<0.1% 且 单次≤1 ms")
PYEOF

echo
echo "== 4. 双开互斥机制演示（SW-I09 机制）=="
( flock -n /var/lock/raspbot-i2c.lock -c 'sleep 4' & echo "  第一个持有者已获得锁（PID $!）" ) >/dev/null
sleep 0.5
if flock -n /var/lock/raspbot-i2c.lock -c 'true' 2>/dev/null; then
  fail "第二个进程意外获得锁"
else
  pass "第二个进程被明确拒绝（互斥生效）"
fi
wait
rm -f /var/lock/raspbot-i2c.lock

echo
echo "== 5. /dev/i2c-1 当前占用者 =="
fuser -v /dev/i2c-1 2>&1 | sed 's/^/  /' || echo "  无占用者"

echo
echo "== 结论 =="
if [ $FAILS -eq 0 ]; then echo "[PASS] 01_link 全部通过"; else echo "[FAIL] 01_link 有 $FAILS 项失败"; exit 1; fi
