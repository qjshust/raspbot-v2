#!/usr/bin/env bash
# ground_env_check.sh —— 落地测试前环境检查（C++ 版）
LC_ALL=C
echo "== 主机/网络 =="
hostname; uptime | head -1
ip -4 addr show wlan0 2>/dev/null | grep inet
echo "== 工作区构建产物 =="
ls ~/ros2_ws/install/raspbot_base/lib/raspbot_base/ 2>/dev/null | head -5
echo "== 设备节点 =="
for d in /dev/raspbot_imu /dev/raspbot_lidar /dev/raspbot_camera /dev/i2c-1 /dev/hailo0; do
  [ -e "$d" ] && echo "  OK  $d" || echo "  MISS $d"
done
echo "== I2C 0x2B =="
i2cdetect -y 1 2>/dev/null | grep -E '^ 2|2b' | head -2
echo "== 磁盘（地图）=="
ls /mnt/ssd/maps/ 2>/dev/null | head -6
echo "== 温度/负载 =="
cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | awk '{printf "  SoC %.1f C\n", $1/1000}'
uptime | sed 's/^/  /'
