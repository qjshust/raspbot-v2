#!/usr/bin/env bash
# 00_env.sh —— 环境快照与上电前检查单（PWR-* 可远程核对的项）
LC_ALL=C
echo "== 日期/主机 =="
date; hostname; uptime -p
echo
echo "== 拓扑 lsusb -t =="
lsusb -t 2>&1 | sed 's/^/  /'
echo
echo "== 设备节点 =="
for d in /dev/i2c-1 /dev/raspbot_lidar /dev/raspbot_imu /dev/raspbot_imu_uart /dev/video0 /dev/video1 /dev/hailo0; do
  if [ -e "$d" ]; then echo "  [OK ] $d -> $(readlink -f "$d") ($(stat -c '%A' "$d"))"; else echo "  [MISS] $d"; fi
done
echo
echo "== I2C 扫描 =="
i2cdetect -y 1 2>&1 | sed 's/^/  /'
echo
echo "== 磁盘 =="
df -h / /mnt/ssd 2>&1 | sed 's/^/  /'
echo
echo "== 内存/governor/温度 =="
free -h | head -2 | sed 's/^/  /'
echo "  governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
echo "  SoC: $(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 )) C"
echo "  throttled(vcgencmd): $(vcgencmd get_throttled 2>&1)"
echo
echo "== 残留进程检查（应为空）=="
pgrep -af 'raspbot|rplidar|slam_toolbox|nav2|component_container' | sed 's/^/  /' || true
echo "  (空=干净)"
echo
echo "== 参数 hash =="
for f in "$HOME/ros2_ws/src/raspbot_base/config/params.yaml" \
         "$HOME/ros2_ws/src/raspbot_imu/config/imu.yaml" \
         "$HOME/ros2_ws/src/raspbot_fusion/config/ekf.yaml" \
         "$HOME/ros2_ws/src/raspbot_fusion/config/limiter.yaml" \
         "$HOME/ros2_ws/src/raspbot_bringup/slam/slam_params.yaml" \
         "$HOME/ros2_ws/src/raspbot_bringup/nav2/nav2_params.yaml"; do
  if [ -f "$f" ]; then echo "  $(sha256sum "$f" | cut -c1-16)  $f"; else echo "  [MISS] $f"; fi
done
echo
echo "== ROS 工作区包 =="
ls "$HOME/ros2_ws/src" | sed 's/^/  /'
echo
echo "== 环境快照结束 =="
