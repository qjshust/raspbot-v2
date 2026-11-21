#!/usr/bin/env bash
LC_ALL=C
echo "== 编译依赖检查 =="
for f in /usr/include/i2c/smbus.h /usr/include/arm-linux-gnueabihf/i2c/smbus.h /usr/include/yaml-cpp/yaml.h; do
  [ -e "$f" ] && echo "  OK  $f" || echo "  MISS $f"
done
find / -name 'smbus.h' -path '*i2c*' 2>/dev/null | head -3
echo "== hailo 头/库 =="
ls /opt/hailo/include/hailo/hailort.hpp 2>/dev/null && echo "  OK hailort.hpp"
ls /opt/hailo/lib/libhailort.so* 2>/dev/null | head -2
find / -name 'hailort.hpp' 2>/dev/null | head -2
echo "== vision_msgs =="
ls /opt/ros/lyrical/share/vision_msgs 2>/dev/null | head -3
echo "== apt 可用性 =="
apt-cache policy libi2c-dev libyaml-cpp-dev 2>/dev/null | grep -E 'libi2c|libyaml|Candidate' | head -4
