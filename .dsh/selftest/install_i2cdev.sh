#!/usr/bin/env bash
LC_ALL=C
echo 'Abc123456' | sudo -S apt-get install -y libi2c-dev 2>&1 | tail -2
echo "== smbus.h =="
ls /usr/include/i2c/smbus.h 2>/dev/null
echo "== hailo lib =="
ls /opt/hailo/lib/libhailort.so* /usr/lib/aarch64-linux-gnu/libhailort.so* 2>/dev/null
find /opt/hailo /usr/lib -name 'libhailort.so*' 2>/dev/null | head -3
