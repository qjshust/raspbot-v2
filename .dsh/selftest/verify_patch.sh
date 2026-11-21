#!/usr/bin/env bash
LC_ALL=C
echo "== 远端 tgz 大小/时间 =="
ls -l /tmp/raspbot_deploy.tgz
echo "== 远端 tgz 内 base CMakeLists 38-46 行 =="
rm -rf /tmp/ck && mkdir -p /tmp/ck
tar xzf /tmp/raspbot_deploy.tgz -C /tmp/ck ros2_ws/src/raspbot_base/CMakeLists.txt
sed -n '38,46p' /tmp/ck/ros2_ws/src/raspbot_base/CMakeLists.txt
echo "== 实际 src 文件同段 =="
sed -n '38,46p' ~/ros2_ws/src/raspbot_base/CMakeLists.txt
