#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
cd ~/ros2_ws
echo "== colcon test（C++ gtest；仅 6 个 C++ 包，bringup/description 为纯数据包无测试）=="
colcon test --packages-select raspbot_base raspbot_imu raspbot_fusion raspbot_safety raspbot_mock raspbot_perception > /tmp/cpp_test.log 2>&1
RC=$?
tail -25 /tmp/cpp_test.log
echo "TEST_RC=$RC"
echo "== 详细结果 =="
colcon test-result --verbose 2>&1 | tail -40
