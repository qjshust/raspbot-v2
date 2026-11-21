#!/usr/bin/env bash
# cpp_clean_build.sh —— C++ 转换后首次全量干净构建 + 测试
LC_ALL=C
set -e
. /opt/ros/lyrical/setup.bash
echo "== 清理转换包旧文件 =="
cd ~/ros2_ws
for p in raspbot_base raspbot_imu raspbot_fusion raspbot_safety raspbot_mock raspbot_perception; do
  rm -rf src/$p
done
rm -rf build install log
echo "== 解包（tar 根为 home）=="
cd ~
tar xzf /tmp/raspbot_deploy.tgz
rm -rf ~/ros2_ws/ros2_ws
cd ~/ros2_ws
echo "== 转换包状态 =="
for p in raspbot_base raspbot_imu raspbot_fusion raspbot_safety raspbot_mock raspbot_perception; do
  echo "  $p: $(ls src/$p | tr '\n' ' ')"
done
echo "== colcon build（全量）=="
colcon build --symlink-install > /tmp/cpp_build.log 2>&1
RC=$?
tail -50 /tmp/cpp_build.log
echo "BUILD_RC=$RC"
exit $RC
