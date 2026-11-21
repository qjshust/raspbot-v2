#!/usr/bin/env bash
# 00_pytests.sh —— 全部包离线单测（colcon test + 各包 pytest）
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
cd "$HOME/ros2_ws"
FAILS=0

echo "== colcon test（仅测带 test 的包，强制 pytest provider）=="
colcon test --python-testing pytest --packages-select \
  raspbot_base raspbot_imu raspbot_fusion raspbot_safety raspbot_mock raspbot_perception \
  --event-handlers console_direct+ 2>&1 | tail -25 | sed 's/^/  /'
echo
echo "== colcon test-result =="
colcon test-result --verbose 2>&1 | sed 's/^/  /' || FAILS=1

echo
echo "== 各包 pytest（源目录直跑，便于单测细节）=="
for pkg in raspbot_base raspbot_imu raspbot_fusion raspbot_safety raspbot_mock raspbot_perception; do
  d="src/$pkg"
  echo "  --- $pkg ---"
  if [ -d "$d/test" ] || [ -d "$d/tests" ]; then
    ( cd "$d" && PYTHONPATH="$PWD:${PYTHONPATH:-}" \
      python3 -m pytest -q 2>&1 | tail -6 | sed 's/^/  /' )
    [ ${PIPESTATUS[0]} -eq 0 ] || FAILS=1
  else
    echo "    （无 test 目录，跳过）"
  fi
done

echo
if [ $FAILS -eq 0 ]; then echo "[PASS] 00_pytests 全部通过"; else echo "[FAIL] 00_pytests 有失败"; exit 1; fi
