#!/usr/bin/env bash
# 08_hailo.sh —— Hailo-8 感知节点实机验证（相机 + NPU）
# 起 camera + perception，验证 /detections、/perception/image、推理帧率。
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
FAILS=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }

cleanup() {
  pkill -f 'perception.launch.py' 2>/dev/null
  pkill -f hailo_node 2>/dev/null
  pkill -f 'camera.launch.py' 2>/dev/null
  pkill -f v4l2_camera_node 2>/dev/null
  sleep 2
}
trap cleanup EXIT INT TERM
cleanup

echo "== /dev/hailo0 与驱动 =="
ls -l /dev/hailo0 | sed 's/^/  /'
hailortcli fw-control identify 2>/dev/null | grep -E 'Firmware|Board' | sed 's/^/  /'

echo
echo "== 启动 camera + perception =="
nohup ros2 launch raspbot_bringup camera.launch.py > /tmp/st_cam.log 2>&1 &
sleep 6
nohup ros2 launch raspbot_bringup perception.launch.py > /tmp/st_hailo.log 2>&1 &
sleep 12

echo "== 节点 =="
timeout -s KILL 8 ros2 node list 2>&1 | sed 's/^/  /'

echo "== /detections 频率与内容 =="
timeout -s KILL 8 ros2 topic hz /detections 2>&1 | grep 'average rate' | sed 's/^/  /'
DET=$(timeout -s KILL 8 ros2 topic echo /detections --once 2>&1 | head -20)
echo "$DET" | sed 's/^/  /'
echo "$DET" | grep -q 'frame_id: camera_link' && pass "/detections frame_id=camera_link" || fail "/detections frame_id 错误"

echo "== /perception/image =="
timeout -s KILL 8 ros2 topic hz /perception/image 2>&1 | grep 'average rate' | sed 's/^/  /'

echo "== 抓一张标注帧 =="
python3 "$HOME/tools/selftest/save_image.py" /perception/image /tmp/st_perception_frame.jpg --timeout 25 2>&1 | sed 's/^/  /'
if [ -s /tmp/st_perception_frame.jpg ]; then
  ls -l /tmp/st_perception_frame.jpg | sed 's/^/  /'
  pass "标注帧已保存（回传后人工查看）"
else
  fail "标注帧抓取失败"
fi

echo "== hailo 节点日志（含 FPS）=="
tail -12 /tmp/st_hailo.log | sed 's/^/  /'

echo
if [ $FAILS -eq 0 ]; then echo "[PASS] 08_hailo 全部通过"; else echo "[FAIL] 08_hailo 有 $FAILS 项失败"; exit 1; fi
