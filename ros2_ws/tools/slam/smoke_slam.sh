#!/usr/bin/env bash
# RASPBOT-V2 · SLAM 冒烟测试（雷达 + IMU + slam_toolbox，不启动底盘）
#
# 定位：脱离最终系统的独立诊断工具（测试方案 TOOL-01 / SW-I02）。
#   最终系统里的 TF 全部由 E 提供（EKF 发布 odom->base_link、robot_state_publisher 发布
#   base_link->laser 等静态外参）；本脚本里的两个静态 TF 仅用于在没有 E 的情况下
#   单独验证 SLAM 链路，属于占位值，不代表真实外参（真实外参见 mech/README.md §7.4）。
#
# 用法（在 Pi 上）:
#     bash tools/slam/smoke_slam.sh
# 从 Windows 跑:
#     pwsh .dsh\pi.ps1 -Path tools\slam\smoke_slam.sh
#
# 它做什么：
#   1. 起两个静态 TF 占位：odom->base_link（真实值应由 EKF 发布）、base_link->laser（外参待实测）
#   2. 起 rplidar_node（C1, DenseBoost 460800）与 raspbot_imu（HWT101, 100 Hz）
#   3. 用 slam_params_smoke.yaml 起 slam_toolbox（注意 launch 参数名 slam_params_file）
#   4. 断言（SW-I02 / TOOL-01，任一失败则退出码非 0）：
#        a) /scan 频率 10±1 Hz
#        b) /imu/data 频率 100±10 Hz
#        c) /map Publisher count = 1
#        d) slam 日志 "Failed to compute odom pose" = 0 次
#   5. 输出 [PASS]/[FAIL] 汇总行；结束自动收尾（Ctrl-C 也收尾）
#
# 注意：
#   * 全程不发任何 /cmd_vel，轮子不会动；但雷达会转起来。
#   * 不要在这里开 set -u：ROS 的 setup.bash 会引用未定义变量，source 时直接退出。
LC_ALL=C

PARAMS="$(cd "$(dirname "$0")" && pwd)/slam_params_smoke.yaml"

. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"

PASSED=0
FAILED=0

# check <判定(0=过/非0=挂)> <断言名> <详情>
check() {
  if [ "$1" -eq 0 ]; then
    PASSED=$((PASSED + 1))
    echo "  [PASS] $2"
  else
    FAILED=$((FAILED + 1))
    echo "  [FAIL] $2  $3"
  fi
}

# topic_hz_average <话题> <采样秒数> -> stdout 打印 average rate 数值（无发布者时为空）
topic_hz_average() {
  timeout -s KILL "$2" ros2 topic hz "$1" 2>&1 \
    | grep 'average rate' | head -1 | awk '{print $3}'
}

cleanup() {
  pkill -f async_slam_toolbox_node 2>/dev/null
  pkill -f rplidar_node 2>/dev/null
  pkill -f imu_node 2>/dev/null
  pkill -f 'static_transform_publisher.*(odom|base_link)' 2>/dev/null
  sleep 2
}
trap cleanup EXIT INT TERM
cleanup

echo "== 参数文件: $PARAMS =="
[ -f "$PARAMS" ] || { echo "找不到参数文件，退出"; exit 1; }
grep -E 'base_frame|odom_frame|scan_topic|min_laser_range|max_laser_range' "$PARAMS" | sed 's/^/  /'

echo
echo "== 启动静态 TF / 雷达 / IMU =="
nohup ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
      --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 > /tmp/smoke_tf_odom.log 2>&1 &
nohup ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id laser \
      --x 0 --y 0.107 --z 0.034 --roll 0 --pitch 0 --yaw 0 > /tmp/smoke_tf_laser.log 2>&1 &
nohup ros2 run rplidar_ros rplidar_node --ros-args \
      -p serial_port:=/dev/raspbot_lidar -p serial_baudrate:=460800 \
      -p frame_id:=laser -p angle_compensate:=true -p scan_mode:=DenseBoost > /tmp/smoke_lidar.log 2>&1 &
nohup ros2 launch raspbot_imu imu.launch.py > /tmp/smoke_imu.log 2>&1 &
sleep 9

echo "== 启动 slam_toolbox（注意参数名 slam_params_file） =="
nohup ros2 launch slam_toolbox online_async_launch.py \
      use_sim_time:=false slam_params_file:="$PARAMS" > /tmp/smoke_slam.log 2>&1 &
sleep 14

echo
echo "== 节点 =="
timeout -s KILL 8 ros2 node list 2>&1 | sed 's/^/  /'

echo
echo "== 参数确认（应为 base_link / odom / False） =="
for p in base_frame odom_frame use_sim_time; do
  printf '  %-12s ' "$p"
  timeout -s KILL 6 ros2 param get /slam_toolbox "$p" 2>&1 | head -1
done

echo
echo "===== 断言区（SW-I02 / TOOL-01） ====="

# a) /scan 频率 10±1 Hz
SCAN_HZ="$(topic_hz_average /scan 8)"
echo "  /scan average rate: ${SCAN_HZ:-<无输出>} Hz"
if [ -z "$SCAN_HZ" ]; then
  check 1 "/scan 频率 10±1 Hz" "无发布者或测不出频率，查 /tmp/smoke_lidar.log"
else
  echo "$SCAN_HZ" | awk '{exit !($1>=9.0 && $1<=11.0)}'
  check $? "/scan 频率 10±1 Hz" "实测 ${SCAN_HZ} Hz"
fi

# b) /imu/data 频率 100±10 Hz
IMU_HZ="$(topic_hz_average /imu/data 8)"
echo "  /imu/data average rate: ${IMU_HZ:-<无输出>} Hz"
if [ -z "$IMU_HZ" ]; then
  check 1 "/imu/data 频率 100±10 Hz" "无发布者或测不出频率，查 /tmp/smoke_imu.log"
else
  echo "$IMU_HZ" | awk '{exit !($1>=90.0 && $1<=110.0)}'
  check $? "/imu/data 频率 100±10 Hz" "实测 ${IMU_HZ} Hz"
fi

# c) /map Publisher count = 1
MAP_INFO="$(timeout -s KILL 8 ros2 topic info /map 2>&1)"
echo "$MAP_INFO" | sed 's/^/  /'
echo "$MAP_INFO" | grep -q 'Publisher count: 1'
check $? "/map Publisher count = 1" "$(echo "$MAP_INFO" | grep 'Publisher count' | tr -s ' ')"
# 防御性检查：不允许出现多个发布者（契约：一个话题只能有一个发布者）
if echo "$MAP_INFO" | grep -q 'Publisher count: 0'; then
  echo "        （/map 无发布者：slam_toolbox 未出图，查 /tmp/smoke_slam.log）"
fi

# d) "Failed to compute odom pose" = 0 次
ODOM_FAILS=$(grep -c 'Failed to compute odom pose' /tmp/smoke_slam.log 2>/dev/null || true)
[ -n "$ODOM_FAILS" ] || ODOM_FAILS=0
[ "$ODOM_FAILS" -eq 0 ]
check $? '"Failed to compute odom pose" = 0 次' "实测 $ODOM_FAILS 次"

echo
echo "== 附加信息（不参与判定） =="
echo "== /map 内容（等最多 8 s） =="
timeout -s KILL 8 ros2 topic echo /map --once --field info 2>&1 | head -10 | sed 's/^/  /'

echo
echo "== map -> base_link =="
timeout -s KILL 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -E 'At time|Translation|Invalid' | head -3 | sed 's/^/  /'

echo
echo "== slam 日志（尾部） =="
tail -15 /tmp/smoke_slam.log 2>/dev/null | sed 's/^/  /'

echo
echo "=========================================="
echo "  冒烟结果汇总: PASS=$PASSED  FAIL=$FAILED"
if [ "$FAILED" -eq 0 ]; then
  echo "  [PASS] SLAM 冒烟全部断言通过"
  echo "  （收尾中，雷达即将停转）"
  exit 0
else
  echo "  [FAIL] 存在 $FAILED 项断言失败，退出码 1"
  echo "  （收尾中，雷达即将停转）"
  exit 1
fi
