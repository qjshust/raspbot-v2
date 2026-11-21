#!/usr/bin/env bash
# activate_nav2.sh —— 初始位姿发布后，把未激活的 Nav2 节点按依赖顺序补激活。
# 关键顺序：planner 激活后必须等其 compute_path_to_pose action server 被发现，
# 再激活 bt_navigator（其 BT 加载只有 1 s 等待窗口，DDS 发现慢会失败）。
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
rc=0

ensure_active() {
  local n=$1
  local ST
  ST=$(timeout -s KILL 8 ros2 lifecycle get "/$n" 2>/dev/null | grep -oE 'active \[3\]')
  if [ -n "$ST" ]; then echo "  [OK] /$n 已激活"; return 0; fi
  # 真硬件负载下代价图 configure/activate 可能超过单次 CLI 窗口，逐次重试
  for attempt in 1 2 3; do
    timeout -s KILL 20 ros2 lifecycle set "/$n" configure >/dev/null 2>&1
    timeout -s KILL 30 ros2 lifecycle set "/$n" activate >/dev/null 2>&1
    for poll in 1 2 3; do
      ST=$(timeout -s KILL 8 ros2 lifecycle get "/$n" 2>/dev/null | grep -oE 'active \[3\]')
      [ -n "$ST" ] && break
      sleep 5
    done
    if [ -n "$ST" ]; then echo "  [OK] /$n 已激活（第 $attempt 次）"; return 0; fi
    echo "  [重试] /$n 激活未确认，等 8 s（$attempt/3）"
    sleep 8
  done
  echo "  [FAIL] /$n 未能激活"
  return 1
}

echo "  == 先关闭 lifecycle_manager_nav2（消除与本脚本的双写竞争）=="
timeout -s KILL 15 ros2 lifecycle set /lifecycle_manager_nav2 shutdown >/dev/null 2>&1
sleep 3

ensure_active controller_server || rc=1
ensure_active planner_server || rc=1
echo "  == 等 10 s 让 bt_navigator 的 DDS 发现完成（其 BT 加载只等 action 1 s）=="
sleep 10
ensure_active behavior_server || rc=1
if ! ensure_active bt_navigator; then
  for attempt in 1 2 3; do
    echo "  == bt_navigator 激活失败，等 10 s 重试（$attempt/3）=="
    sleep 10
    if ensure_active bt_navigator; then break; fi
    if [ $attempt -eq 3 ]; then rc=1; fi
  done
fi
ensure_active collision_monitor || rc=1
exit $rc
