#!/usr/bin/env bash
LC_ALL=C
echo "== nav2_util TwistSubscriber / TwistPublisher 类型 =="
grep -rn 'TwistStamped\|Twist' /opt/ros/lyrical/include/nav2_util/nav2_util/twist_subscriber.hpp /opt/ros/lyrical/include/nav2_util/nav2_util/twist_publisher.hpp 2>/dev/null | grep -E 'class|using|typedef|msg::' | head -12
echo
echo "== collision_monitor 输入订阅类型 =="
grep -rn 'cmd_vel_in\|TwistSubscriber' /opt/ros/lyrical/include/nav2_collision_monitor/nav2_collision_monitor/collision_monitor_node.hpp | head -6
echo
echo "== 官方 1.5.2 谁订阅 cmd_vel（nav2_bringup 示例 base）=="
grep -rn 'TwistStamped' /opt/ros/lyrical/include/nav2_velocity_smoother/ 2>/dev/null | head -5
