"""WP-E 融合启动：ekf_node + cmd_vel_limiter + odom_relay。

用法：
    ros2 launch raspbot_fusion fusion.launch.py
    ros2 launch raspbot_fusion fusion.launch.py use_sim_time:=true

职责（CONTRACTS v1.2）：
- ekf_node：/odom 与 odom→base_link TF 的唯一发布者（remap odometry/filtered→odom）
- cmd_vel_limiter：/cmd_vel_safe 的唯一发布者
- odom_relay：/odom → /odometry/filtered 轻量转发（FR-E-05 P2）
前置：raspbot_description 的 robot_state_publisher（base_link→imu_link TF）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('raspbot_fusion')
    ekf_params = os.path.join(share, 'config', 'ekf.yaml')
    limiter_params = os.path.join(share, 'config', 'limiter.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (bag) time instead of wall time')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',  # 与 ekf.yaml 顶层键一致（无命名空间）
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        # 唯一 /odom 发布者（r_l 默认输出 odometry/filtered）
        remappings=[
            ('odometry/filtered', 'odom'),
            # r_l 内部固定注册 /diagnostics 发布器（print_diagnostics: false
            # 只停止发布、不撤销注册，仍会与 H 的聚合节点冲突/SW-C04），
            # 重映射到私有话题规避单发布者冲突。
            ('diagnostics', 'ekf/diagnostics'),
        ],
    )

    cmd_vel_limiter = Node(
        package='raspbot_fusion',
        executable='cmd_vel_limiter',
        name='cmd_vel_limiter',  # 与 limiter.yaml 顶层键一致
        output='screen',
        parameters=[limiter_params, {'use_sim_time': use_sim_time}],
    )

    odom_relay = Node(
        package='raspbot_fusion',
        executable='odom_relay',
        name='odom_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [declare_use_sim_time, ekf_node, cmd_vel_limiter, odom_relay])
