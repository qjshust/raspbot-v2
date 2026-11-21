"""启动 robot_state_publisher，从包内 URDF 发布全车静态 TF。

用法：
    ros2 launch raspbot_description description.launch.py
    ros2 launch raspbot_description description.launch.py use_sim_time:=true

说明（CONTRACTS §3）：
- 本 launch 是 base_link → laser / imu_link / camera_link / ultrasonic /
  base_footprint 全部静态变换的唯一发布者（robot_state_publisher 读 URDF）；
- 禁止 static_transform_publisher 重复发布这些变换；
- odom → base_link 由 ekf_node（raspbot_fusion）发布，不在此列。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('raspbot_description')
    urdf_path = os.path.join(share, 'urdf', 'raspbot.urdf')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (bag) time instead of wall time')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([declare_use_sim_time, robot_state_publisher])
