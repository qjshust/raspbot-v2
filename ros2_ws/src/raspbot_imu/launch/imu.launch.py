"""启动 IMU 节点，参数取自 config/imu.yaml。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory('raspbot_imu'), 'config', 'imu.yaml'
    )
    return LaunchDescription(
        [
            Node(
                package='raspbot_imu',
                executable='imu_node',
                name='imu_node',
                output='screen',
                parameters=[params],
            )
        ]
    )
