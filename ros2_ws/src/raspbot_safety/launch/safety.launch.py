"""启动安全监控节点，参数取自 config/safety.yaml。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('raspbot_safety')
    default_params = os.path.join(share, 'config', 'safety.yaml')

    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML file with raspbot_safety parameters')

    node = Node(
        package='raspbot_safety',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([declare_params, node])
