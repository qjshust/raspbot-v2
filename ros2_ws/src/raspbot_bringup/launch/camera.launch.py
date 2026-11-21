#!/usr/bin/env python3
"""RASPBOT-V2 WP-D 相机 bringup：v4l2_camera_node → /image_raw + /camera_info。

需求/裁决对应：
    FR-D-01  /image_raw + /camera_info（内参随图像发布，camera_info_url 指向
             camera/intrinsics.yaml）
    J-03     契约 15 Hz（framerate 键在 camera.yaml 中记录；v4l2_camera 0.8.1
             无 framerate 参数，实际频率由设备决定，见 camera/README.md）
    SW-C03   frame_id=camera_link（参数 camera_frame_id）
    FR-D-03  分辨率可配（camera_params_file 参数覆盖，改 intrinsics.yaml 后生效）

image_transport 压缩转发（republish）默认注释，原因：
    - Pi 上未安装 image_transport_plugins（/opt/ros/lyrical/share 下无
      compressed_image_transport，证据见硬件部署检查）；
    - raspbot_bringup/package.xml 未声明该依赖（属 lead 维护，本 WP 不能改）。
    启用需：1) 安装 ros-lyrical-image-transport-plugins；
            2) lead 在 package.xml 增补 <exec_depend>image_transport_plugins</exec_depend>。
    注：/image_raw 为 rgb8 未压缩（camera.yaml output_encoding: rgb8），
        仅跨机低带宽传输场景才有必要启用压缩转发，本车本地消费默认不需要。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('raspbot_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='bag 回放时设 true'),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=os.path.join(pkg_share, 'camera', 'camera.yaml'),
            description='v4l2_camera 参数文件（FR-D-03 分辨率可配的切换口）'),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[LaunchConfiguration('camera_params_file'),
                        {'use_sim_time': use_sim_time}],
        ),

        # ---- 可选：raw -> compressed 转发（默认注释，见模块 docstring）----
        # Node(
        #     package='image_transport',
        #     executable='republish',
        #     name='image_republish',
        #     arguments=['raw', 'compressed'],
        #     remappings=[('in', '/image_raw'), ('out', '/image_raw/compressed')],
        # ),
    ])
