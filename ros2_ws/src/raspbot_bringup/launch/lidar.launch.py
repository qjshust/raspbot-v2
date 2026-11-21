#!/usr/bin/env python3
"""RASPBOT-V2 WP-C 雷达 bringup。

节点拓扑（rear_cut_enabled=true，默认）：
    rplidar_node --scan_raw--> scan_to_scan_filter_chain --scan--> /scan
rear_cut_enabled=false 时跳过过滤链，rplidar_node 直接发布 /scan（单发布者不变）。

裁决/需求对应：
    J-01  frame_id=laser（参数文件 rplidar.yaml，绝不允许 laser_link）
    J-02  rplidar_ros 2.1.4（package/executable 名 rplidar_ros/rplidar_node）
    J-08  后向 ±15° 立柱遮挡滤除（laser_filters/LaserScanAngularBoundsFilter）
    FR-C-01  /scan 10 Hz±10%、量程 0.2~15 m（实测基线 10.0 Hz，硬件报告 §5.1）
    FR-C-04  lidar_params_file 参数作为室内/户外量程模板切换口
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('raspbot_bringup')
    rear_cut = LaunchConfiguration('rear_cut_enabled').perform(context).strip().lower()
    rear_cut_enabled = rear_cut in ('true', '1', 'yes', 'on')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nodes = []
    # 雷达驱动：默认 remap scan -> scan_raw；不过滤时直接发布 /scan
    lidar_remappings = [('scan', 'scan_raw')] if rear_cut_enabled else []
    nodes.append(Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[LaunchConfiguration('lidar_params_file'),
                    {'use_sim_time': use_sim_time}],
        remappings=lidar_remappings,
    ))

    if rear_cut_enabled:
        # laser_filters 过滤链（默认输入/输出话题 scan / scan_filtered，
        # 依据 scan_to_scan_filter_chain.cpp；订阅 SensorDataQoS）
        # 注意：该节点自带 diagnostics updater，默认直发 /diagnostics（破坏
        # SW-C04 单发布者），必须 remap 到私有话题 lidar_filter/diagnostics。
        nodes.append(Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            parameters=[os.path.join(pkg_share, 'lidar', 'rear_filter.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('scan', 'scan_raw'),
                        ('scan_filtered', 'scan'),
                        ('diagnostics', 'lidar_filter/diagnostics')],
        ))

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('raspbot_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'rear_cut_enabled', default_value='true',
            description='启用后向 ±15° 立柱遮挡滤除（J-08）；'
                        'false 时 /scan 为 rplidar 原始扫描（直通）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='bag 回放时设 true'),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=os.path.join(pkg_share, 'lidar', 'rplidar.yaml'),
            description='rplidar_node 参数文件（FR-C-04 室内/户外量程模板切换口）'),
        OpaqueFunction(function=launch_setup),
    ])
