"""RASPBOT-V2 总启动（WP-H）：一键 bringup（SYS-F01 / FR-H-01）。

launch 参数：
    mode           none | slam | nav（默认 none：仅遥控）
    use_mock       true 时用 mock 三件套（fake_base/fake_imu/fake_scan）
                   替代 A/B/C 硬件驱动（SW-I04）
    use_camera     true 时启动相机（D）
    use_perception true 时启动感知（I，需 use_camera）
    use_sim_time   true 时使用仿真时间（bag 回放配合 ros2 bag play --clock）
    map            导航模式的地图 yaml 路径（mode:=nav 时必填）

用法：
    ros2 launch raspbot_bringup bringup.launch.py
    ros2 launch raspbot_bringup bringup.launch.py mode:=slam
    ros2 launch raspbot_bringup bringup.launch.py mode:=nav map:=/mnt/ssd/maps/floor1.yaml
    ros2 launch raspbot_bringup bringup.launch.py use_mock:=true mode:=slam

引用文件（与其他 WP 交付一致，勿改名）：
    raspbot_description/launch/description.launch.py（robot_state_publisher，E）
    raspbot_base/launch/base.launch.py、raspbot_imu/launch/imu.launch.py
    raspbot_bringup/launch/lidar.launch.py（C）、camera.launch.py（D）
    raspbot_fusion/launch/fusion.launch.py（E：ekf_node + cmd_vel_limiter + odom_relay）
    raspbot_bringup/launch/slam.launch.py、localization.launch.py（F）
    raspbot_bringup/launch/nav2.launch.py（G，控制器输出 remap 到 /cmd_vel_nav）
    raspbot_bringup/launch/perception.launch.py（I，节点在 raspbot_perception 包）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

BRINGUP = 'raspbot_bringup'
MOCK = 'raspbot_mock'


def _include(pkg, *path, condition=None, launch_arguments=None):
    """IncludeLaunchDescription 包进 GroupAction(scoped=True)。

    每个被 include 的 launch 文件都在自己的参数作用域里声明 params_file /
    use_sim_time 等同名参数；scoped 组隔离后互不覆盖（否则同一 context 下
    先声明者全局生效）。传入的 launch_arguments 照常生效，If/UnlessCondition
    挂在 GroupAction 上。
    """
    return GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory(pkg), *path)),
                launch_arguments=launch_arguments,
            ),
        ],
        scoped=True,
        condition=condition,
    )


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    use_mock = LaunchConfiguration('use_mock')
    use_camera = LaunchConfiguration('use_camera')
    use_perception = LaunchConfiguration('use_perception')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    declared = [
        DeclareLaunchArgument(
            'mode', default_value='none', choices=['none', 'slam', 'nav'],
            description='运行模式：none=仅遥控，slam=建图，nav=定位+导航'),
        DeclareLaunchArgument(
            'use_mock', default_value='false',
            description='true 时用 mock 三件套替代硬件驱动（SW-I04）'),
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='启动相机（/image_raw /camera_info）'),
        DeclareLaunchArgument(
            'use_perception', default_value='false',
            description='启动感知（需 use_camera:=true）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='使用仿真时间（bag 回放）'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='导航模式地图 yaml 路径（mode:=nav 时必填）'),
    ]

    not_mock = UnlessCondition(use_mock)
    is_mock = IfCondition(use_mock)
    is_slam = IfCondition(PythonExpression(["'", mode, "' == 'slam'"]))
    is_nav = IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    is_mock_nav = IfCondition(PythonExpression(
        ["'", use_mock, "' == 'true' and '", mode, "' == 'nav'"]))

    # ---- 基础（始终启动）----
    description = _include(
        'raspbot_description', 'launch', 'description.launch.py',
        launch_arguments={'use_sim_time': use_sim_time}.items())
    twist_mux = _include(
        BRINGUP, 'system', 'twist_mux.launch.py',
        launch_arguments={'use_sim_time': use_sim_time}.items())
    safety = _include('raspbot_safety', 'launch', 'safety.launch.py')

    # ---- 驱动：真硬件 或 mock（SW-I04）----
    base = _include('raspbot_base', 'launch', 'base.launch.py',
                    condition=not_mock)
    imu = _include('raspbot_imu', 'launch', 'imu.launch.py',
                   condition=not_mock)
    lidar = _include(
        BRINGUP, 'launch', 'lidar.launch.py',
        condition=not_mock,
        launch_arguments={'use_sim_time': use_sim_time}.items())

    fake_base = Node(
        package=MOCK, executable='fake_base', name='fake_base',
        output='screen', condition=is_mock)
    fake_imu = Node(
        package=MOCK, executable='fake_imu', name='fake_imu',
        output='screen', condition=is_mock)
    fake_scan = Node(
        package=MOCK, executable='fake_scan', name='fake_scan',
        output='screen', condition=is_mock)

    # ---- 融合（始终：ekf_node + cmd_vel_limiter + odom_relay，E）----
    fusion = _include(
        'raspbot_fusion', 'launch', 'fusion.launch.py',
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # ---- 相机与感知（可选）----
    camera = _include(BRINGUP, 'launch', 'camera.launch.py',
                      condition=IfCondition(use_camera))
    perception = _include(
        BRINGUP, 'launch', 'perception.launch.py',
        condition=IfCondition(use_perception))

    # ---- 模式：slam / nav ----
    slam = _include(BRINGUP, 'launch', 'slam.launch.py', condition=is_slam)
    localization = _include(
        BRINGUP, 'launch', 'localization.launch.py',
        condition=is_nav,
        launch_arguments={'map': map_yaml}.items())
    # nav2 的 lifecycle manager 不 autostart：导航模式需要先发 /initialpose
    # （AMCL 定位）再按依赖顺序激活（tools/selftest/activate_nav2.sh），
    # 否则 manager 会因缺 map→odom 中止、并与外部激活脚本双写竞争（集成实测）。
    nav2 = _include(BRINGUP, 'launch', 'nav2.launch.py', condition=is_nav,
                    launch_arguments={'autostart': 'false'}.items())

    # use_mock + mode=nav：无真实地图源，localization 依赖静态地图
    mock_nav_warn = LogInfo(
        msg='[bringup] 注意：use_mock:=true 且 mode:=nav。mock 模式没有真车建图，'
            'localization 需要已保存的静态地图（map 参数）；否则仅做 slam 冒烟。',
        condition=is_mock_nav)

    return LaunchDescription([
        *declared,
        description,
        twist_mux,
        safety,
        base, imu, lidar,
        fake_base, fake_imu, fake_scan,
        fusion,
        camera, perception,
        slam, localization, nav2,
        mock_nav_warn,
    ])
