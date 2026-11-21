cmake_minimum_required(VERSION 3.8)
project(raspbot_fusion)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# ---------------------------------------------------------------------------
# 纯逻辑限速器库（无 ROS 依赖，离线可测；供节点与 gtest 共用）
# ---------------------------------------------------------------------------
add_library(raspbot_fusion_lib src/limiter.cpp)
target_include_directories(raspbot_fusion_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---------------------------------------------------------------------------
# cmd_vel_limiter —— /cmd_vel_safe 唯一发布者
# ---------------------------------------------------------------------------
add_executable(cmd_vel_limiter src/cmd_vel_limiter.cpp)
target_link_libraries(cmd_vel_limiter raspbot_fusion_lib)
# Lyrical 已移除 ament_target_dependencies，改用导入目标
target_link_libraries(cmd_vel_limiter
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_generator_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
)

# ---------------------------------------------------------------------------
# odom_relay —— /odom → /odometry/filtered 轻量转发（FR-E-05 P2）
# ---------------------------------------------------------------------------
add_executable(odom_relay src/odom_relay.cpp)
target_link_libraries(odom_relay
  rclcpp::rclcpp
  nav_msgs::nav_msgs__rosidl_generator_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
)

install(TARGETS cmd_vel_limiter odom_relay
  DESTINATION lib/${PROJECT_NAME})

# config（ekf.yaml 供 robot_localization、limiter.yaml 供 cmd_vel_limiter）
# 与 launch 原样安装到 share/。
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES resource/raspbot_fusion
  DESTINATION share/ament_index/resource_index/packages)

# ---------------------------------------------------------------------------
# 测试：tests/test_limiter.cpp（与 tests/test_limiter.py 33 用例 1:1）
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_limiter tests/test_limiter.cpp)
  if(TARGET test_limiter)
    target_link_libraries(test_limiter raspbot_fusion_lib)
  endif()
endif()

ament_package()
