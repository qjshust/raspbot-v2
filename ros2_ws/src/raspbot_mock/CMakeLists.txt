cmake_minimum_required(VERSION 3.8)
project(raspbot_mock)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
# TODO(WP-H/A 对齐): A 交付 raspbot_base C++ kinematics 库后启用：
#   find_package(raspbot_base REQUIRED)
# 并把 fake_base.cpp 的 #include "raspbot_mock/kinematics.hpp" 换成
# <raspbot_base/kinematics.hpp>（接口签名保持一致，见 kinematics.hpp 头注释）。

# 纯几何/运动学库（无 ROS 依赖，离线 gtest 可测）
add_library(raspbot_mock_utils
  src/geometry.cpp
  src/kinematics.cpp)
target_include_directories(raspbot_mock_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---- fake_base：/cmd_vel_safe -> /odom_raw（30 Hz，半隐式积分）----
add_executable(fake_base src/fake_base.cpp)
target_link_libraries(fake_base raspbot_mock_utils)
# Lyrical 已移除 ament_target_dependencies，改用导入目标
target_link_libraries(fake_base
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_generator_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  tf2_ros::tf2_ros)

# ---- fake_imu：/imu/data（100 Hz，SensorData QoS）----
add_executable(fake_imu src/fake_imu.cpp)
target_link_libraries(fake_imu
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)

# ---- fake_scan：/scan（10 Hz，5x4 m 合成房间，SensorData QoS）----
add_executable(fake_scan src/fake_scan.cpp)
target_link_libraries(fake_scan raspbot_mock_utils)
target_link_libraries(fake_scan
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)

install(TARGETS
  fake_base fake_imu fake_scan raspbot_mock_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros)
ament_export_include_directories(include)
ament_export_libraries(raspbot_mock_utils)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 1:1 映射 legacy Python 用例 tests/test_geometry.py（5 用例）
  ament_add_gtest(test_geometry tests/test_geometry.cpp)
  target_link_libraries(test_geometry raspbot_mock_utils)
  # 运动学补充用例（Python 版无对应测试，C++ 版新增）
  ament_add_gtest(test_kinematics tests/test_kinematics.cpp)
  target_link_libraries(test_kinematics raspbot_mock_utils)
endif()

ament_package()
