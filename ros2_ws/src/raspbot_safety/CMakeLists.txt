cmake_minimum_required(VERSION 3.8)
project(raspbot_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 纯解析/阈值库（无 ROS 依赖，离线 gtest 可测）
add_library(raspbot_safety_utils
  src/sysinfo.cpp
  src/thresholds.cpp)
target_include_directories(raspbot_safety_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---- safety_node：1 Hz 发布 /diagnostics（自身状态 + 聚合 + 安全告警）----
add_executable(safety_node src/safety_node.cpp)
target_link_libraries(safety_node
  rclcpp::rclcpp
  diagnostic_msgs::diagnostic_msgs__rosidl_generator_cpp
  diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_generator_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_generator_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
)
target_link_libraries(safety_node raspbot_safety_utils)

install(TARGETS
  safety_node raspbot_safety_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp diagnostic_msgs sensor_msgs nav_msgs std_msgs)
ament_export_include_directories(include)
ament_export_libraries(raspbot_safety_utils)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 1:1 映射 legacy Python 用例：tests/test_sysinfo.py（17 用例）
  ament_add_gtest(test_sysinfo tests/test_sysinfo.cpp)
  target_link_libraries(test_sysinfo raspbot_safety_utils)
  # 1:1 映射 legacy Python 用例：tests/test_thresholds.py（8 用例）
  ament_add_gtest(test_thresholds tests/test_thresholds.cpp)
  target_link_libraries(test_thresholds raspbot_safety_utils)
endif()

ament_package()
