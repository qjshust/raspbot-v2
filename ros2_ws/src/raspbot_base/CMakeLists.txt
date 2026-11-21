cmake_minimum_required(VERSION 3.8)
project(raspbot_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# ---------------------------------------------------------------- pure logic
# kinematics / odometry / sensors / safety: no ROS headers, offline testable.
add_library(raspbot_base_lib
  src/kinematics.cpp
  src/odometry.cpp
  src/sensors.cpp
  src/safety.cpp
)
target_include_directories(raspbot_base_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------- base node
add_executable(base_node
  src/base_node.cpp
  src/i2c_bus.cpp
)
target_link_libraries(base_node raspbot_base_lib)
# Lyrical 已移除 ament_target_dependencies，改用导入目标
target_link_libraries(base_node
  rclcpp::rclcpp
  std_msgs::std_msgs__rosidl_generator_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_generator_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  diagnostic_msgs::diagnostic_msgs__rosidl_generator_cpp
  diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
)

# ------------------------------------------------------------------ install
install(TARGETS raspbot_base_lib base_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# -------------------------------------------------------------------- tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics raspbot_base_lib)

  ament_add_gtest(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry raspbot_base_lib)

  ament_add_gtest(test_safety test/test_safety.cpp)
  target_link_libraries(test_safety raspbot_base_lib)

  ament_add_gtest(test_sensors test/test_sensors.cpp)
  target_link_libraries(test_sensors raspbot_base_lib)

  ament_add_gtest(test_component test/test_component.cpp)
  target_link_libraries(test_component raspbot_base_lib)
endif()

ament_package()
