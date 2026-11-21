cmake_minimum_required(VERSION 3.8)
project(raspbot_imu)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# --- 核心库（无 ROS 依赖：四元数/姿态/HWT101 帧解析/ICM 解析/消息组装/配置校验） ---
add_library(raspbot_imu_core STATIC
  src/quaternion.cpp
  src/attitude.cpp
  src/witmotion.cpp
  src/icm42688.cpp
  src/msgs.cpp
  src/config.cpp
  src/mock_source.cpp
)
target_include_directories(raspbot_imu_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- 设备层（termios 串口 + spidev，无 ROS 依赖） ---
add_library(raspbot_imu_sources STATIC src/sources.cpp)
target_link_libraries(raspbot_imu_sources PUBLIC raspbot_imu_core)
target_include_directories(raspbot_imu_sources PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- 节点可执行文件（名字不变：imu_node） ---
add_executable(imu_node src/node.cpp)
target_link_libraries(imu_node
  raspbot_imu_sources
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
)

install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

# --- 单元测试（1:1 映射 Python 版 43 用例） ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_attitude test/test_attitude.cpp)
  target_link_libraries(test_attitude raspbot_imu_core)

  ament_add_gtest(test_icm42688 test/test_icm42688.cpp)
  target_link_libraries(test_icm42688 raspbot_imu_core)

  ament_add_gtest(test_sources test/test_sources.cpp)
  target_link_libraries(test_sources raspbot_imu_sources)

  ament_add_gtest(test_msgs test/test_msgs.cpp)
  target_link_libraries(test_msgs raspbot_imu_core)

  ament_add_gtest(test_config test/test_config.cpp)
  target_link_libraries(test_config raspbot_imu_core)
endif()

ament_package()
