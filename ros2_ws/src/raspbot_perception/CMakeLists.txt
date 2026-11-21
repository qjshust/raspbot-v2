cmake_minimum_required(VERSION 3.8)
project(raspbot_perception)

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
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------------------
# HailoRT（可选依赖）：Pi 实测路径 include=/usr/include（hailo/hailort.hpp）、
# lib=/usr/lib/libhailort.so（4.23.0）。找不到则跳过 hailo_node 构建，
# 纯函数核心库与全部 gtest 不受影响。
# ---------------------------------------------------------------------------
find_path(HAILORT_INCLUDE_DIR NAMES hailo/hailort.hpp
  HINTS /opt/hailo/include /usr/local/include /usr/include)
find_library(HAILORT_LIBRARY NAMES hailort
  HINTS /opt/hailo/lib /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu)

# ---------------------------------------------------------------------------
# 核心库（纯函数：letterbox / classes / nms / image_utils，无 NPU 依赖）
# ---------------------------------------------------------------------------
add_library(raspbot_perception_core STATIC
  src/letterbox.cpp
  src/classes.cpp
  src/nms.cpp
  src/image_utils.cpp
)
target_include_directories(raspbot_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---------------------------------------------------------------------------
# hailo_node（需要 HailoRT）
# ---------------------------------------------------------------------------
if(HAILORT_INCLUDE_DIR AND HAILORT_LIBRARY)
  message(STATUS "HailoRT found: include=${HAILORT_INCLUDE_DIR} lib=${HAILORT_LIBRARY}")
  add_library(hailo_backend STATIC
    src/hailo_backend.cpp
  )
  # HailoRT 头目录 PUBLIC 传播给 hailo_node（hailo_backend.hpp 包含 hailort.hpp）
  target_include_directories(hailo_backend PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    SYSTEM ${HAILORT_INCLUDE_DIR})

  add_executable(hailo_node src/hailo_node.cpp)
  target_link_libraries(hailo_node raspbot_perception_core hailo_backend
    ${HAILORT_LIBRARY})
  # Lyrical 已移除 ament_target_dependencies，改用导入目标
  target_link_libraries(hailo_node
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    vision_msgs::vision_msgs__rosidl_generator_cpp
    vision_msgs::vision_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_generator_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
  )

  install(TARGETS hailo_node
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "HailoRT 未找到（hailo/hailort.hpp / libhailort）：跳过 hailo_node 构建。"
    " 可在 CMake 缓存中手动指定 HAILORT_INCLUDE_DIR/HAILORT_LIBRARY。")
endif()

# ---------------------------------------------------------------------------
# 安装
# ---------------------------------------------------------------------------
install(TARGETS raspbot_perception_core
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch tools
  DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# 测试（不依赖 NPU；Python tests/*.py 原样保留为参考规格）
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 23 用例 1:1 映射 Python 版 tests/test_letterbox.py(9)/test_postprocess.py(10)/
  # test_classes.py(4)
  ament_add_gtest(test_letterbox tests/test_letterbox.cpp)
  target_link_libraries(test_letterbox raspbot_perception_core)

  ament_add_gtest(test_nms tests/test_nms.cpp)
  target_link_libraries(test_nms raspbot_perception_core)

  ament_add_gtest(test_classes tests/test_classes.cpp)
  target_link_libraries(test_classes raspbot_perception_core)

  # C++ 新增功能附加覆盖：YUYV->RGB、点阵标注（Python 版无对应入口）
  ament_add_gtest(test_yuyv tests/test_yuyv.cpp)
  target_link_libraries(test_yuyv raspbot_perception_core)

  ament_add_gtest(test_draw tests/test_draw.cpp)
  target_link_libraries(test_draw raspbot_perception_core)
endif()

ament_package()
