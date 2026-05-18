cmake_minimum_required(VERSION 3.8)
project(robot_lidar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lidar_driver_node
  src/robot_lidar_driver/serial_port.cpp
  src/robot_lidar_driver/epoll_serial_reader.cpp
  src/robot_lidar_driver/lds03_parser.cpp
  src/robot_lidar_driver/laser_scan_builder.cpp
  src/robot_lidar_driver/lidar_driver_node.cpp
  src/main.cpp
)

target_include_directories(lidar_driver_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lidar_driver_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS lidar_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
