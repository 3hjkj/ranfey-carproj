cmake_minimum_required(VERSION 3.8)
project(radar_adapter)

# 使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(can_control_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(radar_adapter src/radar_adapter.cpp)
ament_target_dependencies(radar_adapter rclcpp radar_msgs can_control_msgs std_msgs)

install(TARGETS radar_adapter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
