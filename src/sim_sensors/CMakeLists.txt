cmake_minimum_required(VERSION 3.8)
project(sim_sensors)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lidar_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sim_sensors src/sim_sensors.cpp)
ament_target_dependencies(sim_sensors rclcpp sensor_msgs lidar_msgs std_msgs)

install(TARGETS sim_sensors
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
