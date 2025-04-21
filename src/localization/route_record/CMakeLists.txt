cmake_minimum_required(VERSION 3.8)
project(route_record)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(can_control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 

include_directories(
  include
)

add_executable(slam_route_record_trans_node src/slam_route_record_trans.cpp)
ament_target_dependencies(slam_route_record_trans_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs 
  localization_msgs
  can_control_msgs
)

add_executable(gps_route_record_trans_node src/gps_route_record_trans.cpp)
ament_target_dependencies(gps_route_record_trans_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  localization_msgs
  tf2_geometry_msgs 
  can_control_msgs
)

install(TARGETS
  slam_route_record_trans_node
  gps_route_record_trans_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
