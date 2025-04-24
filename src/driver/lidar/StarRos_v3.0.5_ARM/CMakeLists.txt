cmake_minimum_required(VERSION 3.8)
project(rfans_driver LANGUAGES C CXX)

# ① 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ② ROS 2 & 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(message_filters REQUIRED)

pkg_check_modules(PCAP REQUIRED libpcap)

# ③ 接口文件生成（msg/srv）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RfansPacket.msg"
  "msg/Packet.msg"
  "msg/RfansScan.msg"
  "msg/Command.msg"
  "msg/ImuData.msg"
  "msg/HeartMsg.msg"
  "srv/RfansCommand.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# 导出编译依赖
ament_export_dependencies(rosidl_default_runtime)

# ④ include 路径
include_directories(
  include
  ${PCAP_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/lib"
  "/opt/ros/humble/include"
)
#气笑了
link_directories(
  "${PROJECT_SOURCE_DIR}/lib"
  ${PCAP_LIBRARY_DIRS}
)

set(LIB_GATHER
  LidarCalc
  LidarCFG
  LidarDecoder
  LidarIO
)

# ⑤ 可执行文件定义
add_executable(driver_node
  src/publisher.cpp
  src/ioapi.cpp
  src/rfans_driver.cpp
  src/ssFrameLib.cpp
  src/common.cpp
)
ament_target_dependencies(driver_node rclcpp std_msgs geometry_msgs sensor_msgs message_filters)
target_link_libraries(driver_node ${LIB_GATHER} ${PCAP_LIBRARIES} Qt5::Widgets)
rosidl_target_interfaces(driver_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(heart_node
  src/heart_node.cpp
  src/ioapi.cpp
  src/ssFrameLib.cpp
  src/common.cpp
)
ament_target_dependencies(heart_node rclcpp std_msgs geometry_msgs message_filters)
target_link_libraries(heart_node ${LIB_GATHER} ${PCAP_LIBRARIES})
rosidl_target_interfaces(heart_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(u_coordinate
  src/u_coordinate.cpp
  src/ioapi.cpp
  src/ssFrameLib.cpp
)
ament_target_dependencies(u_coordinate rclcpp geometry_msgs message_filters)
target_link_libraries(u_coordinate ${LIB_GATHER} ${PCAP_LIBRARIES})
rosidl_target_interfaces(u_coordinate ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ⑥ RViz 插件
add_library(rviz_teleop_commander SHARED src/teleop_pad.cpp)
ament_target_dependencies(rviz_teleop_commander rviz_common geometry_msgs message_filters)
target_link_libraries(rviz_teleop_commander ${LIB_GATHER} Qt5::Widgets)
rosidl_target_interfaces(rviz_teleop_commander ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ⑦ 安装
install(TARGETS
  driver_node
  heart_node
  u_coordinate
  rviz_teleop_commander
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch cfg msg srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
