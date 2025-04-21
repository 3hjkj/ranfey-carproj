# ✅ ROS 2 CMakeLists.txt (for package `control_node`)
cmake_minimum_required(VERSION 3.8)
project(control_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(decision_planning_msgs REQUIRED)
find_package(can_control_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# 添加你的自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Autocontrol.msg"
  "msg/AutocontrolRadardata.msg"
  "msg/Getmap.msg"
  "msg/MqttData.msg"
  "msg/Planningdata.msg"
  "msg/Radardate.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(control_node_exec
  src/Datainput.cpp
  src/rt_nonfinite.cpp
  src/can_com_pub.cpp
  src/AutoVehicleControl.cpp
  src/AutoVehicleControl_data.cpp
  src/rtGetNaN.cpp
  src/rtGetInf.cpp
  src/RoutePlanning.cpp
)

ament_target_dependencies(control_node_exec
  rclcpp
  std_msgs
  geometry_msgs
  decision_planning_msgs
  can_control_msgs
  localization_msgs
  tf2
  tf2_geometry_msgs
)

# 链接 .so 动态库（如果用到了）
# target_link_libraries(control_node_exec
#   ${CMAKE_CURRENT_SOURCE_DIR}/lib/libOdrManager.so
# )

# find_library(ODR_LIB
#   NAMES OdrManager
#   PATHS
#     ${CMAKE_CURRENT_SOURCE_DIR}/can_com/lib      # 你放置 aarch64 版 .so 的目录
#     /usr/local/lib                               # 或系统安装路径
# )

# if(ODR_LIB)
#   message(STATUS "Found OdrManager: ${ODR_LIB}")
#   target_link_libraries(control_node_exec ${ODR_LIB})
# else()
#   message(WARNING "OdrManager not found or incompatible; skipping link")
# endif()



install(TARGETS control_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  can_start.sh
  can_down.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
