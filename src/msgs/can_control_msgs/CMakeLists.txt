cmake_minimum_required(VERSION 3.8)
project(can_control_msgs)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 声明消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Autocontrol.msg"
  "msg/AutocontrolRadardata.msg"
  "msg/EightPoints.msg"
  "msg/Getmap.msg"
  "msg/MqttData.msg"
  "msg/Planningdata.msg"
  "msg/Radardate.msg"
  "msg/SingleObj.msg"
  "msg/SinglePoint.msg"
  "msg/VehicleStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 安装 msg 文件
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# 编译依赖项
ament_export_dependencies(rosidl_default_runtime)

ament_package()

