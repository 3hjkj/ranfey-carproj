cmake_minimum_required(VERSION 3.8)
project(lidar_msgs)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 添加消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cell.msg"
  "msg/Cells.msg"
  "msg/Object.msg"
  "msg/Objects.msg"
  "msg/VisionBox.msg"
  "msg/VisionBoxes.msg"
  DEPENDENCIES std_msgs
)

# 安装消息目录
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# ROS 2 特有导出
ament_export_dependencies(rosidl_default_runtime)
ament_package()

