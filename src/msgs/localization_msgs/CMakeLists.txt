cmake_minimum_required(VERSION 3.8)
project(localization_msgs)

# 编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖项
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 接口定义（添加 msg 文件）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Xyz.msg"
  "msg/PoseAngle.msg"
  "msg/GPS.msg"
  "msg/Time.msg"
  "msg/Localization.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 安装 msg 文件（可选）
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# 作为 interface package 声明
ament_export_dependencies(rosidl_default_runtime)

# 结束构建
ament_package()

