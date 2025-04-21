cmake_minimum_required(VERSION 3.8)
project(radar_msgs)

# 使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找构建工具和依赖包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  DEPENDENCIES std_msgs
)

# 安装 msg 目录（可选）
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# 声明该包是 ROS 2 接口包
ament_export_dependencies(rosidl_default_runtime)
ament_package()

