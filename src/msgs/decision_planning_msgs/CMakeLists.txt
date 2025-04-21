cmake_minimum_required(VERSION 3.8)
project(decision_planning_msgs)

# 编译要求
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 声明消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DecisionPlanning.msg"
  DEPENDENCIES std_msgs
)

# 安装消息文件（可选）
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 package.xml 所需元信息
ament_export_dependencies(rosidl_default_runtime)
ament_package()

