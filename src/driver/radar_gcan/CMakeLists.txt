cmake_minimum_required(VERSION 3.5)
project(radar_gcan)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 安装 Python 节点。源文件不需要可执行位，install(PROGRAMS) 会在安装时补上，
# 但脚本必须带 shebang —— launch 里经 ros2 run 拉起。
install(PROGRAMS
  src/gcan_protocol.py
  src/radar_gcan_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# 用 QUIET + 条件判断，不要 REQUIRED：Jetson 上不一定装了
# ros-humble-ament-cmake-pytest，REQUIRED 会让整个包直接构建失败。
# 没有它也不影响单测 —— test/test_protocol.py 是纯 stdlib，可以裸跑：
#     python3 src/driver/radar_gcan/test/test_protocol.py
if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_protocol test/test_protocol.py)
  else()
    message(STATUS "ament_cmake_pytest 未找到，跳过 colcon test 集成"
                   "（仍可 python3 test/test_protocol.py 裸跑）")
  endif()
endif()

ament_package()
