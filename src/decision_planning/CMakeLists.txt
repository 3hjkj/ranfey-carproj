cmake_minimum_required(VERSION 3.8)
project(decision_planning)

# ──────────────── 编译选项 ────────────────
set(CMAKE_CXX_STANDARD 17)          # C++17 已是 ROS 2 推荐
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)            # 与旧工程保持一致
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)       # colcon 可用 --cmake-args -DCMAKE_BUILD_TYPE=Release 覆盖
endif()

# ──────────────── 依赖 ────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(decision_planning_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(can_control_msgs  REQUIRED)
find_package(lidar_msgs        REQUIRED)
find_package(tf2 REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  src/local_path_planning       # 若只是头文件路径，保留即可
)

# ──────────────── 可执行程序 ────────────────
add_executable(decision_planning
  src/local_path_planning/route.cpp
  src/decision_planning.cpp
)
ament_target_dependencies(decision_planning
  rclcpp geometry_msgs decision_planning_msgs
  localization_msgs can_control_msgs lidar_msgs
  tf2 tf2_geometry_msgs visualization_msgs radar_msgs
)

add_executable(lidar_process
  src/lidar_process.cpp         # 请确保文件后缀正确
)
ament_target_dependencies(lidar_process
  rclcpp lidar_msgs geometry_msgs
)

add_executable(la
  src/jw.cpp
)
ament_target_dependencies(la
  rclcpp geometry_msgs decision_planning_msgs
)

# ──────────────── 安装 ────────────────
install(
  TARGETS decision_planning lidar_process la
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
