cmake_minimum_required(VERSION 3.8)          # ROS 2 采用至少 3.8
project(perception)

#
# ─── 1. 编译选项 ────────────────────────────────────────────────
#
# 不同架构的分支控制逻辑保留不变
message(STATUS "CMAKE_x86 = ${x86}")
message(STATUS "CMAKE_ARM = ${ARM}")

if(x86)
  message(STATUS "x86 start cmake")
endif()
if(ARM)
  message(STATUS "arm start cmake")
  set(CMAKE_CXX_STANDARD 17)                 # ROS 2 默认使用 C++17
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#
# ─── 2. 依赖查找 ────────────────────────────────────────────────
#
# ament_cmake 必须放在第一行
find_package(ament_cmake REQUIRED)

# 系统 / 第三方库
find_package(Eigen3  REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(PCL     REQUIRED COMPONENTS common io)   # 如果只用到 PCL header，也可省略

# ROS 2 包
find_package(rclcpp          REQUIRED)
find_package(std_msgs        REQUIRED)
find_package(sensor_msgs     REQUIRED)
find_package(geometry_msgs   REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs        REQUIRED)
find_package(lidar_msgs      REQUIRED)   # 自定义消息包，确保已经迁移到 ROS 2

#
# ─── 3. 头文件与库路径 ─────────────────────────────────────────
#
set(lib_path ${PROJECT_SOURCE_DIR}/third_libs)

file(GLOB spdlog_static  ${lib_path}/spdlog/*.a)
file(GLOB json_static    ${lib_path}/json/*.a)
file(GLOB json_shared    ${lib_path}/json/*.so*)
set(third_party_libs
  ${spdlog_static}
  ${json_static}
  ${json_shared}
)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # 项目内头文件
  ${PROJECT_SOURCE_DIR}
  common
  architecture/lidar_cells/include
  architecture/lidar_objs/include
  architecture/fusion/include
  communication/include
  ${lib_path}
)

#
# ─── 4. 源码与可执行文件 ────────────────────────────────────────
#
set(SRC_FILES
  architecture/lidar_objs/src/lidar_preprocess.cpp
  architecture/lidar_objs/src/lidar_preprocess2.cpp
  common/tool/read_config.cpp
  common/tool/geotool.cc
  communication/src/rosbridge.cpp          # 如果依赖 rosbridge_suite，需要同时迁移
  communication/src/main.cpp
  communication/src/perception_thread.cpp
  architecture/lidar_cells/src/lidar_cell.cpp
  architecture/lidar_objs/src/grid_cluster.cpp
  architecture/lidar_objs/src/lidar_cluster.cpp
  architecture/lidar_objs/src/dbscan.cpp
  architecture/lidar_objs/src/min_rotate_rect.cpp
)

add_executable(perception ${SRC_FILES})

#
# ─── 5. 目标依赖与链接 ─────────────────────────────────────────
#
ament_target_dependencies(perception
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_msgs
  lidar_msgs
  Eigen3
  OpenCV
)

target_link_libraries(perception
  ${third_party_libs}
  PCL::common
  PCL::io
  proj    # 若依赖 libproj，请确认系统已安装；否则移除或替换
)

#
# ─── 6. 安装目标 ───────────────────────────────────────────────
#
install(TARGETS perception
  RUNTIME DESTINATION lib/${PROJECT_NAME})

#
# ─── 7. 生成并导出 ─────────────────────────────────────────────
#
ament_export_include_directories(include)
ament_package()
