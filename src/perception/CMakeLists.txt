###############################################################################
# 1. 最低版本 & 项目配置
###############################################################################
cmake_minimum_required(VERSION 3.8)      # ROS 2 基本要求 ≥3.8
project(perception)

# --- 编译选项 ---------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS  OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# 若仍需区分 ARM / x86，可以保留你原来的变量判断
message(STATUS "CMAKE_x86 = ${x86}")
message(STATUS "CMAKE_ARM = ${ARM}")

###############################################################################
# 2. 依赖查找  (先 ament_cmake，再其他)
###############################################################################
find_package(ament_cmake REQUIRED)

# 第三方 / 系统库
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Qhull REQUIRED COMPONENTS qhull_r)

# ROS 2 接口库
find_package(rclcpp          REQUIRED)
find_package(sensor_msgs     REQUIRED)
find_package(geometry_msgs   REQUIRED)
find_package(pcl_ros         REQUIRED)
find_package(std_msgs        REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(Qhull      REQUIRED COMPONENTS qhull_r)
find_package(can_control_msgs REQUIRED)
find_package(pcl_msgs        REQUIRED)
find_package(lidar_msgs      REQUIRED)
find_package(spdlog REQUIRED)


###############################################################################
# 3. 第三方静态 / 共享库集合
###############################################################################
set(lib_path ${PROJECT_SOURCE_DIR}/third_libs)

file(GLOB json_static   ${lib_path}/json/*.a)
file(GLOB json_shared   ${lib_path}/json/*.so*)

set(third_party_libs

  ${json_static}
  ${json_shared}
)

###############################################################################
# 4. 头文件搜索路径
###############################################################################
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}               # 工程根
  common
  architecture/lidar_cells/include
  architecture/lidar_objs/include
  architecture/fusion/include
  communication/include
  ${lib_path}                          # 本地第三方
)

###############################################################################
# 5. 源文件列表
###############################################################################
set(SRC_FILES
  architecture/lidar_objs/src/lidar_preprocess.cpp
  architecture/lidar_objs/src/lidar_preprocess2.cpp
  common/tool/read_config.cpp
  common/tool/geotool.cc
  communication/src/rosbridge.cpp
  communication/src/main.cpp
  communication/src/perception_thread.cpp
  architecture/lidar_cells/src/lidar_cell.cpp
  architecture/lidar_objs/src/grid_cluster.cpp
  architecture/lidar_objs/src/lidar_cluster.cpp
  architecture/lidar_objs/src/dbscan.cpp
  architecture/lidar_objs/src/min_rotate_rect.cpp
)

###############################################################################
# 6. 目标 & 依赖
###############################################################################
add_executable(perception ${SRC_FILES})

# 如果高版本 PCL 导出了 PCL::common 等目标就用它们；否则 fall-back
if(TARGET PCL::common)
  target_link_libraries(perception
    PCL::common
    PCL::io
    PCL::filters
    ${third_party_libs}
    ${OpenCV_LIBRARIES}
    proj     
    Qhull::qhull_r   
    spdlog::spdlog                      
  )
else()
  target_link_libraries(perception
    ${PCL_LIBRARIES}                 # 旧式聚合变量
    ${third_party_libs}
    ${OpenCV_LIBRARIES}
    proj
    Qhull::qhull_r
    spdlog::spdlog 
  )
endif()

ament_target_dependencies(perception
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  pcl_msgs
  lidar_msgs
  OpenCV
  localization_msgs
  pcl_ros
  can_control_msgs
)

###############################################################################
# 7. 安装与导出
###############################################################################
install(TARGETS perception
  RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY communication/include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}) 

# 如果还想装头文件可按需开启
# install(DIRECTORY common/include/ DESTINATION include)

ament_export_include_directories(
  common
  architecture/lidar_cells/include
  architecture/lidar_objs/include
  architecture/fusion/include
  communication/include
)

ament_package()
