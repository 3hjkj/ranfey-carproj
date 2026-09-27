cmake_minimum_required(VERSION 3.8)
project(rfans_driver LANGUAGES C CXX)

# ① 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ② ROS 2 & 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# RViz 手柄插件不是采集点云所必需，关掉可省去 rviz_common 一大串依赖。
# 默认 ON，保持原有行为；只要点云数据时用 -DBUILD_TELEOP_PLUGIN=OFF。
option(BUILD_TELEOP_PLUGIN "Build the RViz teleop panel plugin" ON)
if(BUILD_TELEOP_PLUGIN)
  find_package(rviz_common REQUIRED)
endif()
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(message_filters REQUIRED)

pkg_check_modules(PCAP REQUIRED libpcap)

# ③ 接口文件生成（msg/srv）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RfansPacket.msg"
  "msg/Packet.msg"
  "msg/RfansScan.msg"
  "msg/Command.msg"
  "msg/ImuData.msg"
  "msg/HeartMsg.msg"
  "srv/RfansCommand.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# 导出编译依赖
ament_export_dependencies(rosidl_default_runtime)

# ④ include 路径
include_directories(
  include
  ${PCAP_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/lib"
  "/opt/ros/humble/include"
)
#气笑了
link_directories(
  "${PROJECT_SOURCE_DIR}/lib"
  ${PCAP_LIBRARY_DIRS}
)

set(LIB_GATHER
  LidarCalc
  LidarCFG
  LidarDecoder
  LidarIO
)

# 厂商那 4 个 .so 把 libstdc++ 静态打包进了自己，并且把 __cxa_throw /
# __cxa_begin_catch / __cxa_guard_acquire 这些 C++ 运行时符号以【强定义】导出
# （每个库 5000+ 个符号）。如果它们排在 libstdc++.so.6 前面进入 DT_NEEDED，
# 动态链接器就会用厂商那份来解析全进程的异常处理，FastRTPS 的异常控制流随即
# 崩掉——表现为没有任何错误输出的 abort()（core dumped），排查起来极其隐蔽。
# 强制 libstdc++ 排到最前面即可让系统那份覆盖掉厂商的副本。
# 用 objdump -p <bin> | grep NEEDED 可以确认 libstdc++.so.6 在厂商库之前。
set(LIB_PRE -Wl,--no-as-needed -lstdc++)
set(LIB_POST -Wl,--as-needed)

# ⑤ 可执行文件定义
add_executable(driver_node
  src/publisher.cpp
  src/ioapi.cpp
  src/rfans_driver.cpp
  src/ssFrameLib.cpp
  src/common.cpp
)
ament_target_dependencies(driver_node rclcpp std_msgs geometry_msgs sensor_msgs message_filters)
target_link_libraries(driver_node ${LIB_PRE} ${LIB_GATHER} ${PCAP_LIBRARIES} Qt5::Widgets ${LIB_POST})
rosidl_target_interfaces(driver_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(heart_node
  src/heart_node.cpp
  src/ioapi.cpp
  src/ssFrameLib.cpp
  src/common.cpp
)
ament_target_dependencies(heart_node rclcpp std_msgs geometry_msgs message_filters)
target_link_libraries(heart_node ${LIB_PRE} ${LIB_GATHER} ${PCAP_LIBRARIES} ${LIB_POST})
rosidl_target_interfaces(heart_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(u_coordinate
  src/u_coordinate.cpp
  src/ioapi.cpp
  src/ssFrameLib.cpp
)
ament_target_dependencies(u_coordinate rclcpp geometry_msgs message_filters)
target_link_libraries(u_coordinate ${LIB_PRE} ${LIB_GATHER} ${PCAP_LIBRARIES} ${LIB_POST})
rosidl_target_interfaces(u_coordinate ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ⑥ RViz 插件
if(BUILD_TELEOP_PLUGIN)
  add_library(rviz_teleop_commander SHARED src/teleop_pad.cpp)
  ament_target_dependencies(rviz_teleop_commander rviz_common geometry_msgs message_filters)
  target_link_libraries(rviz_teleop_commander ${LIB_PRE} ${LIB_GATHER} Qt5::Widgets ${LIB_POST})
  rosidl_target_interfaces(rviz_teleop_commander ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

# ⑦ 安装
set(_rfans_targets driver_node heart_node u_coordinate)
if(BUILD_TELEOP_PLUGIN)
  list(APPEND _rfans_targets rviz_teleop_commander)
endif()

install(TARGETS
  ${_rfans_targets}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# 厂商那 4 个 .so 必须跟着装，否则 install 出来的二进制跑不起来：
#   error while loading shared libraries: libLidarCalc.so
# 链接走的是源码树里的 lib/（见上面的 link_directories），但 install 会剥掉
# 编译期的 RUNPATH。直接跑 build/ 里的可执行文件时 CMake 会自动带上指向源码
# lib/ 的 RUNPATH，所以这个坑只在 `ros2 run` / `ros2 launch` 走 install 那棵树
# 时才暴露——车机上也是同一个坑。
# INSTALL_RPATH 用 $ORIGIN/.. ：可执行文件在 lib/rfans_driver/，库装在 lib/，
# 相对定位，换机器、换绝对路径都不用重编。
set_target_properties(${_rfans_targets} PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
)
install(FILES
  lib/libLidarCalc.so
  lib/libLidarCFG.so
  lib/libLidarDecoder.so
  lib/libLidarIO.so
  DESTINATION lib
)

# revise/ 是各台雷达的标定文件，cfg_path 参数默认就指向它，必须一起安装
install(DIRECTORY launch cfg msg srv revise
  DESTINATION share/${PROJECT_NAME}
)

# 根目录这几个 .rviz 也要装：node_manager.launch.py 启动 rviz2 时传的
# -d 就是 share/rfans_driver/ 下的这些文件。原来一条都没装，车机上跑
# launch 会拿到一个不存在的配置路径。
#
# 其中 single_Rviz_cfg.rviz 等是 RViz1 格式、话题写死 /points_raw，
# 在 Humble 的 rviz2 里显示不出点云。Rviz2_live_cfg.rviz 是 RViz2 格式，
# 话题 /rfans_points、Fixed Frame rflink，与驱动实际发布的一致。
install(FILES
  single_Rviz_cfg.rviz
  StarROS_Rviz_cfg.rviz
  multi_Rviz_cfg.rviz
  apollo_rviz_cfg.rviz
  Rviz2_live_cfg.rviz
  DESTINATION share/${PROJECT_NAME}
)

# 宿主侧中继：把容器 UDP 转发出来的点云重新发布成 ROS2 话题，供 rviz2 订阅。
# 只有 PC + qemu 这条路上用得到（见 scripts/rfans_relay.py 的说明）；
# 车机上不需要，但装着无害，不影响直接 publish 的那条路。
install(PROGRAMS scripts/rfans_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
