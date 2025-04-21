cmake_minimum_required(VERSION 3.8)
project(gps_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)


include_directories(
  include
  src
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
    ${PROJ_INCLUDE_DIRS}        # ★ 新增

)

# 添加可执行文件
file(GLOB SRCS0 "src/*.cpp")
file(GLOB SRCS1 "src/*.cc")

add_executable(gps_localization_exec
  ${SRCS0}
  ${SRCS1}
)

# 链接依赖
ament_target_dependencies(gps_localization_exec
  rclcpp
  std_msgs
  geometry_msgs
  localization_msgs
)

target_link_libraries(gps_localization_exec
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJ_LIBRARIES}
)

# 安装可执行文件
install(TARGETS gps_localization_exec
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（如有）
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}) 

ament_package()
