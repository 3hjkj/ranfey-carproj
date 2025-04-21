cmake_minimum_required(VERSION 3.8)
project(ck_camera_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_SKIP_RPATH TRUE)

# ─── 依赖包 ───────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(TIFF REQUIRED)

# ─── SDK 头文件 & 库路径 ─────────────────────────────
set(CAMERA_SDK_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(CAMERA_SDK_LIBDIR  ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(
  ${CAMERA_SDK_INCLUDE}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${CAMERA_SDK_LIBDIR})

# ─── 可执行节点 ──────────────────────────────────────
add_executable(camera_node src/main.cpp)

ament_target_dependencies(camera_node
  rclcpp sensor_msgs image_transport cv_bridge)

target_link_libraries(camera_node
  CKCameraSDK           # SDK 动态库
  usb pthread
  ${OpenCV_LIBS}
  CURL::libcurl
  TIFF::TIFF)

# 让可执行程序运行时能找到 SDK so
target_link_options(camera_node PUBLIC
  -Wl,-rpath,${CAMERA_SDK_LIBDIR})

# ─── 安装 ────────────────────────────────────────────
install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/  DESTINATION include)
install(DIRECTORY lib/      DESTINATION lib)

ament_package()
