cmake_minimum_required(VERSION 3.5)
project(yolo)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  src/main.py
  src/startYolo.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}) 

ament_package()

