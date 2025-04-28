cmake_minimum_required(VERSION 2.8.3)
project(decision_planning)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  rospy
  geometry_msgs
  decision_planning_msgs
  localization_msgs
  can_control_msgs
  lidar_msgs
)

ament_target_dependencies(
  CATKIN_DEPENDS
  message_runtime
  INCLUDE_DIRS include
#  LIBRARIES can_com
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  ./src/local_path_planning
  ./include
# include
# ${catkin_INCLUDE_DIRS}
)



add_executable(decision_planning
  src/local_path_planning/route.cpp
  src/decision_planning.cpp
)
add_executable(lidar_process src/lidar_process
)
add_executable(la src/jw.cpp
)
target_link_libraries(la

  ${catkin_LIBRARIES}
)
add_dependencies(decision_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(decision_planning
  ${catkin_LIBRARIES}
)
target_link_libraries(lidar_process
  ${catkin_LIBRARIES}
)
