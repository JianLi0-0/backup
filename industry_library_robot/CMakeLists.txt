cmake_minimum_required(VERSION 3.0.2)
project(industry_library_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  trajectory_msgs
  trac_ik_lib
  tf
)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  pick_and_place.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(picking
  src/picking.cpp
  src/my_robot.cpp
  include/my_robot.h
)
add_dependencies(picking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(picking
  ${catkin_LIBRARIES}
  jsoncpp
)