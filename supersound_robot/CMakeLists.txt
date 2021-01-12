cmake_minimum_required(VERSION 3.0.2)
project(supersound_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  tf
  dynamic_reconfigure
)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/force_controller.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES supersound_robot
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(admittance_controller
  src/admittance_controller.cpp
  src/StatesHub.cpp
  src/ForceTorqueController.cpp
  include/StatesHub.h
  include/ForceTorqueController.h
  include/utils/utils.hpp
)
add_dependencies(admittance_controller 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(admittance_controller
  ${catkin_LIBRARIES}
)


