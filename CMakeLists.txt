cmake_minimum_required(VERSION 2.8.3)
project(zero_gravity_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED dynamic_reconfigure COMPONENTS
  roscpp
  std_msgs
  tf
  moveit_ros_planning
  moveit_core
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/zero_gravity_control.cfg
  #...
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES effort_control
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/zero_gravity_control_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

