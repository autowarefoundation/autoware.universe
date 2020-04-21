cmake_minimum_required(VERSION 2.8.3)
project(velocity_controller)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  geometry_msgs
  autoware_planning_msgs
  autoware_control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    geometry_msgs
    autoware_planning_msgs
    autoware_control_msgs
)

###########
## Build ##
###########

include_directories(
  include/velocity_controller
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  velocity_controller
  src/velocity_controller_core.cpp
  src/velocity_controller_node.cpp
  src/pid.cpp
  src/velocity_controller_mathutils.cpp
)

add_dependencies(
  velocity_controller
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  velocity_controller
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS velocity_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
