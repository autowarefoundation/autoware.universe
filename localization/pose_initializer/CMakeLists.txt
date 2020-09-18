cmake_minimum_required(VERSION 3.0.2)
project(pose_initializer)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
  autoware_localization_srvs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pose_initializer
  src/pose_initializer_node.cpp
  src/pose_initializer_core.cpp
)

add_dependencies(pose_initializer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pose_initializer ${catkin_LIBRARIES})

install(
  TARGETS
    pose_initializer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
