cmake_minimum_required(VERSION 2.8.3)
project(ndt_scan_matcher)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  diagnostic_msgs
  pcl_conversions
  pcl_ros
  dynamic_reconfigure
  ndt_pcl_modified
  ndt_omp
  ndt
  autoware_localization_srvs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ndt_scan_matcher
  src/ndt_scan_matcher_node.cpp
  src/ndt_scan_matcher_core.cpp
)

add_dependencies(ndt_scan_matcher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ndt_scan_matcher ${catkin_LIBRARIES})

install(
  TARGETS
    ndt_scan_matcher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
