cmake_minimum_required(VERSION 2.8.3)
project(traffic_light_map_based_detector)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_planning_msgs
  geometry_msgs
  lanelet2_extension
  roscpp
  sensor_msgs
  tf2
  tf2_ros
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    autoware_perception_msgs
    autoware_planning_msgs
    geometry_msgs
    lanelet2_extension
    roscpp
    sensor_msgs
    tf2
    tf2_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(traffic_light_map_based_detector_node
  src/node.cpp
  src/main.cpp
)

add_dependencies(traffic_light_map_based_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(traffic_light_map_based_detector_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    traffic_light_map_based_detector_node
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
