cmake_minimum_required(VERSION 2.8.3)
project(cluster_data_association)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster_data_association_node
  src/utils/utils.cpp
  src/data_association/data_association.cpp
  src/data_association/successive_shortest_path/successive_shortest_path.cpp
  src/node.cpp
  src/main.cpp
)

add_dependencies(cluster_data_association_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cluster_data_association_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
