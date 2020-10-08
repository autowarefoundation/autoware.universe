cmake_minimum_required(VERSION 3.0.2)
project(euclidean_cluster)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  nodelet
  pcl_conversions
  autoware_perception_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    euclidean_cluster
  CATKIN_DEPENDS
    pcl_ros
    roscpp
    sensor_msgs
    nodelet
    pcl_conversions
    autoware_perception_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(euclidean_cluster src/euclidean_cluster_nodelet.cpp
  src/voxel_grid_based_euclidean_cluster_nodelet.cpp)
target_link_libraries(euclidean_cluster ${catkin_LIBRARIES})
add_dependencies(euclidean_cluster ${catkin_EXPORTED_TARGETS})

add_executable(euclidean_cluster_node src/euclidean_cluster_node.cpp)
target_link_libraries(euclidean_cluster_node
  euclidean_cluster
  ${catkin_LIBRARIES}
)
add_dependencies(euclidean_cluster_node ${catkin_EXPORTED_TARGETS})
add_executable(voxel_grid_based_euclidean_cluster_node
  src/voxel_grid_based_euclidean_cluster_node.cpp)
target_link_libraries(voxel_grid_based_euclidean_cluster_node
  euclidean_cluster
  ${catkin_LIBRARIES}
)
add_dependencies(voxel_grid_based_euclidean_cluster_node
  ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    euclidean_cluster_node
    voxel_grid_based_euclidean_cluster_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    euclidean_cluster
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)    

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
