cmake_minimum_required(VERSION 3.0.2)
project(roi_cluster_fusion)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  tf2_sensor_msgs
  tf2_ros
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    roi_cluster_fusion
  CATKIN_DEPENDS
    autoware_perception_msgs
    nodelet
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf2
    tf2_sensor_msgs
    tf2_ros
    message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(roi_cluster_fusion src/roi_cluster_fusion_nodelet.cpp)
target_link_libraries(roi_cluster_fusion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(roi_cluster_fusion ${catkin_EXPORTED_TARGETS})

add_executable(roi_cluster_fusion_node src/roi_cluster_fusion_node.cpp)
target_link_libraries(roi_cluster_fusion_node
  roi_cluster_fusion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(roi_cluster_fusion_node ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    roi_cluster_fusion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    roi_cluster_fusion
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
