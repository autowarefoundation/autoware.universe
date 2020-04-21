cmake_minimum_required(VERSION 2.8.3)
project(roi_cluster_fusion)

add_compile_options(-std=c++11 -O3)

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

find_package(PCL REQUIRED)
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
  ${PCL_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(roi_cluster_fusion src/roi_cluster_fusion_nodelet.cpp)
target_link_libraries(roi_cluster_fusion
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(roi_cluster_fusion ${catkin_EXPORTED_TARGETS})

add_executable(roi_cluster_fusion_node src/roi_cluster_fusion_node.cpp)
target_link_libraries(roi_cluster_fusion_node
  roi_cluster_fusion
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(roi_cluster_fusion_node ${catkin_EXPORTED_TARGETS})

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
