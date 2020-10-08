cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_to_laserscan)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_to_laserscan
  CATKIN_DEPENDS roscpp message_filters nodelet sensor_msgs tf2 tf2_ros visualization_msgs pcl_ros pcl_conversions

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pointcloud_to_laserscan src/pointcloud_to_laserscan_nodelet.cpp)
target_link_libraries(pointcloud_to_laserscan ${catkin_LIBRARIES})

add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
target_link_libraries(pointcloud_to_laserscan_node pointcloud_to_laserscan ${catkin_LIBRARIES})

install(TARGETS pointcloud_to_laserscan pointcloud_to_laserscan_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
