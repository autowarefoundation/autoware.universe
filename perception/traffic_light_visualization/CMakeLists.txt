cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_visualization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  cv_bridge
  image_transport
  message_filters
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS autoware_perception_msgs cv_bridge image_transport message_filters roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_executable(traffic_light_roi_visualizer_node
  src/node.cpp
  src/main.cpp
)

add_dependencies(traffic_light_roi_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(traffic_light_roi_visualizer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    traffic_light_roi_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
