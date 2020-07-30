cmake_minimum_required(VERSION 3.0.2)
project(image_transport_decompressor)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  cv_bridge
  pluginlib
  rosconsole
  roscpp
  roslib
  sensor_msgs
  nodelet
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_transport_decompressor_nodelet
  CATKIN_DEPENDS image_transport cv_bridge pluginlib rosconsole roscpp roslib sensor_msgs nodelet
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(image_transport_decompressor_nodelet SHARED
  src/nodelet.cpp
)

add_dependencies(image_transport_decompressor_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_transport_decompressor_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_transport_decompressor_node
src/node.cpp
)

add_dependencies(image_transport_decompressor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_transport_decompressor_node
  image_transport_decompressor_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS
  image_transport_decompressor_node
  image_transport_decompressor_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
  image_transport_decompressor.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
