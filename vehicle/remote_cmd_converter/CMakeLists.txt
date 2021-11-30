cmake_minimum_required(VERSION 2.8.3)
project(remote_cmd_converter)
add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_vehicle_msgs
  autoware_control_msgs
  geometry_msgs
  raw_vehicle_cmd_converter
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs autoware_vehicle_msgs autoware_control_msgs geometry_msgs raw_vehicle_cmd_converter
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(remote_cmd_converter_node
  src/node.cpp
  src/main.cpp
)
add_dependencies(remote_cmd_converter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(remote_cmd_converter_node ${catkin_LIBRARIES})

install(TARGETS
    remote_cmd_converter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
