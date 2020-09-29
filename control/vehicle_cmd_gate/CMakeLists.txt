cmake_minimum_required(VERSION 3.0.2)
project(vehicle_cmd_gate)

add_compile_options(-std=c++14)

find_package(
  catkin REQUIRED COMPONENTS
    autoware_control_msgs
    autoware_vehicle_msgs
    geometry_msgs
    roscpp
    rostest
    rosunit
    roslint
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    autoware_control_msgs
    autoware_vehicle_msgs
    geometry_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_cmd_gate src/vehicle_cmd_gate_node.cpp src/vehicle_cmd_gate.cpp src/vehicle_cmd_filter.cpp)
target_link_libraries(vehicle_cmd_gate ${catkin_LIBRARIES})
add_dependencies(vehicle_cmd_gate ${catkin_EXPORTED_TARGETS})

install(
  TARGETS vehicle_cmd_gate
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
