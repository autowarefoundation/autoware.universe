cmake_minimum_required(VERSION 2.8.3)
project(turn_signal_decider)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  autoware_vehicle_msgs
  geometry_msgs
  lanelet2_extension
  roscpp
  std_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(turn_signal_decider src/turn_signal_decider_node.cpp src/turn_signal_decider_core.cpp src/data_manager.cpp src/frenet_coordinate.cpp)
add_dependencies(turn_signal_decider ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turn_signal_decider
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    turn_signal_decider
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
