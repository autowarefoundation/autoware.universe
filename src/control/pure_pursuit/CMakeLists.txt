cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_control_msgs
  autoware_planning_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_control_msgs
    autoware_planning_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_eigen
    tf2_ros
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# pure_pursuit_core
add_library(pure_pursuit_core
  src/pure_pursuit_core/planning_utils.cpp
  src/pure_pursuit_core/pure_pursuit.cpp
  src/pure_pursuit_core/interpolate.cpp
)

target_link_libraries(pure_pursuit_core
  ${catkin_LIBRARIES}
)

add_dependencies(pure_pursuit_core
  ${catkin_EXPORTED_TARGETS}
)

# pure_pursuit
add_executable(pure_pursuit
  src/pure_pursuit/main.cpp
  src/pure_pursuit/pure_pursuit_node.cpp
  src/pure_pursuit/pure_pursuit_viz.cpp
)

target_link_libraries(pure_pursuit
  pure_pursuit_core
  ${catkin_LIBRARIES}
)

add_dependencies(pure_pursuit
  ${catkin_EXPORTED_TARGETS}
)

# Install executables and/or libraries
install(
  TARGETS
    pure_pursuit_core
    pure_pursuit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install files
install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
