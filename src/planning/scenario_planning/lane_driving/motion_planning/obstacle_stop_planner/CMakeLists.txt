cmake_minimum_required(VERSION 2.8.3)
project(obstacle_stop_planner)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
  pcl_ros
)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    autoware_planning_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_eigen
    tf2_ros
    visualization_msgs
    pcl_ros
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_stop_planner_node
  src/debug_marker.cpp
  src/node.cpp
  src/main.cpp
)
add_dependencies(obstacle_stop_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(obstacle_stop_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(
  TARGETS
    obstacle_stop_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
