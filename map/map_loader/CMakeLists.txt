cmake_minimum_required(VERSION 3.0.2)
project(map_loader)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  lanelet2_extension
  pcl_ros
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    std_msgs
    tf2_geometry_msgs
    visualization_msgs
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_map_loader src/pointcloud_map_loader/main.cpp src/pointcloud_map_loader/pointcloud_map_loader_node.cpp)
target_link_libraries(pointcloud_map_loader ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(pointcloud_map_loader ${catkin_EXPORTED_TARGETS})

add_executable(lanelet2_map_loader src/lanelet2_map_loader/lanelet2_map_loader.cpp)
target_link_libraries(lanelet2_map_loader ${catkin_LIBRARIES})
add_dependencies(lanelet2_map_loader ${catkin_EXPORTED_TARGETS})

add_executable(lanelet2_map_visualization src/lanelet2_map_loader/lanelet2_map_visualization.cpp)
target_link_libraries(lanelet2_map_visualization ${catkin_LIBRARIES})
add_dependencies(lanelet2_map_visualization ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(
  TARGETS
    pointcloud_map_loader
    lanelet2_map_loader
    lanelet2_map_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install files
install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
