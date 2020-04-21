cmake_minimum_required(VERSION 2.8.3)
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

find_package(PCL REQUIRED COMPONENTS io)

# See: https://github.com/ros-perception/perception_pcl/blob/lunar-devel/pcl_ros/CMakeLists.txt#L10-L22
if (NOT "${PCL_LIBRARIES}" STREQUAL "")
    # For debian: https://github.com/ros-perception/perception_pcl/issues/139
    list(REMOVE_ITEM PCL_IO_LIBRARIES
      "vtkGUISupportQt"
      "vtkGUISupportQtOpenGL"
      "vtkGUISupportQtSQL"
      "vtkGUISupportQtWebkit"
      "vtkViewsQt"
      "vtkRenderingQt")
endif ()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    std_msgs
    tf2_geometry_msgs
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
)

add_executable(pointcloud_map_loader src/pointcloud_map_loader/main.cpp src/pointcloud_map_loader/pointcloud_map_loader_node.cpp)
target_link_libraries(pointcloud_map_loader ${catkin_LIBRARIES} ${PCL_IO_LIBRARIES})
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
