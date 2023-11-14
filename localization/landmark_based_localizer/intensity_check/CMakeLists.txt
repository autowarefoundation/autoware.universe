cmake_minimum_required(VERSION 3.14)
project(intensity_check)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

ament_auto_add_executable(intensity_check
  src/subscriber_member_function.cpp 
  )
ament_auto_add_executable(lidar_marker_detector
  src/lidar_marker_detector_node.cpp 
  src/lidar_marker_detector_core.cpp 
  src/pose_array_interpolator.cpp 
  src/util_func.cpp 
)

target_link_libraries(lidar_marker_detector
  ${PCL_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
