cmake_minimum_required(VERSION 3.14)
project(autoware_lidar_marker_localizer)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

ament_auto_add_executable(lidar_marker_localizer
  src/lidar_marker_localizer.cpp
  src/diagnostics/diagnostics_module.cpp
  src/main.cpp
)

target_link_libraries(lidar_marker_localizer
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
