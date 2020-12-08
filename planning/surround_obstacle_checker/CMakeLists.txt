cmake_minimum_required(VERSION 3.5)
project(surround_obstacle_checker)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_executable(${PROJECT_NAME}_node
  src/debug_marker.cpp
  src/node.cpp
  src/main.cpp
)

set(${PROJECT_NAME}_DEPENDENCIES
  eigen3_cmake_module
  Eigen3
  PCL
  autoware_perception_msgs
  autoware_planning_msgs
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
  pcl_conversions
  vehicle_info_util
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC ${${PROJECT_NAME}_DEPENDENCIES})

target_compile_definitions(${PROJECT_NAME}_node PUBLIC ${PCL_DEFINITIONS})
target_include_directories(${PROJECT_NAME}_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PCL_LIBRARIES})
target_link_directories(${PROJECT_NAME}_node PUBLIC ${PCL_LIBRARY_DIRS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
