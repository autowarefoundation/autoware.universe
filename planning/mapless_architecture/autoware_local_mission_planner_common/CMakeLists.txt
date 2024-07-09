cmake_minimum_required(VERSION 3.8)
project(autoware_local_mission_planner_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(autoware_cmake REQUIRED)
ament_auto_find_build_dependencies()
autoware_package()

# Add library to be exported
add_library(${PROJECT_NAME} SHARED
  src/helper_functions.cpp)

include_directories(include)

# Add dependent libraries
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  lanelet2_core
  autoware_planning_msgs
  visualization_msgs)

# Include public headers
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Export library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependent libraries
ament_export_dependencies(
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  lanelet2_core
  autoware_planning_msgs
  visualization_msgs)

# Install library
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_auto_add_gtest(${PROJECT_NAME}_tests
    test/gtest_main.cpp
    src/helper_functions.cpp)

  ament_lint_auto_find_test_dependencies()
  target_include_directories(${PROJECT_NAME}_tests PRIVATE test/include)
endif()

# Ensure all packages are correctly installed
ament_package()
