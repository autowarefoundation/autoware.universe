cmake_minimum_required(VERSION 3.8)
project(autoware_local_mission_planner_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(autoware_cmake REQUIRED)  # automatically find dependencies
ament_auto_find_build_dependencies()
autoware_package()

# add library to be exported
add_library(${PROJECT_NAME} SHARED
  src/helper_functions.cpp)

include_directories(include)

# add dependent libraries
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  lanelet2_core
  autoware_planning_msgs
  visualization_msgs)

# include public headers
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# export library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# export dependent libraries
ament_export_dependencies(
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  lanelet2_core
  autoware_planning_msgs
  visualization_msgs)

# install library
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

# configure clang format -----------------------------------------------------------------------------------------------
set(ament_cmake_clang_format_CONFIG_FILE ../../.clang-format)

# testing --------------------------------------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_auto_add_gtest(${PROJECT_NAME}_tests
    # MAIN
    test/gtest_main.cpp
    #
    src/helper_functions.cpp)

  ament_lint_auto_find_test_dependencies()
  target_include_directories(${PROJECT_NAME}_tests PRIVATE test/include)
endif()

ament_package()
