cmake_minimum_required(VERSION 3.8)
project(autoware_local_mission_planner)

# Check for compiler
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- FIND DEPENDENCIES ---
find_package(autoware_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mission_planner_node.cpp
)

# Register node
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::mapless_architecture::MissionPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

# Specify include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Specify required C and C++ standards
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

# Install the target library
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install the launch and param directories
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME})

# --- SPECIFY TESTS ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_auto_add_gtest(${PROJECT_NAME}_tests test/gtest_main.cpp
    test/src/test_mission_planner_core.cpp
    src/mission_planner_node.cpp)

  ament_lint_auto_find_test_dependencies()
endif()

# Ensure all packages are correctly installed
ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
