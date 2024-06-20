cmake_minimum_required(VERSION 3.8)
project(autoware_local_mission_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- FIND DEPENDENCIES ---
find_package(autoware_cmake REQUIRED)  # automatically find dependencies
ament_auto_find_build_dependencies()
autoware_package()

# build executables
ament_auto_add_executable(${PROJECT_NAME}
  src/mission_planner_main.cpp
  src/mission_planner_node.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install executable(s)
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# install launchfile(s)/param file(s)
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME})

# --- SPECIFY TESTS ---
# configure clang format
set(ament_cmake_clang_format_CONFIG_FILE ../../.clang-format)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_auto_add_gtest(${PROJECT_NAME}_tests test/gtest_main.cpp
    test/src/test_mission_planner_core.cpp
    src/mission_planner_node.cpp)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
