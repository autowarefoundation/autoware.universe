cmake_minimum_required(VERSION 3.14)
project(recordreplay_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Build library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/recordreplay_planner/recordreplay_planner.cpp
)

### Test
if(BUILD_TESTING)
  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ## Unit test
  ament_add_gtest(recordreplay_planner_unit_tests
    test/gtest_main.cpp
    test/sanity_checks.cpp)
  target_compile_options(recordreplay_planner_unit_tests PRIVATE -Wno-conversion -Wno-double-promotion)
  target_link_libraries(recordreplay_planner_unit_tests ${PROJECT_NAME})
  target_include_directories(recordreplay_planner_unit_tests PRIVATE "include")
endif()

ament_auto_package()
