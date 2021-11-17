cmake_minimum_required(VERSION 3.5)
project(signal_filters)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/signal_filter.cpp)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(signal_filter_test test/sanity_check.cpp test/gtest_main.cpp)
  autoware_set_compile_options(signal_filter_test)
  target_compile_options(signal_filter_test PRIVATE -Wno-sign-conversion)
  target_include_directories(signal_filter_test PRIVATE include)
  ament_target_dependencies(signal_filter_test "autoware_auto_common")
endif()

ament_auto_package()
