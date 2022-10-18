cmake_minimum_required(VERSION 3.5)
project(signal_processing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(lowpass_filters SHARED
  src/lowpass_filter_1d.cpp
  src/butterworth.cpp)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_signal_processing
    test/src/lowpass_filter_1d_test.cpp)

  target_link_libraries(test_signal_processing
    lowpass_filters)
endif()

ament_auto_package()

add_executable(main_butterworth usage_examples/main_butterworth.cpp)
target_link_libraries(main_butterworth lowpass_filters)