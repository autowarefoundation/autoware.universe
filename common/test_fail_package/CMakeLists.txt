cmake_minimum_required(VERSION 3.8)
project(test_fail_package)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/*.cpp)

  ament_add_ros_isolated_gtest(test_always_fail ${test_files})
endif()

ament_auto_package()
