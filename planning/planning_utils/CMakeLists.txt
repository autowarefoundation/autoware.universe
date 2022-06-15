cmake_minimum_required(VERSION 3.14)
project(planning_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_add_library(planning_utils SHARED
  src/planning_utils.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_planning_utils ${test_files})

  target_link_libraries(test_planning_utils
    planning_utils
  )
endif()

ament_auto_package()
