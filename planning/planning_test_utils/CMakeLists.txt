cmake_minimum_required(VERSION 3.14)
project(planning_test_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(route_parser SHARED
  src/route_parser.cpp)

target_link_libraries(route_parser
  yaml-cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_route_parser
    test/test_route_parser.cpp)

  target_link_libraries(test_route_parser
    route_parser)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  test_map
  test_data
)
