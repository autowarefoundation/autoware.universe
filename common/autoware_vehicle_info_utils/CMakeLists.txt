cmake_minimum_required(VERSION 3.14)
project(autoware_vehicle_info_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(vehicle_info_utils SHARED
  src/vehicle_info.cpp
  src/vehicle_info_utils.cpp
)

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} test/test_vehicle_info_utils.cpp)

  target_link_libraries(test_${PROJECT_NAME}
    vehicle_info_utils
  )
endif()

install(PROGRAMS
  scripts/min_turning_radius_calculator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
