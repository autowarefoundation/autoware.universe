cmake_minimum_required(VERSION 3.14)
project(localization_util)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(util_func SHARED
  src/util_func.cpp
)

ament_auto_add_library(pose_array_interpolator SHARED
  src/pose_array_interpolator.cpp
)

ament_auto_add_library(tf2_listener_module SHARED
  src/tf2_listener_module.cpp
)

ament_auto_add_library(tree_structured_parzen_estimator SHARED
  src/tree_structured_parzen_estimator.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tpe
    test/test_tpe.cpp
    src/tree_structured_parzen_estimator.cpp
  )
  target_include_directories(test_tpe PRIVATE include)
  target_link_libraries(test_tpe)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
