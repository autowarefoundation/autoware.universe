cmake_minimum_required(VERSION 3.5)
project(bezier_sampler)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(bezier_sampler SHARED
  src/bezier.cpp
  src/bezier_sampling.cpp
  src/path_splitting.cpp
  src/constraint_checker.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_bezier_sampler
    test/test_constraint_checker.cpp
  )
  target_link_libraries(test_bezier_sampler
    bezier_sampler
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
