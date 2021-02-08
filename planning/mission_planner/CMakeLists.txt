cmake_minimum_required(VERSION 3.5)
project(mission_planner)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(mission_planner
  lib/mission_planner_base.cpp
  src/mission_planner_lanelet2/mission_planner_lanelet2.cpp
  src/mission_planner_lanelet2/route_handler.cpp
  src/mission_planner_lanelet2/mission_planner_main.cpp
  src/mission_planner_lanelet2/utility_functions.cpp
)

ament_auto_add_executable(goal_pose_visualizer
  src/goal_pose_visualizer/main.cpp
  src/goal_pose_visualizer/goal_pose_visualizer.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
