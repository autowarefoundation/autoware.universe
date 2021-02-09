cmake_minimum_required(VERSION 3.5)
project(frenet_planner_node)

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
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(OpenCV REQUIRED)  # used for drivable area


ament_auto_add_library(frenet_planner_node SHARED
  src/frenet_planner_node.cpp
  src/prepare_inputs.cpp
  src/trajectory_generation.cpp
)

target_link_libraries(frenet_planner_node
  Qt5::Widgets
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(frenet_planner_node
  PLUGIN "frenet_planner_node::FrenetPlannerNode"
  EXECUTABLE frenet_planner_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_frenet_planner_node
    test/test_prepare_inputs.cpp
  )
  target_link_libraries(test_frenet_planner_node
    frenet_planner_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
