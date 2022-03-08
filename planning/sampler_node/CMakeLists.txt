cmake_minimum_required(VERSION 3.5)
project(sampler_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

### Dependency for debug visualization executable
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(QCustomPlot REQUIRED)


ament_auto_add_library(sampler_node SHARED
  DIRECTORY src
)

target_link_libraries(sampler_node
  Qt5::Widgets
  ${QCustomPlot_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(sampler_node
  PLUGIN "sampler_node::TrajectorySamplerNode"
  EXECUTABLE trajectory_sampler_node_exe
)

rclcpp_components_register_node(sampler_node
  PLUGIN "sampler_node::PathSamplerNode"
  EXECUTABLE path_sampler_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_sampler_node
    test/test_prepare_inputs.cpp
  )
  target_link_libraries(test_sampler_node
    sampler_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
