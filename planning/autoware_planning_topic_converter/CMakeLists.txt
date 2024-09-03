cmake_minimum_required(VERSION 3.5)
project(autoware_planning_topic_converter)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(converter_base_parameters
  param/converter_base_parameters.yaml
)

ament_auto_add_library(planning_topic_converter SHARED
  src/path_to_trajectory.cpp
)

target_link_libraries(planning_topic_converter
  converter_base_parameters
)

rclcpp_components_register_node(planning_topic_converter
  PLUGIN "autoware::planning_topic_converter::PathToTrajectory"
  EXECUTABLE path_to_trajectory_converter
)

ament_auto_package()
