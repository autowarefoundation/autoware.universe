cmake_minimum_required(VERSION 3.14)
project(autoware_path_generator)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(path_generator_parameters
  param/path_generator_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  path_generator_parameters
)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::path_generator::PathGenerator"
  EXECUTABLE path_generator_node
)

ament_auto_package()
