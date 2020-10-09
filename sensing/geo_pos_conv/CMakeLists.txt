cmake_minimum_required(VERSION 3.5)
project(geo_pos_conv)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Add dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


### Create library
add_library(geo_pos_conv
  src/geo_pos_conv.cpp
)

target_include_directories(geo_pos_conv
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(geo_pos_conv rclcpp)

### Install library
ament_export_targets(geo_pos_conv HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS geo_pos_conv
  EXPORT geo_pos_conv
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
