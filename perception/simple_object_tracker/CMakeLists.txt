cmake_minimum_required(VERSION 3.8)
project(simple_object_tracker)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

### Find Packages
find_package(ament_cmake_auto REQUIRED)

### Find dependencies listed in the package.xml
ament_auto_find_build_dependencies()

ament_auto_add_library(simple_object_tracker_node SHARED
  src/simple_object_tracker_core.cpp
)

rclcpp_components_register_node(simple_object_tracker_node
  PLUGIN "SimpleObjectTracker"
  EXECUTABLE simple_object_tracker
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
