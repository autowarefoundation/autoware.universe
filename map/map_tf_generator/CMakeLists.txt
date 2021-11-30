cmake_minimum_required(VERSION 3.5)
project(map_tf_generator)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### ROS Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### non-ROS Dependencies
find_package(PCL REQUIRED)

# Generate exe file
ament_auto_add_executable(map_tf_generator nodes/map_tf_generator.cpp)
target_link_libraries(map_tf_generator ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Install
ament_auto_package(INSTALL_TO_SHARE
  launch
)
