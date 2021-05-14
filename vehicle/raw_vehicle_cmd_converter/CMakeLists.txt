cmake_minimum_required(VERSION 3.5)

project(raw_vehicle_cmd_converter)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(accel_map_converter SHARED
  src/accel_map.cpp
  src/brake_map.cpp
  src/csv_loader.cpp
  src/interpolate.cpp
)

ament_auto_add_executable(raw_vehicle_cmd_converter_node
  src/node.cpp
  src/main.cpp
)

add_dependencies(raw_vehicle_cmd_converter_node accel_map_converter)
target_link_libraries(raw_vehicle_cmd_converter_node accel_map_converter)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  data
)

install(PROGRAMS scripts/plot_accel_brake_map.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES DESTINATION share/${PROJECT_NAME})
