cmake_minimum_required(VERSION 3.5)
project(multi_object_tracker)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Find Packages
find_package(ament_cmake_auto REQUIRED)

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

### Find dependencies listed in the package.xml
ament_auto_find_build_dependencies()

# Generate exe file
set(MULTI_OBJECT_TRACKER_SRC
  src/multi_object_tracker_core.cpp
  src/utils/utils.cpp
  src/tracker/model/tracker_base.cpp
  src/tracker/model/vehicle_tracker.cpp
  src/tracker/model/bicycle_tracker.cpp
  src/tracker/model/pedestrian_tracker.cpp
  src/data_association/data_association.cpp
  src/data_association/successive_shortest_path/successive_shortest_path.cpp
)

set(MULTI_OBJECT_TRACKER_HEADERS
  src/data_association/successive_shortest_path/successive_shortest_path.hpp
  include/multi_object_tracker/data_association/data_association.hpp
  include/multi_object_tracker/tracker/tracker.hpp
  include/multi_object_tracker/tracker/model/tracker_base.hpp
  include/multi_object_tracker/tracker/model/bicycle_tracker.hpp
  include/multi_object_tracker/tracker/model/pedestrian_tracker.hpp
  include/multi_object_tracker/tracker/model/vehicle_tracker.hpp
  include/multi_object_tracker/utils/utils.hpp
  include/multi_object_tracker/multi_object_tracker_core.hpp
)

ament_auto_add_library(multi_object_tracker_node SHARED
  ${MULTI_OBJECT_TRACKER_SRC}
  ${MULTI_OBJECT_TRACKER_HEADERS}
)

target_link_libraries(multi_object_tracker_node Eigen3::Eigen)

rclcpp_components_register_node(multi_object_tracker_node
  PLUGIN "MultiObjectTracker"
  EXECUTABLE multi_object_tracker
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
