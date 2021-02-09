cmake_minimum_required(VERSION 3.5)
project(object_merger)

find_package(PCL REQUIRED COMPONENTS common filters)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

include_directories(
  include
)

ament_auto_add_executable(object_association_merger_node
  src/object_association_merger/utils/utils.cpp
  src/object_association_merger/data_association/data_association.cpp
  src/object_association_merger/data_association/successive_shortest_path/successive_shortest_path.cpp
  src/object_association_merger/node.cpp
  src/object_association_merger/main.cpp
)
target_compile_definitions(object_association_merger_node PRIVATE ${PCL_DEFINITIONS})
target_include_directories(object_association_merger_node PRIVATE ${PCL_INCLUDE_DIRS})
# Unfortunately, this one can't be PRIVATE because only the plain or only the
# keyword (PRIVATE) signature of target_link_libraries can be used for one
# target, not both. The plain signature is already used inside
# `ament_target_dependencies` and possibly rosidl_target_interfaces.
target_link_libraries(object_association_merger_node ${PCL_LIBRARIES})
target_link_directories(object_association_merger_node PRIVATE ${PCL_LIBRARY_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
)
