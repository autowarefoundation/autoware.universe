cmake_minimum_required(VERSION 3.5)
project(bluetooth_monitor)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Target executable
ament_auto_add_executable(bluetooth_monitor
  src/bluetooth_monitor_node.cpp
  src/bluetooth_monitor.cpp
)

ament_auto_add_executable(l2ping_service
  service/main.cpp
  service/l2ping_service.cpp
  service/l2ping.cpp
)

find_package(Boost REQUIRED COMPONENTS
  serialization
)

## Specify libraries to link a library or executable target against
target_link_libraries(bluetooth_monitor bluetooth ${Boost_LIBRARIES})
target_link_libraries(l2ping_service bluetooth ${Boost_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
