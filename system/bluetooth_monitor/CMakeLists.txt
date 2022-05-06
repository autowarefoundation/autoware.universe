cmake_minimum_required(VERSION 3.5)
project(bluetooth_monitor)

### Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

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
