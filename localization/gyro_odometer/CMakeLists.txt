cmake_minimum_required(VERSION 3.5)
project(gyro_odometer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(GRYO_ODOMETER_SRC
  src/gyro_odometer_core.cpp)

set(GRYO_ODOMETER_HEADERS
  include/gyro_odometer/gyro_odometer_core.h)

ament_auto_add_executable(${PROJECT_NAME}
  src/gyro_odometer_node.cpp
  ${GRYO_ODOMETER_SRC}
  ${GRYO_ODOMETER_HEADERS}
 )

ament_auto_package(INSTALL_TO_SHARE
  launch
)
