cmake_minimum_required(VERSION 3.5)
project(system_monitor)

## Compile as C++14, supported in ROS Melodic and newer
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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(NVML)

###########
## Build ##
###########

## Specify additional locations of header files

find_path(LIBNL3_INCLUDE_DIRS
  NAMES netlink/netlink.h
  PATH_SUFFIXES libnl3
)

if (NVML_FOUND)
  include_directories(
    include
    ${LIBNL3_INCLUDE_DIRS}
    ${NVML_INCLUDE_DIRS}
  )
else()
  include_directories(
    include
    ${LIBNL3_INCLUDE_DIRS}
  )
endif()

## Declare a C++ executable

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(CMAKE_CPU_PLATFORM "intel")
  ADD_DEFINITIONS(-D_CPU_INTEL_)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
  if (CMAKE_HOST_SYSTEM_VERSION MATCHES ".*raspi.*")
    set(CMAKE_CPU_PLATFORM "raspi")
    ADD_DEFINITIONS(-D_CPU_RASPI_)
  elseif (CMAKE_HOST_SYSTEM_VERSION MATCHES ".*tegra.*")
    set(CMAKE_CPU_PLATFORM "tegra")
    ADD_DEFINITIONS(-D_CPU_TEGRA_)
  else()
    set(CMAKE_CPU_PLATFORM "arm")
    ADD_DEFINITIONS(-D_CPU_ARM_)
  endif()
else()
  set(CMAKE_CPU_PLATFORM "unknown")
endif()

if (NVML_FOUND)
  set(CMAKE_GPU_PLATFORM "nvml")
  ADD_DEFINITIONS(-D_GPU_NVML_)
  set(GPU_LIBRARY ${NVML_LIBRARIES})
else()
  if (CMAKE_CPU_PLATFORM STREQUAL "tegra")
    set(CMAKE_GPU_PLATFORM "tegra")
    ADD_DEFINITIONS(-D_GPU_TEGRA_)
  else()
    set(CMAKE_GPU_PLATFORM "unknown")
  endif()
endif()

message(STATUS "HOST_SYSTEM_VERSION: " ${CMAKE_HOST_SYSTEM_VERSION})
message(STATUS "SYSTEM_PROCESSOR: " ${CMAKE_SYSTEM_PROCESSOR})
message(STATUS "CPU PLATFORM: " ${CMAKE_CPU_PLATFORM})
message(STATUS "GPU PLATFORM: " ${CMAKE_GPU_PLATFORM})

set(CPU_MONITOR_SOURCE
  src/cpu_monitor/cpu_monitor_base.cpp
  src/cpu_monitor/${CMAKE_CPU_PLATFORM}_cpu_monitor.cpp
)

ament_auto_add_executable(cpu_monitor
  src/cpu_monitor/cpu_monitor_node.cpp
  ${CPU_MONITOR_SOURCE}
)

ament_auto_add_executable(hdd_monitor
  src/hdd_monitor/hdd_monitor_node.cpp
  src/hdd_monitor/hdd_monitor.cpp
)

ament_auto_add_executable(mem_monitor
  src/mem_monitor/mem_monitor_node.cpp
  src/mem_monitor/mem_monitor.cpp
)

ament_auto_add_executable(net_monitor
  src/net_monitor/net_monitor_node.cpp
  src/net_monitor/net_monitor.cpp
  src/net_monitor/nl80211.cpp
)

ament_auto_add_executable(ntp_monitor
  src/ntp_monitor/ntp_monitor_node.cpp
  src/ntp_monitor/ntp_monitor.cpp
)

ament_auto_add_executable(process_monitor
  src/process_monitor/process_monitor_node.cpp
  src/process_monitor/process_monitor.cpp
)

set(GPU_MONITOR_SOURCE
  src/gpu_monitor/gpu_monitor_base.cpp
  src/gpu_monitor/${CMAKE_GPU_PLATFORM}_gpu_monitor.cpp
)
ament_auto_add_executable(gpu_monitor
  src/gpu_monitor/gpu_monitor_node.cpp
  ${GPU_MONITOR_SOURCE}
)

ament_auto_add_executable(msr_reader
  reader/msr_reader/msr_reader.cpp
)

ament_auto_add_executable(hdd_reader
  reader/hdd_reader/hdd_reader.cpp
)

find_library(NL3 nl-3 REQUIRED)
find_library(NLGENL3 nl-genl-3 REQUIRED)
list(APPEND NL_LIBS ${NL3} ${NLGENL3})

find_package(Boost REQUIRED COMPONENTS
  serialization
  thread
  filesystem
  regex
)

## Specify libraries to link a library or executable target against
target_link_libraries(cpu_monitor ${Boost_LIBRARIES})
target_link_libraries(hdd_monitor ${Boost_LIBRARIES})
target_link_libraries(net_monitor ${NL_LIBS})
target_link_libraries(ntp_monitor ${Boost_LIBRARIES})
target_link_libraries(process_monitor ${Boost_LIBRARIES})
target_link_libraries(gpu_monitor ${GPU_LIBRARY})
target_link_libraries(msr_reader ${Boost_LIBRARIES})
target_link_libraries(hdd_reader ${Boost_LIBRARIES})

# TODO(yunus.caliskan): Port the tests to ROS2, robustify the tests.
if(BUILD_TESTING)                                                                                                                                                                                                  
  find_package(ament_lint_auto REQUIRED)                                                                                                                                                                           
  ament_lint_auto_find_test_dependencies()                                                                                                                                                                         
endif()  

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)