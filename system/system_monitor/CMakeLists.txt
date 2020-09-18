cmake_minimum_required(VERSION 3.0.2)
project(system_monitor)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roslint
  roscpp
  std_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(NVML)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS std_msgs
)

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
    ${catkin_INCLUDE_DIRS}
    ${LIBNL3_INCLUDE_DIRS}
    ${NVML_INCLUDE_DIRS}
  )
else()
  include_directories(
    include
    ${LIBNL3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
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

add_executable(cpu_monitor
  src/cpu_monitor/cpu_monitor_node.cpp
  ${CPU_MONITOR_SOURCE}
)

add_executable(hdd_monitor
  src/hdd_monitor/hdd_monitor_node.cpp
  src/hdd_monitor/hdd_monitor.cpp
)
add_executable(mem_monitor
  src/mem_monitor/mem_monitor_node.cpp
  src/mem_monitor/mem_monitor.cpp
)
add_executable(net_monitor
  src/net_monitor/net_monitor_node.cpp
  src/net_monitor/net_monitor.cpp
  src/net_monitor/nl80211.cpp
)
add_executable(ntp_monitor
  src/ntp_monitor/ntp_monitor_node.cpp
  src/ntp_monitor/ntp_monitor.cpp
)
add_executable(process_monitor
  src/process_monitor/process_monitor_node.cpp
  src/process_monitor/process_monitor.cpp
)

set(GPU_MONITOR_SOURCE
  src/gpu_monitor/gpu_monitor_base.cpp
  src/gpu_monitor/${CMAKE_GPU_PLATFORM}_gpu_monitor.cpp
)
add_executable(gpu_monitor
  src/gpu_monitor/gpu_monitor_node.cpp
  ${GPU_MONITOR_SOURCE}
)

add_executable(msr_reader
  reader/msr_reader/msr_reader.cpp
)

add_executable(hdd_reader
  reader/hdd_reader/hdd_reader.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(cpu_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(hdd_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(mem_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(net_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(ntp_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(process_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(gpu_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(msr_reader ${catkin_EXPORTED_TARGETS})
add_dependencies(hdd_reader ${catkin_EXPORTED_TARGETS})

find_library(NL3 nl-3)
find_library(NLGENL3 nl-genl-3)
list(APPEND NL_LIBS ${NL3} ${NLGENL3})

find_package(Boost REQUIRED COMPONENTS
  serialization
)

## Specify libraries to link a library or executable target against
target_link_libraries(cpu_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(hdd_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(mem_monitor ${catkin_LIBRARIES})
target_link_libraries(net_monitor ${catkin_LIBRARIES} ${NL_LIBS})
target_link_libraries(ntp_monitor ${catkin_LIBRARIES})
target_link_libraries(process_monitor ${catkin_LIBRARIES})
target_link_libraries(gpu_monitor ${catkin_LIBRARIES} ${GPU_LIBRARY})
target_link_libraries(msr_reader ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(hdd_reader ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

## Mark executables for installation
install(
  TARGETS
    cpu_monitor
    hdd_monitor
    mem_monitor
    net_monitor
    ntp_monitor
    process_monitor
    gpu_monitor
    msr_reader
    hdd_reader
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_cpu_monitor
    test/test_cpu_monitor.test
    test/src/cpu_monitor/test_${CMAKE_CPU_PLATFORM}_cpu_monitor.cpp
    ${CPU_MONITOR_SOURCE}
  )
  target_link_libraries(test_cpu_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  add_rostest_gtest(test_hdd_monitor
    test/test_hdd_monitor.test
    test/src/hdd_monitor/test_hdd_monitor.cpp
    src/hdd_monitor/hdd_monitor.cpp
  )
  target_link_libraries(test_hdd_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  add_rostest_gtest(test_mem_monitor
    test/test_mem_monitor.test
    test/src/mem_monitor/test_mem_monitor.cpp
    src/mem_monitor/mem_monitor.cpp
  )
  target_link_libraries(test_mem_monitor ${catkin_LIBRARIES})

  add_rostest_gtest(test_net_monitor
    test/test_net_monitor.test
    test/src/net_monitor/test_net_monitor.cpp
    src/net_monitor/net_monitor.cpp
    src/net_monitor/nl80211.cpp
  )
  target_link_libraries(test_net_monitor ${catkin_LIBRARIES} ${NL_LIBS})

  add_rostest_gtest(test_ntp_monitor
    test/test_ntp_monitor.test
    test/src/ntp_monitor/test_ntp_monitor.cpp
    src/ntp_monitor/ntp_monitor.cpp
  )
  target_link_libraries(test_ntp_monitor ${catkin_LIBRARIES})

  add_rostest_gtest(test_process_monitor
    test/test_process_monitor.test
    test/src/process_monitor/test_process_monitor.cpp
    src/process_monitor/process_monitor.cpp
  )
  target_link_libraries(test_process_monitor ${catkin_LIBRARIES})

  add_rostest_gtest(test_gpu_monitor
    test/test_gpu_monitor.test
    test/src/gpu_monitor/test_${CMAKE_GPU_PLATFORM}_gpu_monitor.cpp
    ${GPU_MONITOR_SOURCE}
  )
  target_link_libraries(test_gpu_monitor ${catkin_LIBRARIES} ${GPU_LIBRARY})

  # Dummy executables
  add_executable(mpstat1 test/src/cpu_monitor/mpstat1.cpp)
  add_executable(mpstat2 test/src/cpu_monitor/mpstat2.cpp)
  add_executable(df1 test/src/hdd_monitor/df1.cpp)
  add_executable(free1 test/src/mem_monitor/free1.cpp)
  add_executable(ntpdate1 test/src/ntp_monitor/ntpdate1.cpp)
  add_executable(top1 test/src/process_monitor/top1.cpp)
  add_executable(top2 test/src/process_monitor/top2.cpp)
  add_executable(top3 test/src/process_monitor/top3.cpp)
  add_executable(echo1 test/src/process_monitor/sort1.cpp)
  add_executable(sort1 test/src/process_monitor/sort1.cpp)
  add_executable(sed1 test/src/process_monitor/sed1.cpp)

  install(TARGETS mpstat1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS mpstat2 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS df1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS free1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS ntpdate1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS top1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS top2 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS top3 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS echo1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS sort1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS sed1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()
