cmake_minimum_required(VERSION 3.5)

cmake_policy(SET CMP0048 NEW) # CMake 3.6 required

if(WIN32)
  cmake_policy(SET CMP0074 NEW) # CMake 3.12 required
endif(WIN32)

project(rs_driver VERSION 1.5.10)

#========================
#  Project setup
#========================
if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

#=============================
#  Compile Features
#=============================
option(DISABLE_PCAP_PARSE         "Disable PCAP file parse" OFF) 
option(ENABLE_TRANSFORM           "Enable transform functions" OFF)

option(ENABLE_DOUBLE_RCVBUF       "Enable double size of RCVBUF" OFF)
option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
option(ENABLE_EPOLL_RECEIVE       "Receive packets with epoll() instead of select()" OFF)

option(ENABLE_STAMP_WITH_LOCAL    "Enable stamp point cloud with local time" OFF)
option(ENABLE_PCL_POINTCLOUD      "Enable PCL Point Cloud" OFF)
option(ENABLE_CRC32_CHECK         "Enable CRC32 Check on MSOP Packet" OFF)
option(ENABLE_DIFOP_PARSE         "Enable parsing DIFOP Packet" OFF)

#=============================
#  Compile Demos, Tools, Tests
#=============================
option(COMPILE_DEMOS "Build rs_driver demos" OFF)
option(COMPILE_TOOLS "Build rs_driver tools" OFF)
option(COMPILE_TOOL_VIEWER "Build point cloud visualization tool" OFF)
option(COMPILE_TOOL_PCDSAVER "Build point cloud pcd saver tool" OFF)
option(COMPILE_TESTS "Build rs_driver unit tests" OFF)

#========================
#  Platform cross setup
#========================
if(MSVC)

  set(COMPILER "MSVC Compiler")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Zi")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
  set(CompilerFlags
    CMAKE_CXX_FLAGS                       CMAKE_C_FLAGS
    CMAKE_CXX_FLAGS_DEBUG                 CMAKE_C_FLAGS_DEBUG
    CMAKE_CXX_FLAGS_RELEASE               CMAKE_C_FLAGS_RELEASE
	  CMAKE_CXX_FLAGS_MINSIZEREL            CMAKE_C_FLAGS_MINSIZEREL
	  CMAKE_CXX_FLAGS_RELWITHDEBINFO        CMAKE_C_FLAGS_RELWITHDEBINFO)

  foreach(CompilerFlag ${CompilerFlags})
    string(REPLACE "/MT" "/MD" ${CompilerFlag} "${${CompilerFlag}}")
  endforeach()

  add_compile_definitions(_DISABLE_EXTENDED_ALIGNED_STORAGE) # to disable a msvc error
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:100000000")

elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")

  if(UNIX)
    set(COMPILER "UNIX GNU Compiler")
  elseif(MINGW)
    set(COMPILER "MINGW Compiler")
  else()
    message(FATAL_ERROR "Unsupported compiler.")
  endif()

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-unused-parameter")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

message(=============================================================)
message("-- CMake run for ${COMPILER}")
message(=============================================================)

#========================
#  Path Setup
#========================
set(CMAKE_INSTALL_PREFIX /usr/local)
set(INSTALL_DRIVER_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_PROJECT_NAME}/include)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake)
set(DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
set(DRIVER_CMAKE_ROOT ${CMAKE_CURRENT_LIST_DIR}/cmake)

if(WIN32)
else()
  if (CMAKE_SYSTEM_NAME STREQUAL "QNX")
    list(APPEND EXTERNAL_LIBS socket)
  else()
    list(APPEND EXTERNAL_LIBS pthread)
  endif()
endif(WIN32)

#========================
#  Build Features
#========================

if(${ENABLE_EPOLL_RECEIVE})

  message(=============================================================)
  message("-- Enable Epoll Receive")
  message(=============================================================)

  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif(${ENABLE_EPOLL_RECEIVE})

if(${DISABLE_PCAP_PARSE})

  message(=============================================================)
  message("-- Disable PCAP parse")
  message(=============================================================)

  add_definitions("-DDISABLE_PCAP_PARSE")

else()

  if(WIN32)
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
    find_package(PCAP REQUIRED)
    include_directories(${PCAP_INCLUDE_DIR})
    list(APPEND EXTERNAL_LIBS ${PCAP_LIBRARY})
  else()
    list(APPEND EXTERNAL_LIBS pcap)
  endif(WIN32)

endif(${DISABLE_PCAP_PARSE})

if(${ENABLE_TRANSFORM})

  message(=============================================================)
  message("-- Enable Transform Fucntions")
  message(=============================================================)

  add_definitions("-DENABLE_TRANSFORM")

  # Eigen
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})

endif(${ENABLE_TRANSFORM})

#============================
#  Build Demos, Tools, Tests
#============================

if(${ENABLE_DOUBLE_RCVBUF})
  add_definitions("-DENABLE_DOUBLE_RCVBUF")
endif(${ENABLE_DOUBLE_RCVBUF})

if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif(${ENABLE_WAIT_IF_QUEUE_EMPTY})

if(${ENABLE_STAMP_WITH_LOCAL})
  add_definitions("-DENABLE_STAMP_WITH_LOCAL")
endif(${ENABLE_STAMP_WITH_LOCAL})

if(${ENABLE_PCL_POINTCLOUD})
  add_definitions("-DENABLE_PCL_POINTCLOUD")
endif(${ENABLE_PCL_POINTCLOUD})

if(${ENABLE_CRC32_CHECK})
  add_definitions("-DENABLE_CRC32_CHECK")
endif(${ENABLE_CRC32_CHECK})

if(${ENABLE_DIFOP_PARSE})
  add_definitions("-DENABLE_DIFOP_PARSE")
endif(${ENABLE_DIFOP_PARSE})

if(${COMPILE_DEMOS})
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/demo)
endif(${COMPILE_DEMOS})

if (${COMPILE_TOOLS})
  set(COMPILE_TOOL_VIEWER ON)
  set(COMPILE_TOOL_PCDSAVER ON)
endif (${COMPILE_TOOLS})

if(${COMPILE_TOOL_VIEWER} OR ${COMPILE_TOOL_PCDSAVER})
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tool)
endif(${COMPILE_TOOL_VIEWER} OR ${COMPILE_TOOL_PCDSAVER})

if(${COMPILE_TESTS})
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test)
endif(${COMPILE_TESTS})

#========================
#  Cmake
#========================  
configure_file(
  ${DRIVER_CMAKE_ROOT}/rs_driverConfig.cmake.in
  ${DRIVER_CMAKE_ROOT}/rs_driverConfig.cmake 
  @ONLY)

configure_file(
  ${DRIVER_CMAKE_ROOT}/rs_driverConfigVersion.cmake.in
  ${DRIVER_CMAKE_ROOT}/rs_driverConfigVersion.cmake 
  @ONLY)

configure_file (
  ${CMAKE_CURRENT_LIST_DIR}/src/rs_driver/macro/version.hpp.in
  ${CMAKE_CURRENT_LIST_DIR}/src/rs_driver/macro/version.hpp
  @ONLY)

if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  set(rs_driver_DIR ${DRIVER_CMAKE_ROOT} PARENT_SCOPE)
endif()

#========================
#  Install & Uninstall
#========================
if(UNIX AND ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

  install(FILES ${DRIVER_CMAKE_ROOT}/rs_driverConfig.cmake
                ${DRIVER_CMAKE_ROOT}/rs_driverConfigVersion.cmake
          DESTINATION ${INSTALL_CMAKE_DIR}/${CMAKE_PROJECT_NAME})

  install(DIRECTORY src/
          DESTINATION ${INSTALL_DRIVER_DIR}
          FILES_MATCHING PATTERN "*.h")

  install(DIRECTORY src/
          DESTINATION ${INSTALL_DRIVER_DIR}
          FILES_MATCHING PATTERN "*.hpp")

  if(NOT TARGET uninstall)
    configure_file(
      ${CMAKE_CURRENT_LIST_DIR}/cmake/cmake_uninstall.cmake.in
      ${PROJECT_BINARY_DIR}/cmake_uninstall.cmake @ONLY)
    add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
  endif(NOT TARGET uninstall)

endif(UNIX AND ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

