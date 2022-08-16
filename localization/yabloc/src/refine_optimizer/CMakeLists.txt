cmake_minimum_required(VERSION 3.5)
project(refine_optimizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclpy REQUIRED)

# ===================================================
# YAML
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# ceres
find_package(Ceres REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# ===================================================
include_directories(
  SYSTEM
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIRS}
)

# ===================================================
# Library
macro(ADD_LIB)
  cmake_parse_arguments(_ARG "" "TARGET" "SRC;LIB;INC" ${ARGN})

  set(SOURCE "")

  foreach(QUERY IN ITEMS ${_ARG_SRC})
    file(GLOB SOURCE_TMP ${QUERY})
    set(SOURCE "${SOURCE};${SOURCE_TMP}")
  endforeach()

  ament_auto_add_library(${_ARG_TARGET} SHARED ${SOURCE})
  target_include_directories(${_ARG_TARGET} PUBLIC ${_ARG_INC})
  target_link_libraries(${_ARG_TARGET} ${_ARG_LIB})
  target_compile_definitions(${_ARG_TARGET} PUBLIC SOPHUS_USE_BASIC_LOGGING=1)
endmacro()

# ===================================================
# Executable
macro(ADD_EXE)
  cmake_parse_arguments(_ARG "" "TARGET" "SRC;LIB;INC" ${ARGN})

  set(SOURCE "")

  foreach(QUERY IN ITEMS ${_ARG_SRC})
    file(GLOB SOURCE_TMP ${QUERY})
    set(SOURCE "${SOURCE};${SOURCE_TMP}")
  endforeach()

  set(TARGET_NAME ${_ARG_TARGET})
  ament_auto_add_executable(${TARGET_NAME} ${SOURCE})
  target_include_directories(${TARGET_NAME} PUBLIC ${_ARG_INC})
  target_link_libraries(${TARGET_NAME} ${_ARG_LIB})
endmacro()

ADD_LIB(TARGET refine_lib SRC src/refine_optimizer.cpp INC include LIB ${CERES_LIBRARIES})
ADD_EXE(TARGET refine_node SRC src/refine_node.cpp INC include LIB refine_lib)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE )