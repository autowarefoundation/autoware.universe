cmake_minimum_required(VERSION 3.5)
project(sign_detector)

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

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(Sophus REQUIRED)

# ===================================================
# YAML
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIRS}
)

# ===================================================
# GeographicLib
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PAHT_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)
include_directories(${GeographicLib_INCLUDE_DIRS})

# ===================================================
# Library
macro(ADD_LIB)
  cmake_parse_arguments(_ARG "" "SRC;TARGET" "LIB;INC" ${ARGN})
  file(GLOB SOURCE_TMP ${_ARG_SRC})
  ament_auto_add_library(${_ARG_TARGET} SHARED ${SOURCE_TMP})
  target_include_directories(${_ARG_TARGET} PUBLIC ${_ARG_INC})
  target_link_libraries(${_ARG_TARGET} ${_ARG_LIB})
  target_compile_definitions(${_ARG_TARGET} PUBLIC SOPHUS_USE_BASIC_LOGGING=1)
endmacro()

ADD_LIB(TARGET cv_lsd SRC 3rd/opencv_lsd/lsd/*.cpp INC 3rd/opencv_lsd LIB ${OpenCV_LIBS})
ADD_LIB(TARGET regulatory_elements SRC 3rd/regulatory_elements/lib/*.cpp INC 3rd/regulatory_elements/include)
ADD_LIB(TARGET util SRC lib/*.cpp INC 3rd/regulatory_elements/include include)
ADD_LIB(TARGET cost_map SRC src/particle_filter/hierarchical_cost_map.cpp INC include)

# ===================================================
# Executable
macro(ADD_EXE)
  cmake_parse_arguments(_ARG "" "SRC;TARGET" "LIB;INC" ${ARGN})

  # get_filename_component(TARGET_NAME ${_ARG_SRC} NAME_WE)
  file(GLOB SOURCE_TMP ${_ARG_SRC})
  set(TARGET_NAME ${_ARG_TARGET})
  ament_auto_add_executable(${TARGET_NAME} ${SOURCE_TMP})
  target_include_directories(${TARGET_NAME} PUBLIC ${_ARG_INC})
  target_link_libraries(${TARGET_NAME} ${_ARG_LIB})
endmacro()

ADD_EXE(TARGET fix_to_pose_node SRC src/trajectory/fix_to_pose_node.cpp INC LIB util Geographic ${PCL_LIBRARIES})
ADD_EXE(TARGET pose_to_path_node SRC src/trajectory/pose_to_path_node.cpp INC LIB)

ADD_EXE(TARGET lsd_node SRC src/lsd_*.cpp INC 3rd/opencv_lsd LIB util cv_lsd ${OpenCV_LIBS})
ADD_EXE(TARGET ll2_to_image_node SRC src/ll2_to_image_*.cpp INC LIB util Geographic)
ADD_EXE(TARGET undistort_node SRC src/undistort_node.cpp INC LIB util)

ADD_EXE(TARGET camera_corrector_node SRC
  src/particle_filter/camera_corrector*.cpp
  INC include LIB util cost_map)

ADD_EXE(TARGET gnss_corrector_node SRC src/particle_filter/gnss_corrector*.cpp INC include LIB util Geographic)

ADD_EXE(TARGET twist_node SRC src/trajectory/twist_*.cpp INC include LIB util Geographic)
ADD_EXE(TARGET pose_init_node SRC src/trajectory/pose_initializer.cpp INC include LIB util Geographic)

ADD_EXE(TARGET overlay_node SRC src/validation/overlay_*.cpp INC include LIB util)
ADD_EXE(TARGET free_space_node SRC src/validation/freespace_*.cpp INC include LIB util)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE launch config)