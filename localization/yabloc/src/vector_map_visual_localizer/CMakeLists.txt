cmake_minimum_required(VERSION 3.5)
project(vector_map_visual_localizer)

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

# ceres
find_package(Ceres REQUIRED)

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

ADD_LIB(TARGET cv_lsd SRC 3rd/opencv_lsd/lsd/*.cpp INC 3rd/opencv_lsd LIB ${OpenCV_LIBS})
ADD_LIB(TARGET regulatory_elements SRC 3rd/regulatory_elements/lib/*.cpp INC 3rd/regulatory_elements/include)
ADD_LIB(TARGET ll2_util SRC lib/ll2_util.cpp INC 3rd/regulatory_elements/include include)
ADD_LIB(TARGET util SRC lib/util.cpp lib/color.cpp lib/base_camera_info_node.cpp INC include)
ADD_LIB(TARGET cost_map SRC src/particle_filter/hierarchical_cost_map.cpp INC include)
ADD_LIB(TARGET modified_seg SRC lib/graph_segmentation.cpp INC include)
ADD_LIB(TARGET vanish SRC lib/ransac_vanish_point.cpp INC include)
ADD_LIB(TARGET orientation SRC lib/orientation_optimizer.cpp INC include LIB ${CERES_LIBRARIES})

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

# imgproc
ADD_EXE(TARGET vanish_node SRC src/imgproc/vanish_*.cpp INC include LIB util cv_lsd vanish orientation)
ADD_EXE(TARGET lsd_node SRC src/imgproc/lsd_*.cpp INC 3rd/opencv_lsd LIB util cv_lsd ${OpenCV_LIBS})
ADD_EXE(TARGET undistort_node SRC src/imgproc/undistort_node.cpp INC LIB util)
ADD_EXE(TARGET segment_filter_node SRC src/imgproc/segment_filter_node.cpp INC LIB util)
ADD_EXE(TARGET graph_segment_node SRC src/imgproc/graph_segment_node.cpp INC LIB util)

# trajectory
ADD_EXE(TARGET fix_to_pose_node SRC src/trajectory/fix_to_pose_node.cpp INC LIB util Geographic ${PCL_LIBRARIES})
ADD_EXE(TARGET pose_to_path_node SRC src/trajectory/pose_to_path_node.cpp INC LIB)
ADD_EXE(TARGET twist_node SRC src/trajectory/twist_*.cpp INC include LIB util Geographic)
ADD_EXE(TARGET pose_init_node SRC src/trajectory/pose_initializer.cpp INC include LIB util Geographic)

# particle_filter
ADD_EXE(TARGET camera_corrector_node
  SRC src/particle_filter/camera_corrector*.cpp
  INC include LIB util cost_map)
ADD_EXE(TARGET gnss_corrector_node SRC src/particle_filter/gnss_corrector*.cpp INC include LIB util Geographic)
ADD_EXE(TARGET sign_corrector_node SRC src/particle_filter/sign_corrector_*.cpp INC include LIB util)

# validation
ADD_EXE(TARGET overlay_node SRC src/validation/overlay_*.cpp INC include LIB util)
ADD_EXE(TARGET covariance_node SRC src/validation/covariance_*.cpp INC include LIB util)
ADD_EXE(TARGET ape_node SRC src/validation/ape_*.cpp INC include LIB util)
ADD_LIB(TARGET refine_lib SRC src/validation/refine_optimizer.cpp INC include LIB ${CERES_LIBRARIES})
ADD_EXE(TARGET refine_node SRC src/validation/refine_node.cpp INC include LIB util refine_lib)

# map
ADD_EXE(TARGET ll2_decompose_node SRC src/map/ll2_decompose_*.cpp INC LIB ll2_util Geographic)
ADD_EXE(TARGET ground_server_node SRC src/map/ground_server_*.cpp INC LIB)
ADD_EXE(TARGET ground_client_node SRC src/map/ground_client_*.cpp INC LIB)

# sign_board
ADD_LIB(TARGET sign_lib SRC src/sign_board/sign_lib.cpp INC include LIB)
ADD_EXE(TARGET sign_node SRC src/sign_board/sign_detector.cpp INC LIB util sign_lib)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE launch config)