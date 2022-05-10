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
find_library(GeographicLib_LIBRARIES  NAMES Geographic)
include_directories(${GeographicLib_INCLUDE_DIRS})

# ===================================================
# Library
macro(ADD_LIB)
  cmake_parse_arguments(_ARG "" "SRC;TARGET"  "LIB;INC" ${ARGN})
  file(GLOB SOURCE_TMP ${_ARG_SRC})
  ament_auto_add_library(${_ARG_TARGET} SHARED ${SOURCE_TMP})
  target_include_directories(${_ARG_TARGET} PUBLIC ${_ARG_INC})
  target_link_libraries(${_ARG_TARGET} ${_ARG_LIB})
  target_compile_definitions(${_ARG_TARGET} PUBLIC SOPHUS_USE_BASIC_LOGGING=1)
endmacro()

ADD_LIB(SRC 3rd/opencv_lsd/lsd/*.cpp          TARGET cv_lsd LIB ${OpenCV_LIBS} INC 3rd/opencv_lsd)
ADD_LIB(SRC 3rd/regulatory_elements/lib/*.cpp TARGET regulatory_elements       INC 3rd/regulatory_elements/include)
ADD_LIB(SRC lib/ll2_util.cpp                  TARGET ll2_util                  INC 3rd/regulatory_elements/include)
ADD_LIB(SRC lib/particle_filter.cpp           TARGET pf                        INC include)
ADD_LIB(SRC lib/util.cpp                      TARGET util                      INC include)
ADD_LIB(SRC lib/lsd.cpp                       TARGET lsd util                  INC include)

# ===================================================
# Executable
macro(ADD_EXE)
  cmake_parse_arguments(_ARG "" "SRC"  "LIB;INC" ${ARGN})
  get_filename_component(TARGET_NAME ${_ARG_SRC} NAME_WE)
  ament_auto_add_executable(${TARGET_NAME} ${_ARG_SRC})
  target_include_directories(${TARGET_NAME} PUBLIC ${_ARG_INC})
  target_link_libraries(${TARGET_NAME} ${_ARG_LIB})
endmacro()

ADD_EXE(SRC src/lsd_node.cpp          INC 3rd/opencv_lsd include LIB cv_lsd lsd util ${OpenCV_LIBS})
ADD_EXE(SRC src/ll2_to_image_node.cpp INC                include LIB ll2_util Geographic )
ADD_EXE(SRC src/fix_to_pose_node.cpp  INC                include LIB ll2_util Geographic ${PCL_LIBRARIES})
ADD_EXE(SRC src/pose_to_path_node.cpp INC                include LIB )
ADD_EXE(SRC src/imu_kf_node.cpp       INC                include LIB )
ADD_EXE(SRC src/particle_filter_node.cpp INC             include LIB pf)
ADD_EXE(SRC src/optflow_node.cpp         INC             include LIB util)

# ===================================================
# Python
ament_python_install_package(scripts)
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  #FILES_MATCHING PATTERN "*.py" # TODO: this option disables symlink-install. i dont know why
)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE  launch config)