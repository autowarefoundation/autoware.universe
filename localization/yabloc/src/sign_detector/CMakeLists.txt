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

# ===================================================
# OpenCV
find_package(OpenCV REQUIRED)
# PCL
find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
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
ament_auto_add_library(cv_lsd SHARED  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/opencv_lsd/lsd/lsd.cpp)
target_include_directories(cv_lsd PUBLIC  3rd/opencv_lsd)
target_link_libraries(cv_lsd ${OpenCV_LIBS})

file(GLOB RE_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/3rd/regulatory_elements/lib/*.cpp)
ament_auto_add_library(regulatory_elements SHARED ${RE_SOURCE})
target_include_directories(regulatory_elements PUBLIC  3rd/regulatory_elements/include)

ament_auto_add_library(ll2_util SHARED lib/ll2_util.cpp)
target_include_directories(ll2_util PUBLIC include/ll2_util.hpp 3rd/regulatory_elements/include)
target_link_libraries(ll2_util regulatory_elements)


# ===================================================
# Executable
ament_auto_add_executable(lsd_node src/lsd_node.cpp src/lsd_node.cpp)
target_include_directories(lsd_node PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/opencv_lsd include)
target_link_libraries(lsd_node ${OpenCV_LIBS} cv_lsd)

ament_auto_add_executable(fix_to_pose_node src/fix_to_pose_node.cpp)
target_include_directories(fix_to_pose_node PUBLIC include)
target_link_libraries(fix_to_pose_node ll2_util ${PCL_LIBRARIES} Geographic)

ament_auto_add_executable(pose_to_path_node src/pose_to_path_node.cpp)
target_include_directories(pose_to_path_node PUBLIC include)
target_link_libraries(pose_to_path_node)

ament_auto_add_executable(ll2_to_image_node src/ll2_to_image_node.cpp)
target_include_directories(ll2_to_image_node PUBLIC include)
target_link_libraries(ll2_to_image_node ll2_util Geographic)

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