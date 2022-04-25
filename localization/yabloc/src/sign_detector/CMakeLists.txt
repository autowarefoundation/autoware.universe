cmake_minimum_required(VERSION 3.5)
project(sign_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ===================================================
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

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
target_link_libraries(regulatory_elements ${OpenCV_LIBS})

# ===================================================
# Executable
ament_auto_add_executable(lsd_node src/lsd_node.cpp src/lsd.cpp)
target_include_directories(lsd_node PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/opencv_lsd include)
target_link_libraries(lsd_node ${OpenCV_LIBS} cv_lsd)

ament_auto_add_executable(map_node src/map_node.cpp)
target_include_directories(map_node PUBLIC include 3rd/regulatory_elements/include)
target_link_libraries(map_node  regulatory_elements )

ament_auto_add_executable(eagleye_node src/eagleye_node.cpp)
target_include_directories(eagleye_node PUBLIC include )
target_link_libraries(eagleye_node Geographic)

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