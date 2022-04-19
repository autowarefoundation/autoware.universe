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
# Library
ament_auto_add_library(cv_lsd SHARED  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/opencv_lsd/lsd/lsd.cpp)
target_include_directories(cv_lsd PUBLIC  3rd/opencv_lsd)
target_link_libraries(cv_lsd ${OpenCV_LIBS})

# ===================================================
# Executable
ament_auto_add_executable(main src/main.cpp)
target_include_directories(main PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/opencv_lsd include)
target_link_libraries(main  ${OpenCV_LIBS} cv_lsd)


# ===================================================
# Python
ament_python_install_package(scripts)
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE  launch)