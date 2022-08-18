cmake_minimum_required(VERSION 3.5)
project(gnss_particle_corrector)

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

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# ===================================================
include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
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
# Executable
ament_auto_add_executable(gnss_particle_corrector src/gnss_corrector_node.cpp src/gnss_corrector_core.cpp)
target_include_directories(gnss_particle_corrector PUBLIC include)

# ===================================================
ament_auto_package()