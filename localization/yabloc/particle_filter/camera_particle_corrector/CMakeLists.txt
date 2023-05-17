cmake_minimum_required(VERSION 3.5)
project(camera_particle_corrector)

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

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common)

# glog
find_package(glog REQUIRED)

# ===================================================
# Executable
set(TARGET camera_particle_corrector_node)
ament_auto_add_executable(${TARGET}
  src/camera_particle_corrector_node.cpp
  src/filt_lsd.cpp
  src/logit.cpp
  src/camera_particle_corrector_core.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} Sophus::Sophus ${PCL_LIBRARIES} glog::glog)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config)