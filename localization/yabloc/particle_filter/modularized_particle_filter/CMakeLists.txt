cmake_minimum_required(VERSION 3.5)
project(modularized_particle_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_find_build_dependencies()

# ===================================================
# Libraries
ament_auto_add_library(
  abst_corrector
  SHARED
  src/correction/abst_corrector.cpp
  src/common/visualize.cpp
  src/common/mean.cpp
)

ament_auto_add_library(predictor
  src/prediction/predictor.cpp
  src/prediction/resampler.cpp
  src/common/visualize.cpp
  src/common/mean.cpp
)

# ===================================================
# Executables
ament_auto_add_executable(predictor_node
  src/prediction/predictor_node.cpp
)
target_link_libraries(predictor_node predictor)

ament_auto_add_executable(particle_visualize
  src/common/particle_visualize_node.cpp
)

# TEST
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(PROGRAMS
  script/particle_array_to_marker_array.py
  script/particle_array_to_pose_array.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
