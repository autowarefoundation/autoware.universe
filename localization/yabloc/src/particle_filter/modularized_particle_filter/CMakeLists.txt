cmake_minimum_required(VERSION 3.5)
project(modularized_particle_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_find_build_dependencies()

ament_auto_add_library(
  abst_corrector 
  SHARED 
  src/correction/abst_corrector.cpp
)

ament_auto_add_executable(predictor
  src/prediction/predictor_node.cpp
  src/prediction/predictor.cpp
  src/prediction/resampler.cpp
)

ament_auto_add_executable(lidar_pose_corrector
  src/correction/lidar_pose_corrector_node.cpp
  src/correction/lidar_pose_corrector.cpp
)

ament_auto_add_executable(gnss_pose_corrector
  src/correction/gnss_pose_corrector_node.cpp
  src/correction/gnss_pose_corrector.cpp
)


ament_auto_add_executable(particle_visualize
  src/visualize/particle_visualize_node.cpp
)

install(PROGRAMS
  script/particle_array_to_marker_array.py
  script/particle_array_to_pose_array.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE  launch)