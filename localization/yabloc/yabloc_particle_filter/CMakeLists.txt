cmake_minimum_required(VERSION 3.5)
project(yabloc_particle_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

# ===================================================
# PCL
find_package(PCL REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# GeographicLib
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)

# ===================================================
# Clear ${PYTHON_EXECUTABLE} defined by mrt_cmake_module so that rosidl_generate_interfaces can look for it properly
unset(PYTHON_EXECUTABLE)
message(STATUS "PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")

# ===================================================
# Message & Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Particle.msg"
  "msg/ParticleArray.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ===================================================
# Libraries
# correction library
ament_auto_add_library(
  abstract_corrector
  SHARED
  src/correction/abstract_corrector.cpp
  src/common/visualize.cpp
  src/common/mean.cpp
)
target_include_directories(abstract_corrector SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(abstract_corrector Sophus::Sophus ${PCL_LIBRARIES})

# Prediction library
ament_auto_add_library(predictor
  src/prediction/predictor.cpp
  src/prediction/resampler.cpp
  src/common/visualize.cpp
  src/common/mean.cpp
)
target_include_directories(predictor SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(predictor Sophus::Sophus ${PCL_LIBRARIES})

# ros2idl typesupport
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(abstract_corrector ${PROJECT_NAME} "rosidl_typesupport_cpp")
  rosidl_target_interfaces(predictor ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(abstract_corrector "${cpp_typesupport_target}")
  target_link_libraries(predictor "${cpp_typesupport_target}")
endif()

# Cost map Library
ament_auto_add_library(ll2_cost_map
  SHARED
  src/ll2_cost_map/hierarchical_cost_map.cpp
  src/ll2_cost_map/direct_cost_map.cpp
)
target_include_directories(ll2_cost_map PUBLIC include)
target_include_directories(ll2_cost_map SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(ll2_cost_map ${PCL_LIBRARIES})

# ===================================================
# Executables
# Predictor
rclcpp_components_register_node(predictor
  PLUGIN "yabloc::modularized_particle_filter::Predictor"
  EXECUTABLE yabloc_predictor_node
  EXECUTOR SingleThreadedExecutor
)

# Visualizer
ament_auto_add_library(particle_visualize
  src/common/particle_visualize_node.cpp
)
rclcpp_components_register_node(particle_visualize
  PLUGIN "yabloc::modularized_particle_filter::ParticleVisualize"
  EXECUTABLE yabloc_particle_visualize_node
  EXECUTOR SingleThreadedExecutor
)

# GNSS corrector
set(TARGET gnss_particle_corrector)
ament_auto_add_library(${TARGET}
  src/common/particle_visualize_node.cpp
  src/gnss_corrector/gnss_corrector_core.cpp
)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} PUBLIC SYSTEM ${GeographicLib_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${TARGET} Sophus::Sophus)
rclcpp_components_register_node(${TARGET}
  PLUGIN "yabloc::modularized_particle_filter::GnssParticleCorrector"
  EXECUTABLE yabloc_gnss_particle_corrector_node
  EXECUTOR SingleThreadedExecutor
)


# Camera corrector
set(TARGET camera_particle_corrector_node)
ament_auto_add_library(${TARGET}
  src/camera_corrector/filter_line_segments.cpp
  src/camera_corrector/logit.cpp
  src/camera_corrector/camera_particle_corrector_core.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} Sophus::Sophus ${PCL_LIBRARIES})
rclcpp_components_register_node(${TARGET}
  PLUGIN "yabloc::modularized_particle_filter::CameraParticleCorrector"
  EXECUTABLE yabloc_camera_particle_corrector_node
  EXECUTOR SingleThreadedExecutor
)

# ===================================================
# TEST
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
