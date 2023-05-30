cmake_minimum_required(VERSION 3.5)
project(yabloc_particle_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
# PCL
find_package(PCL REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# glog
find_package(glog REQUIRED)

# GeographicLib
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)

# ===================================================
# Clear ${PYTHON_EXECUTABLE} defined by mrt_cmake_module
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
  abst_corrector
  SHARED
  src/correction/abst_corrector.cpp
  src/common/visualize.cpp
  src/common/mean.cpp
)
target_include_directories(abst_corrector SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(abst_corrector Sophus::Sophus ${PCL_LIBRARIES})
rosidl_target_interfaces(abst_corrector ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Prediction library
ament_auto_add_library(predictor
  src/prediction/predictor.cpp
  src/prediction/resampler.cpp
  src/common/visualize.cpp
  src/common/mean.cpp
)
target_include_directories(predictor SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(predictor Sophus::Sophus ${PCL_LIBRARIES})
rosidl_target_interfaces(predictor ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
ament_auto_add_executable(predictor_node
  src/prediction/predictor_node.cpp
)
target_link_libraries(predictor_node predictor)

# Visualizer
ament_auto_add_executable(particle_visualize
  src/common/particle_visualize_node.cpp
)

# GNSS corrector
set(TARGET gnss_particle_corrector_node)
ament_auto_add_executable(${TARGET}
  src/gnss_corrector/gnss_corrector_node.cpp
  src/gnss_corrector/gnss_corrector_core.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} PUBLIC SYSTEM ${GeographicLib_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${TARGET} Sophus::Sophus)

# Camera corrector
set(TARGET camera_particle_corrector_node)
ament_auto_add_executable(${TARGET}
  src/camera_corrector/camera_particle_corrector_node.cpp
  src/camera_corrector/filt_lsd.cpp
  src/camera_corrector/logit.cpp
  src/camera_corrector/camera_particle_corrector_core.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} Sophus::Sophus ${PCL_LIBRARIES} glog::glog)

# ===================================================
# TEST
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_auto_package(INSTALL_TO_SHARE config)