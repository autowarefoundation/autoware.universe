cmake_minimum_required(VERSION 3.14)
project(nerf_based_localizer)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(CUDA REQUIRED)
find_package(CUDAToolkit REQUIRED)

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda is not found, so the nerf_based_localizer package won't be built.")
  return()
endif()

# libtorch
set(CMAKE_PREFIX_PATH ${DCMAKE_PREFIX_PATH}$ ${CMAKE_SOURCE_DIR}/external/libtorch/)
find_package(Torch PATHS ${CMAKE_SOURCE_DIR}/external/libtorch NO_DEFAULT_PATH)

# skip if libtorch is not found
if(NOT Torch_FOUND)
  return()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS} -w")
link_directories(${CMAKE_SOURCE_DIR}/external/libtorch/lib/)
include_directories(${TORCH_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARIES})
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE SRC_ALL
  src/*.cpp
  src/*.cu)

ament_auto_add_executable(nerf_based_localizer ${SRC_ALL})
ament_export_dependencies(CUDA)
target_link_libraries(nerf_based_localizer
  ${TORCH_LIBRARIES}
  ${TORCH_CUDA_LIBRARIES}
  ${OpenCV_LIBS}
  stdc++fs
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
