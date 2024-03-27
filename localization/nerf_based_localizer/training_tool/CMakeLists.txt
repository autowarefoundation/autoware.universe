cmake_minimum_required(VERSION 3.18)
project(nerf_training_tool
    VERSION 0.1
    LANGUAGES CUDA CXX)

include(CheckLanguage)
check_language(CUDA)

set(CMAKE_CXX_STANDARD 17)

# ##############################################################################
# CUDA compiler setup
# ##############################################################################
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)
set(CUDA_LINK_LIBRARIES_KEYWORD PUBLIC)

# Libtorch
set(CMAKE_PREFIX_PATH ${DCMAKE_PREFIX_PATH}$ ${CMAKE_SOURCE_DIR}/../external/libtorch/)
find_package(Torch REQUIRED PATHS ${CMAKE_SOURCE_DIR}/../external/libtorch NO_DEFAULT_PATH)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
link_directories(${CMAKE_SOURCE_DIR}/../external/libtorch/lib/)
include_directories(${TORCH_INCLUDE_DIRS})

# eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE SRC_ALL
    src/*.cpp
    src/*.cu
    ../src/nerf/*.cpp
    ../src/nerf/*.cu)

set(ALL_LIBS ${TORCH_LIBRARIES}
    ${TORCH_CUDA_LIBRARIES}
    ${OpenCV_LIBS}
    stdc++fs)

# ------------------------------------ Add executables ---------------------------------------
add_executable(main ${SRC_ALL})
target_link_libraries(main ${ALL_LIBS})
set_property(TARGET main PROPERTY CXX_STANDARD 17)
