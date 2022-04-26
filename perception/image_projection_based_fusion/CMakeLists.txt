cmake_minimum_required(VERSION 3.14)
project(image_projection_based_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(CUDA_VERBOSE ON)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  # Note: cublas_device was depreciated in CUDA version 9.2
  #       https://forums.developer.nvidia.com/t/where-can-i-find-libcublas-device-so-or-libcublas-device-a/67251/4
  #       In LibTorch, CUDA_cublas_device_LIBRARY is used.
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
PATH_SUFFIXES lib lib64 bin
DOC "CUDNN library."
)
if(CUDNN_LIBRARY)
  if(CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)

  # find dependencies
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_auto REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(PCL REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )

  ### node ###
  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/fusion_node.cpp
    src/debugger.cpp
    src/utils/geometry.cpp
    src/utils/utils.cpp
    src/roi_cluster_fusion/node.cpp
    src/roi_detected_object_fusion/node.cpp
    src/pointpainting_fusion/node.cpp
    src/pointpainting_fusion/pointpainting_trt.cpp
    src/pointpainting_fusion/voxel_generator.cpp
  )

  cuda_add_library(pointpainting_cuda_lib SHARED
    src/pointpainting_fusion/preprocess_kernel.cu
  )
  
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${NVINFER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    pointpainting_cuda_lib
  )

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "image_projection_based_fusion::RoiClusterFusionNode"
    EXECUTABLE roi_cluster_fusion_node
  )

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "image_projection_based_fusion::RoiDetectedObjectFusionNode"
    EXECUTABLE roi_detected_object_fusion_node
  )

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "image_projection_based_fusion::PointpaintingFusionNode"
    EXECUTABLE pointpainting_fusion_node
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
  endif()

  ament_auto_package(INSTALL_TO_SHARE
      launch
      data
      config
  )
else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
  )
endif()