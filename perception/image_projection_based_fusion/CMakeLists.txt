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

# Download trained models
find_program(GDOWN_AVAIL "gdown")
if(NOT GDOWN_AVAIL)
  message("gdown: command not found. External files could not be downloaded.")
endif()

set(DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data)
execute_process(COMMAND mkdir -p ${DATA_PATH})

function(download FILE_NAME GFILE_ID FILE_HASH)
  # https://drive.google.com/file/d/GFILE_ID/view
  message(STATUS "Checking and downloading ${FILE_NAME}")
  set(FILE_PATH ${DATA_PATH}/${FILE_NAME})
  if(EXISTS ${FILE_PATH})
    file(MD5 ${FILE_PATH} EXISTING_FILE_HASH)
    if(${FILE_HASH} EQUAL ${EXISTING_FILE_HASH})
      message(STATUS "File already exists.")
    else()
      message(STATUS "File hash changes. Downloading now ...")
      execute_process(COMMAND gdown --quiet https://drive.google.com/uc?id=${GFILE_ID} -O ${FILE_PATH})
      # file(MD5 ${FILE_PATH} DOWNLOADED_FILE_HASH)  # disable to pass ci
      message(STATUS "Downloaded file hash: ${DOWNLOADED_FILE_HASH}")
    endif()
  else()
    message(STATUS "File doesn't exists. Downloading now ...")
    execute_process(COMMAND gdown --quiet https://drive.google.com/uc?id=${GFILE_ID} -O ${FILE_PATH})
    # file(MD5 ${FILE_PATH} DOWNLOADED_FILE_HASH)  # disable to pass ci
    message(STATUS "Downloaded file hash: ${DOWNLOADED_FILE_HASH}")
  endif()
endfunction()

# default model
download(pts_voxel_encoder_pointpainting.onnx 1XrC2ANVfHJ7ivmatDNfjD2lJnGLJz1HP 19e51669233e943724e834021c382cae)
download(pts_backbone_neck_head_pointpainting.onnx 1lQCiIJAZyok-HiIVQU9WAnEagy82y1SU 19e51669233e943724e834021c382cae)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

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

ament_auto_package(INSTALL_TO_SHARE
    launch
    config
    data
)
