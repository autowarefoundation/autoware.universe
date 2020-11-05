cmake_minimum_required(VERSION 3.5)
project(traffic_light_classifier)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if (CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if (CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif ()
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif (CUDA_FOUND)

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVONNXPARSER nvonnxparser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  if (CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVPARSERS: ${NVPARSERS}")
    message("NVINFER_PLUGIN: ${NVINFER_PLUGIN}")
    message("NVONNXPARSER: ${NVONNXPARSER}")
  endif ()
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
  DOC "CUDNN library." )
if(CUDNN_LIBRARY)
  if (CUDA_VERBOSE)
    message("CUDNN is available!")
    message("CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif ()
  set(CUDNN_AVAIL ON)

else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# Download caffemodel and prototxt
find_program(WGET_AVAIL "wget")
if (NOT WGET_AVAIL)
  message("wget: command not found. External files could not be downloaded.")
endif()
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if (NOT EXISTS "${PATH}")
  execute_process(COMMAND mkdir -p ${PATH})
endif()
set(FILE "${PATH}/traffic_light_classifier_mobilenetv2.onnx")
message(STATUS "Checking and downloading traffic_light_classifier_mobilenetv2.onnx")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget "https://drive.google.com/uc?id=19M64ZAo0XNv-Ep2RDynrRipLg3YAm65e" -O ${PATH}/traffic_light_classifier_mobilenetv2.onnx)
endif()
set(FILE "${PATH}/lamp_labels.txt")
message(STATUS "Checking and downloading lamp_labels.txt")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget "https://drive.google.com/uc?id=1C4XkFe-G58LcDJSVMp5xlwQniGVozgwW" -O ${PATH}/lamp_labels.txt)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  add_definitions(-DENABLE_GPU)

  include_directories(
    utils
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    )

  ament_auto_add_library(libutils SHARED
    utils/trt_common.cpp
  )
  target_link_libraries(libutils
    ${OpenCV_LIBRARIES}
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    ${Boost_LIBRARIES}
  )

  ament_auto_add_library(traffic_light_classifier_nodelet SHARED
    src/color_classifier.cpp
    src/cnn_classifier.cpp
    src/nodelet.cpp
  )
  target_link_libraries(traffic_light_classifier_nodelet
    libutils
    ${OpenCV_LIBRARIES}
  )
  rclcpp_components_register_node(traffic_light_classifier_nodelet
    PLUGIN "traffic_light::TrafficLightClassifierNodelet"
    EXECUTABLE traffic_light_classifier_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    data
    launch
  )
  
else()
  message("CUDA and/or TensorRT were not found. build only color classifier")

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    )

  ament_auto_add_library(traffic_light_classifier_nodelet SHARED
    src/color_classifier.cpp
    src/nodelet.cpp
    )
  target_link_libraries(traffic_light_classifier_nodelet
    ${OpenCV_LIBRARIES}
    )

  rclcpp_components_register_node(traffic_light_classifier_nodelet
    PLUGIN "traffic_light::TrafficLightClassifierNodelet"
    EXECUTABLE traffic_light_classifier_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    data
    launch
  )

endif()
