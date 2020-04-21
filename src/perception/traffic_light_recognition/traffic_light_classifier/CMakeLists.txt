cmake_minimum_required(VERSION 2.8.3)
project(traffic_light_classifier)

add_compile_options(-std=c++14)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if (CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib)
  message("CUDA is available!")
  message("CUDA Libs: ${CUDA_LIBRARIES}")
  message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
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
  message("TensorRT is available!")
  message("NVINFER: ${NVINFER}")
  message("NVPARSERS: ${NVPARSERS}")
  message("NVINFER_PLUGIN: ${NVINFER_PLUGIN}")
  message("NVONNXPARSER: ${NVONNXPARSER}")
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
  message("CUDNN is available!")
  message("CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
  message_filters
  dynamic_reconfigure
  )

generate_dynamic_reconfigure_options(cfg/HSVFilter.cfg)

find_package(OpenCV REQUIRED)

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)

  include_directories(
    include
    utils
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    )

  # Download caffemodel and prototxt
  set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
  if (NOT EXISTS "${PATH}")
    execute_process(COMMAND mkdir -p ${PATH})
  endif()
  set(FILE "${PATH}/traffic_light_classifier_mobilenetv2.onnx")
  message(STATUS "Checking and downloading traffic_light_classifier_mobilenetv2.onnx")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND gdown "https://drive.google.com/uc?id=1Q-pZNTUBDNWddcURARrvtz2jmTiqV5T7" -O ${PATH}/traffic_light_classifier_mobilenetv2.onnx)
  endif()
  set(FILE "${PATH}/lamp_labels.txt")
  message(STATUS "Checking and downloading lamp_labels.txt")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND gdown "https://drive.google.com/uc?id=15fxhS2zDAU0aa_cLkrhMKo_1ZY-JnWTE" -O ${PATH}/lamp_labels.txt)
  endif()


  ###########
  ## Build ##
  ###########

  add_library(libutils
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
    )
  add_dependencies(libutils
    ${catkin_EXPORTED_TARGETS})


  add_executable(traffic_light_classifier_node
    src/color_classifier.cpp
    src/cnn_classifier.cpp
    src/node.cpp
    src/main.cpp
    )

  add_dependencies(traffic_light_classifier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
    )
  target_link_libraries(traffic_light_classifier_node
    libutils
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

  #############
  ## Install ##
  #############

  install(
    TARGETS
    traffic_light_classifier_node
    libutils
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(
    DIRECTORY
    launch
    data
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

else()
  find_package(catkin REQUIRED)
  catkin_package()
  message("TrafficLightClassifier won't be built, CUDA and/or TensorRT were not found.")
endif()
