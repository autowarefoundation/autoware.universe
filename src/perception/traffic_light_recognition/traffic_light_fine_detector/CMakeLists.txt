cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++14)

project(traffic_light_fine_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  message_filters
  sensor_msgs
  std_msgs
  autoware_perception_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else ()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

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

# Download onnx model
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if (NOT EXISTS "${PATH}")
  execute_process(COMMAND mkdir -p ${PATH})
endif()
set(FILE "${PATH}/yolov3-tlr.onnx")
message(STATUS "Checking and downloading yolov3-tlr.onnx")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND gdown "https://drive.google.com/uc?id=1MdDX37d_VSHciPeKwlckFjbyTxOq6pqz" -O ${PATH}/yolov3-tlr.onnx)

endif()


if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${Opencv_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )

  catkin_package(
    INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
  )

  set(CMAKE_CXX_FLAGS "-O2 -Wall -Wunused-variable ${CMAKE_CXX_FLAGS} -fpic -std=c++11 -pthread")

  ### tlr_detector ###
  add_library(tlrdetector
    lib/src/EntropyCalibrator.cpp
    lib/src/TrtNet.cpp
  )

  target_include_directories(tlrdetector PUBLIC
    lib/include
  )

  target_link_libraries(tlrdetector
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
  )

  add_executable(traffic_light_fine_detector
    src/traffic_light_fine_detector_main.cpp
    src/traffic_light_fine_detector.cpp
  )
  target_link_libraries(traffic_light_fine_detector
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    tlrdetector
  )
  add_dependencies(traffic_light_fine_detector
    ${catkin_EXPORTED_TARGETS}
  )

  add_executable(traffic_light_roi_image_saver_node
    src/roi_image_saver/node.cpp
    src/roi_image_saver/main.cpp)
  add_dependencies(traffic_light_roi_image_saver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(traffic_light_roi_image_saver_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )

  install(TARGETS
    traffic_light_fine_detector
    tlrdetector
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
  message("TrafficLightFineDetector won't be built, CUDA and/or TensorRT were not found.")
endif()

