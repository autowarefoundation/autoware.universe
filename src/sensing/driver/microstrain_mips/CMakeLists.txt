cmake_minimum_required(VERSION 2.8.3)
project(microstrain_mips)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  message_generation
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  diagnostic_updater
)

set(CMAKE_C_FLAGS "-Wno-implicit-function-declaration -Wno-incompatible-pointer-types -Wno-format -fno-builtin-memcpy")

add_message_files(
  FILES
  status_msg.msg
)

add_service_files(
  FILES
  SetAccelBias.srv
  SetGyroBias.srv
  SetHardIronValues.srv
  SetSoftIronMatrix.srv
  SetComplementaryFilter.srv
  SetFilterEuler.srv
  SetFilterHeading.srv
  SetAccelBiasModel.srv
  SetAccelAdaptiveVals.srv
  SetSensorVehicleFrameTrans.srv
  SetSensorVehicleFrameOffset.srv
  SetReferencePosition.srv
  SetConingScullingComp.srv
  SetEstimationControlFlags.srv
  SetDynamicsMode.srv
  SetZeroAngleUpdateThreshold.srv
  SetTareOrientation.srv
  SetAccelNoise.srv
  SetGyroNoise.srv
  SetMagNoise.srv
  SetGyroBiasModel.srv
  SetAccelAdaptiveVals.srv
  SetMagAdaptiveVals.srv
  SetMagDipAdaptiveVals.srv
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    cmake_modules
    tf2
    tf2_ros
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_runtime
)
###########
## Build ##
###########

# Directories containing MIP SDK header files
set(MIPLIBINC MIPSDK/C/Library/Include)
set(MIPLIBSRC MIPSDK/C/Library/Source)
set(MIPUTILINC MIPSDK/C/Utilities/Include)
set(MIPUTILSRC MIPSDK/C/Utilities/Source)

# The SDK comes with a skeleton for the user-defined functions
# It might be helpful to test compilation against these skeletons
# contains both .c and .h files
# set(MIPUSR "MIPSDK/C/Library/User\ Functions")
#include_directories(include ${MIPLIBINC} ${MIPUTILINC} ${MIPUSR})
include_directories(include/${PROJECT_NAME} ${MIPLIBINC} ${MIPUTILINC} ${catkin_INCLUDE_DIRS})

set(MIPINC
  ${MIPLIBINC}/mip.h
  ${MIPLIBINC}/mip_sdk_base.h
  ${MIPLIBINC}/mip_sdk_inteface.h
  ${MIPLIBINC}/mip_sdk_3dm.h
  ${MIPLIBINC}/mip_sdk_filter.h
  ${MIPLIBINC}/mip_sdk_system.h
  ${MIPLIBINC}/mip_sdk_ahrs.h
  ${MIPLIBINC}/mip_sdk_gps.h
  ${MIPLIBINC}/ring_buffer.h
#  ${MIPUSR}/mip_sdk_user_functions.h
  ${MIPUTILINC}/byteswap_utilities.h
)

set(MIPSRC
  ${MIPLIBSRC}/mip.c
  ${MIPLIBSRC}/mip_sdk_base.c
  ${MIPLIBSRC}/mip_sdk_inteface.c
  ${MIPLIBSRC}/mip_sdk_3dm.c
  ${MIPLIBSRC}/mip_sdk_filter.c
  ${MIPLIBSRC}/mip_sdk_system.c
  ${MIPLIBSRC}/mip_sdk_ahrs.c
  ${MIPLIBSRC}/mip_sdk_gps.c
  ${MIPLIBSRC}/ring_buffer.c
#  ${MIPUSR}/mip_sdk_user_functions.c
  ${MIPUTILSRC}/byteswap_utilities.c
)

# Libraries
add_library(microstrain_mips
  src/microstrain_3dm.cpp
  src/mip_sdk_user_functions.c
  #src/GX4-45_Test.cpp
  ${MIPSRC}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(microstrain_diagnostic_updater src/microstrain_diagnostic_updater.cpp)
add_dependencies(microstrain_diagnostic_updater ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Executables
add_executable(GX4-45_Test
  src/GX4-45_Test.c
  src/mip_sdk_user_functions.c
  ${MIPSRC}
)

add_executable(microstrain_mips_node
  src/microstrain_3dm_node.cpp
  src/microstrain_diagnostic_updater.cpp
)

add_dependencies(microstrain_mips_node
  microstrain_mips_gencpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
add_executable(set_accel_bias_client src/set_accel_bias_client.cpp)

add_executable(get_accel_bias_client src/get_accel_bias_client.cpp)

add_executable(set_gyro_bias_client src/set_gyro_bias_client.cpp)

add_executable(get_gyro_bias_client src/get_gyro_bias_client.cpp)

add_executable(set_hard_iron_values_client src/set_hard_iron_values_client.cpp)

add_executable(get_hard_iron_values_client src/get_hard_iron_values_client.cpp)

add_executable(device_report_client src/device_report_client.cpp)

add_executable(gyro_bias_capture_client src/gyro_bias_capture_client.cpp)

add_executable(set_soft_iron_matrix_client src/set_soft_iron_matrix_client.cpp)

add_executable(get_soft_iron_matrix_client src/get_soft_iron_matrix_client.cpp)

add_executable(set_complementary_filter_client src/set_complementary_filter_client.cpp)

add_executable(get_complementary_filter_client src/get_complementary_filter_client.cpp)

add_executable(set_filter_euler_client src/set_filter_euler_client.cpp)

add_executable(set_filter_heading_client src/set_filter_heading_client.cpp)

add_executable(set_accel_bias_model_client src/set_accel_bias_model_client.cpp)

add_executable(set_accel_adaptive_vals_client src/set_accel_adaptive_vals_client.cpp)

add_executable(set_sensor_vehicle_frame_trans_client src/set_sensor_vehicle_frame_trans_client.cpp)

add_executable(get_sensor_vehicle_frame_trans_client src/get_sensor_vehicle_frame_trans_client.cpp)

add_executable(set_sensor_vehicle_frame_offset_client src/set_sensor_vehicle_frame_offset_client.cpp)

add_executable(get_sensor_vehicle_frame_offset_client src/get_sensor_vehicle_frame_offset_client.cpp)

add_executable(set_reference_position_client src/set_reference_position_client.cpp)

add_executable(get_reference_position_client src/get_reference_position_client.cpp)

add_executable(set_coning_sculling_comp_client src/set_coning_sculling_comp_client.cpp)

add_executable(get_coning_sculling_comp_client src/get_coning_sculling_comp_client.cpp)

add_executable(set_estimation_control_flags_client src/set_estimation_control_flags_client.cpp)

add_executable(get_estimation_control_flags_client src/get_estimation_control_flags_client.cpp)

add_executable(set_dynamics_mode_client src/set_dynamics_mode_client.cpp)

add_executable(get_dynamics_mode_client src/get_dynamics_mode_client.cpp)

add_executable(get_basic_status_client src/get_basic_status_client.cpp)

add_executable(get_diagnostic_report_client src/get_diagnostic_report_client.cpp)

add_executable(set_zero_angle_update_threshold_client src/set_zero_angle_update_threshold_client.cpp)

add_executable(get_zero_angle_update_threshold_client src/get_zero_angle_update_threshold_client.cpp)

add_executable(set_tare_orientation_client src/set_tare_orientation_client.cpp)

add_executable(set_accel_noise_client src/set_accel_noise_client.cpp)

add_executable(get_accel_noise_client src/get_accel_noise_client.cpp)

add_executable(set_gyro_noise_client src/set_gyro_noise_client.cpp)

add_executable(get_gyro_noise_client src/get_gyro_noise_client.cpp)

add_executable(set_mag_noise_client src/set_mag_noise_client.cpp)

add_executable(get_mag_noise_client src/get_mag_noise_client.cpp)

add_executable(set_gyro_bias_model_client src/set_gyro_bias_model_client.cpp)

add_executable(get_gyro_bias_model_client src/get_gyro_bias_model_client.cpp)

add_executable(get_accel_adaptive_vals_client src/get_accel_adaptive_vals_client.cpp)

add_executable(set_mag_adaptive_vals_client src/set_mag_adaptive_vals_client.cpp)

add_executable(get_mag_adaptive_vals_client src/get_mag_adaptive_vals_client.cpp)

add_executable(set_mag_dip_adaptive_vals_client src/set_mag_dip_adaptive_vals_client.cpp)

add_executable(get_mag_dip_adaptive_vals_client src/get_mag_dip_adaptive_vals_client.cpp)

add_executable(get_accel_bias_model_client src/get_accel_bias_model_client.cpp)

add_dependencies(set_accel_bias_client microstrain_mips_gencpp)

add_dependencies(set_accel_bias_client microstrain_mips_gencpp)

add_dependencies(get_accel_bias_client microstrain_mips_gencpp)

add_dependencies(set_gyro_bias_client microstrain_mips_gencpp)

add_dependencies(get_gyro_bias_client microstrain_mips_gencpp)

add_dependencies(set_hard_iron_values_client microstrain_mips_gencpp)

add_dependencies(get_hard_iron_values_client microstrain_mips_gencpp)

add_dependencies(device_report_client microstrain_mips_gencpp)

add_dependencies(gyro_bias_capture_client microstrain_mips_gencpp)

add_dependencies(set_soft_iron_matrix_client microstrain_mips_gencpp)

add_dependencies(set_complementary_filter_client microstrain_mips_gencpp)

add_dependencies(get_complementary_filter_client microstrain_mips_gencpp)

add_dependencies(set_filter_euler_client microstrain_mips_gencpp)

add_dependencies(set_filter_heading_client microstrain_mips_gencpp)

add_dependencies(set_accel_bias_model_client microstrain_mips_gencpp)

add_dependencies(set_accel_adaptive_vals_client microstrain_mips_gencpp)

add_dependencies(set_sensor_vehicle_frame_trans_client microstrain_mips_gencpp)

add_dependencies(get_sensor_vehicle_frame_trans_client microstrain_mips_gencpp)

add_dependencies(set_sensor_vehicle_frame_offset_client microstrain_mips_gencpp)

add_dependencies(get_sensor_vehicle_frame_offset_client microstrain_mips_gencpp)

add_dependencies(set_reference_position_client microstrain_mips_gencpp)

add_dependencies(get_reference_position_client microstrain_mips_gencpp)

add_dependencies(set_coning_sculling_comp_client microstrain_mips_gencpp)

add_dependencies(get_coning_sculling_comp_client microstrain_mips_gencpp)

add_dependencies(set_estimation_control_flags_client microstrain_mips_gencpp)

add_dependencies(get_estimation_control_flags_client microstrain_mips_gencpp)

add_dependencies(set_dynamics_mode_client microstrain_mips_gencpp)

add_dependencies(get_dynamics_mode_client microstrain_mips_gencpp)

add_dependencies(get_basic_status_client microstrain_mips_gencpp)

add_dependencies(get_diagnostic_report_client microstrain_mips_gencpp)

add_dependencies(set_zero_angle_update_threshold_client microstrain_mips_gencpp)

add_dependencies(get_zero_angle_update_threshold_client microstrain_mips_gencpp)

add_dependencies(set_tare_orientation_client microstrain_mips_gencpp)

add_dependencies(set_accel_noise_client microstrain_mips_gencpp)

add_dependencies(get_accel_noise_client microstrain_mips_gencpp)

add_dependencies(set_gyro_noise_client microstrain_mips_gencpp)

add_dependencies(get_gyro_noise_client microstrain_mips_gencpp)

add_dependencies(set_mag_noise_client microstrain_mips_gencpp)

add_dependencies(get_mag_noise_client microstrain_mips_gencpp)

add_dependencies(set_gyro_bias_model_client microstrain_mips_gencpp)

add_dependencies(get_gyro_bias_model_client microstrain_mips_gencpp)

add_dependencies(get_accel_adaptive_vals_client microstrain_mips_gencpp)

add_dependencies(set_mag_adaptive_vals_client microstrain_mips_gencpp)

add_dependencies(get_mag_adaptive_vals_client microstrain_mips_gencpp)

add_dependencies(set_mag_dip_adaptive_vals_client microstrain_mips_gencpp)

add_dependencies(get_mag_dip_adaptive_vals_client microstrain_mips_gencpp)

add_dependencies(get_accel_bias_model_client microstrain_mips_gencpp)


# Linking
target_link_libraries(GX4-45_Test rt)

target_link_libraries(microstrain_mips
  ${catkin_LIBRARIES}
)
target_link_libraries(microstrain_mips_node
  microstrain_mips
  ${catkin_LIBRARIES}
)


target_link_libraries(set_accel_bias_client ${catkin_LIBRARIES})

target_link_libraries(get_accel_bias_client ${catkin_LIBRARIES})

target_link_libraries(set_gyro_bias_client ${catkin_LIBRARIES})

target_link_libraries(get_gyro_bias_client ${catkin_LIBRARIES})

target_link_libraries(set_hard_iron_values_client ${catkin_LIBRARIES})

target_link_libraries(get_hard_iron_values_client ${catkin_LIBRARIES})

target_link_libraries(device_report_client ${catkin_LIBRARIES})

target_link_libraries(gyro_bias_capture_client ${catkin_LIBRARIES})

target_link_libraries(set_soft_iron_matrix_client ${catkin_LIBRARIES})

target_link_libraries(get_soft_iron_matrix_client ${catkin_LIBRARIES})

target_link_libraries(set_complementary_filter_client ${catkin_LIBRARIES})

target_link_libraries(get_complementary_filter_client ${catkin_LIBRARIES})

target_link_libraries(set_filter_euler_client ${catkin_LIBRARIES})

target_link_libraries(set_filter_heading_client ${catkin_LIBRARIES})

target_link_libraries(set_accel_bias_model_client ${catkin_LIBRARIES})

target_link_libraries(set_accel_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(set_sensor_vehicle_frame_trans_client ${catkin_LIBRARIES})

target_link_libraries(get_sensor_vehicle_frame_trans_client ${catkin_LIBRARIES})

target_link_libraries(set_sensor_vehicle_frame_offset_client ${catkin_LIBRARIES})

target_link_libraries(get_sensor_vehicle_frame_offset_client ${catkin_LIBRARIES})

target_link_libraries(set_reference_position_client ${catkin_LIBRARIES})

target_link_libraries(get_reference_position_client ${catkin_LIBRARIES})

target_link_libraries(set_coning_sculling_comp_client ${catkin_LIBRARIES})

target_link_libraries(get_coning_sculling_comp_client ${catkin_LIBRARIES})

target_link_libraries(set_estimation_control_flags_client ${catkin_LIBRARIES})

target_link_libraries(get_estimation_control_flags_client ${catkin_LIBRARIES})

target_link_libraries(set_dynamics_mode_client ${catkin_LIBRARIES})

target_link_libraries(get_dynamics_mode_client ${catkin_LIBRARIES})

target_link_libraries(get_basic_status_client ${catkin_LIBRARIES})

target_link_libraries(get_diagnostic_report_client ${catkin_LIBRARIES})

target_link_libraries(set_zero_angle_update_threshold_client ${catkin_LIBRARIES})

target_link_libraries(get_zero_angle_update_threshold_client ${catkin_LIBRARIES})

target_link_libraries(set_tare_orientation_client ${catkin_LIBRARIES})

target_link_libraries(set_accel_noise_client ${catkin_LIBRARIES})

target_link_libraries(get_accel_noise_client ${catkin_LIBRARIES})

target_link_libraries(set_gyro_noise_client ${catkin_LIBRARIES})

target_link_libraries(get_gyro_noise_client ${catkin_LIBRARIES})

target_link_libraries(set_mag_noise_client ${catkin_LIBRARIES})

target_link_libraries(get_mag_noise_client ${catkin_LIBRARIES})

target_link_libraries(set_gyro_bias_model_client ${catkin_LIBRARIES})

target_link_libraries(get_gyro_bias_model_client ${catkin_LIBRARIES})

target_link_libraries(get_accel_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(set_mag_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(get_mag_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(set_mag_dip_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(get_mag_dip_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(get_accel_bias_model_client ${catkin_LIBRARIES})



#############
## Install ##
#############

install(TARGETS microstrain_mips microstrain_mips_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_microstrain_3dm_gx5.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## Run roslint on only the node files for now
# TODO: Add more files as needed
roslint_cpp(src/microstrain_3dm.cpp src/microstrain_3dm_node.cpp)
