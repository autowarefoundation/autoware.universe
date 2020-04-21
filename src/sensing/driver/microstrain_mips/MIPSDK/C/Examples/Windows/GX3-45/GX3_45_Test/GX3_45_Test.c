/////////////////////////////////////////////////////////////////////////////
//
// GX3_45_Test.c
//
// Test program for the GX3-45
//
// Notes:  This program runs through most of the sdk functions supported
//         by the GX3-45.  It does not permanently alter any of the device
//         settings.
//
//
// External dependencies:
//
//  mip_sdk.h
//
// Written By: 	Nathan Miller
//
//!@copyright 2014 Lord Microstrain Sensing Systems.
//
//!@section CHANGES
//!
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, LORD MICROSTRAIN SENSING SYSTEMS
//! SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES
//! WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR
//! THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION
//! WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <windows.h>
#include "byteswap_utilities.h"
#include "mip_sdk.h"

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS 1000  // milliseconds

////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////

// Help Functions
void print_header();
void print_command_line_usage();
void print_packet_stats();

// MIP Parser Packet Callback Functions
void filter_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type);
void ahrs_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type);
void gps_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type);

////////////////////////////////////////////////////////////////////////////////
//
// Globals
//
////////////////////////////////////////////////////////////////////////////////

// The primary device interface structure
mip_interface device_interface;

// Packet Counters (valid, timeout, and checksum errors)
u32 filter_valid_packet_count = 0;
u32 ahrs_valid_packet_count = 0;
u32 gps_valid_packet_count = 0;

u32 filter_timeout_packet_count = 0;
u32 ahrs_timeout_packet_count = 0;
u32 gps_timeout_packet_count = 0;

u32 filter_checksum_error_packet_count = 0;
u32 ahrs_checksum_error_packet_count = 0;
u32 gps_checksum_error_packet_count = 0;

// Example data field storage

// AHRS
mip_ahrs_scaled_gyro curr_ahrs_gyro;
mip_ahrs_scaled_accel curr_ahrs_accel;
mip_ahrs_scaled_mag curr_ahrs_mag;

// GPS
mip_gps_llh_pos curr_llh_pos;
mip_gps_ned_vel curr_ned_vel;
mip_gps_time curr_gps_time;

// FILTER
mip_filter_llh_pos curr_filter_pos;
mip_filter_ned_velocity curr_filter_vel;
mip_filter_attitude_euler_angles curr_filter_angles;

////////////////////////////////////////////////////////////////////////////////
//
// Main Function
//
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  u32 com_port, baudrate;
  base_device_info_field device_info;
  u8 temp_string[20] = {0};
  u32 bit_result;
  u8 enable = 1;
  u8 data_stream_format_descriptors[10];
  u16 data_stream_format_decimation[10];
  u8 data_stream_format_num_entries = 0;
  u8 readback_data_stream_format_descriptors[10] = {0};
  u16 readback_data_stream_format_decimation[10] = {0};
  u8 readback_data_stream_format_num_entries = 0;
  u16 base_rate = 0;
  u16 device_descriptors[128] = {0};
  u16 device_descriptors_size = 128 * 2;
  s16 i;
  u8 com_mode = 0;
  u8 readback_com_mode = 0;
  float angles[3] = {0};
  float readback_angles[3] = {0};
  float offset[3] = {0};
  float readback_offset[3] = {0};
  u8 dynamics_mode = 0;
  u8 readback_dynamics_mode = 0;
  u16 estimation_control = 0;
  u8 gps_source = 0;
  u8 heading_source = 0;
  u8 auto_init = 0;
  float noise[3] = {0};
  float readback_noise[3] = {0};
  float beta[3] = {0};
  float readback_beta[3] = {0};

  ///
  // Verify the command line arguments
  ///

  if (argc != NUM_COMMAND_LINE_ARGUMENTS) {
    print_command_line_usage();
    return -1;
  }

  // Convert the arguments
  com_port = atoi(argv[1]);
  baudrate = atoi(argv[2]);

  ///
  // Initialize the interface to the device
  ///

  if (mip_interface_init(com_port, baudrate, &device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
    return -1;

  ////////////////////////////////////////////////////////////////////////////////////
  //
  // Base Command Tests
  //
  ////////////////////////////////////////////////////////////////////////////////////

  ///
  // Put the GX3-45 into idle mode
  ///

  printf("\n\nPutting Device Into Idle Mode....\n");

  while (mip_base_cmd_idle(&device_interface) != MIP_INTERFACE_OK) {
  }

  ///
  // Try to ping the GX3-45
  ///

  printf("\n\nPinging Device....\n");

  while (mip_base_cmd_ping(&device_interface) != MIP_INTERFACE_OK) {
  }

  ///
  // Get the device information
  ///

  printf("\n\nGetting Device Information....\n");

  while (mip_base_cmd_get_device_info(&device_interface, &device_info) != MIP_INTERFACE_OK) {
  }

  printf("\n\nDevice Info:\n");
  printf("---------------------------------------------\n");

  memcpy(temp_string, device_info.model_name, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  printf("Model Name       => %s\n", temp_string);

  memcpy(temp_string, device_info.model_number, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  printf("Model Number     => %s\n", temp_string);

  memcpy(temp_string, device_info.serial_number, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  printf("Serial Number    => %s\n", temp_string);

  memcpy(temp_string, device_info.lotnumber, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  printf("Lot Number       => %s\n", temp_string);

  memcpy(temp_string, device_info.device_options, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  printf("Options          => %s\n", temp_string);

  printf("Firmware Version => %d.%d.%d\n\n", (device_info.firmware_version) / 1000,
         (device_info.firmware_version) % 1000 / 100, (device_info.firmware_version) % 100);

  ///
  // Get the supported descriptors
  ///

  printf("\n\nGetting Supported descriptors....\n");

  while (mip_base_cmd_get_device_supported_descriptors(&device_interface, (u8*)device_descriptors,
                                                       &device_descriptors_size) != MIP_INTERFACE_OK) {
  }

  printf("\n\nSupported descriptors:\n\n");

  for (i = 0; i < device_descriptors_size / 2; i++) {
    printf("Descriptor Set: %02x, Descriptor: %02x\n", device_descriptors[i] >> 8, device_descriptors[i] & 0xFF);
  }

  ///
  // Peform a built-in-test
  ///

  printf("\n\nRunning Built In Test....\n");

  while (mip_base_cmd_built_in_test(&device_interface, &bit_result) != MIP_INTERFACE_OK) {
  }

  printf("\nBIT Result (should be 0x00000000) => 0x%08x\n\n", bit_result);
  Sleep(2000);

  ////////////////////////////////////////////////////////////////////////////////////
  //
  // System Command Tests
  //
  ////////////////////////////////////////////////////////////////////////////////////

  ///
  // Set/Read the current com mode
  ///

  printf("\n\nCycle through available com modes....\n\n");

  // Loop through valid modes and set/read current com mode
  for (i = 3; i >= 1; i--) {
    // Set the com mode
    com_mode = i;

    while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK) {
    }

    // Reset the com_mode variable
    readback_com_mode = 0;

    // Read back the com mode
    while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &readback_com_mode) != MIP_INTERFACE_OK) {
    }

    if (com_mode == readback_com_mode) {
      printf("Com mode successfully set to %d\n", com_mode);
    } else {
      printf("ERROR: Failed to set com mode to %d, read %i!!!\n", com_mode, readback_com_mode);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////
  //
  // FILTER Command Tests
  //
  ////////////////////////////////////////////////////////////////////////////////////

  ///
  // Reset the filter
  ///

  printf("\n\nResetting the Filter....\n");

  while (mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK) {
  }

  ///
  // Initialize the filter with Euler Angles
  ///

  printf("\n\nInitializing the Filter with Euler angles....\n");

  angles[0] = angles[1] = angles[2] = 0;

  while (mip_filter_set_init_attitude(&device_interface, angles) != MIP_INTERFACE_OK) {
  }

  ///
  // Reset the filter
  ///

  printf("\n\nResetting the Filter....\n");

  while (mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK) {
  }

  ///
  // Initialize the filter with a heading
  ///

  printf("\n\nInitializing the Filter with a heading angle....\n");

  while (mip_filter_set_init_heading(&device_interface, angles[0]) != MIP_INTERFACE_OK) {
  }

  ///
  // Reset the filter
  ///

  printf("\n\nResetting the Filter....\n");

  while (mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK) {
  }

  ///
  // Initialize the filter from the AHRS with a declination angle
  ///

  printf("\n\nInitializing the Filter from the AHRS....\n");

  while (mip_filter_set_init_attitude_from_ahrs(&device_interface, angles[0]) != MIP_INTERFACE_OK) {
  }

  ///
  // Set/Read the Vehicle dynamics mode
  ///

  printf("\n\nCycle through available Vehicle Dynamics Modes....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 3; i >= 1; i--) {
    // Set the dynamics mode
    dynamics_mode = i;

    while (mip_filter_vehicle_dynamics_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &dynamics_mode) !=
           MIP_INTERFACE_OK) {
    }

    // Reset the readback_dynamics_mode variable
    readback_dynamics_mode = 0;

    // Read back the com mode
    while (mip_filter_vehicle_dynamics_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &readback_dynamics_mode) !=
           MIP_INTERFACE_OK) {
    }

    if (dynamics_mode == readback_dynamics_mode) {
      printf("Vehicle dynamics mode successfully set to %d\n", dynamics_mode);
    } else {
      printf("ERROR: Failed to set vehicle dynamics mode to %d!!!\n", dynamics_mode);
    }
  }

  printf("\n\nLoading the default vehicle dynamics mode....\n\n");

  while (mip_filter_vehicle_dynamics_mode(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) !=
         MIP_INTERFACE_OK) {
  }

  ///
  // Set/Read the Sensor to Vehicle Frame transformation
  ///

  printf("\n\nSetting the sensor to vehicle frame transformation....\n\n");

  angles[0] = 3.14 / 4.0;
  angles[1] = 3.14 / 8.0;
  angles[2] = 3.14 / 12.0;

  while (mip_filter_sensor2vehicle_tranformation(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, angles) !=
         MIP_INTERFACE_OK) {
  }

  // Read back the transformation
  while (mip_filter_sensor2vehicle_tranformation(&device_interface, MIP_FUNCTION_SELECTOR_READ, readback_angles) !=
         MIP_INTERFACE_OK) {
  }

  if ((abs(readback_angles[0] - angles[0]) < 0.001) && (abs(readback_angles[1] - angles[1]) < 0.001) &&
      (abs(readback_angles[2] - angles[2]) < 0.001)) {
    printf("Transformation successfully set.\n");
  } else {
    printf("ERROR: Failed to set transformation!!!\n");
    printf("Sent angles:     %f roll %f pitch %f yaw\n", angles[0], angles[1], angles[2]);
    printf("Returned angles: %f roll %f pitch %f yaw\n", readback_angles[0], readback_angles[1], readback_angles[2]);
  }

  printf("\n\nLoading the default sensor to vehicle transformation....\n\n");

  while (mip_filter_sensor2vehicle_tranformation(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) !=
         MIP_INTERFACE_OK) {
  }

  ///
  // Set/Read the Sensor to Vehicle Frame offset
  ///

  printf("\n\nSetting the sensor to vehicle frame offset....\n\n");

  offset[0] = 1.0;
  offset[1] = 2.0;
  offset[2] = 3.0;

  while (mip_filter_sensor2vehicle_offset(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, offset) != MIP_INTERFACE_OK) {
  }

  // Read back the transformation
  while (mip_filter_sensor2vehicle_offset(&device_interface, MIP_FUNCTION_SELECTOR_READ, readback_offset) !=
         MIP_INTERFACE_OK) {
  }

  if ((abs(readback_offset[0] - offset[0]) < 0.001) && (abs(readback_offset[1] - offset[1]) < 0.001) &&
      (abs(readback_offset[2] - offset[2]) < 0.001)) {
    printf("Offset successfully set.\n");
  } else {
    printf("ERROR: Failed to set offset!!!\n");
    printf("Sent offset:     %f X %f Y %f Z\n", offset[0], offset[1], offset[2]);
    printf("Returned offset: %f X %f Y %f Z\n", readback_offset[0], readback_offset[1], readback_offset[2]);
  }

  printf("\n\nLoading the default sensor to vehicle offset....\n\n");

  while (mip_filter_sensor2vehicle_offset(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) !=
         MIP_INTERFACE_OK) {
  }

  ///
  // Set/Read the GPS antenna offset
  ///

  printf("\n\nSetting the GPS antenna offset....\n\n");

  offset[0] = 1.0;
  offset[1] = 2.0;
  offset[2] = 3.0;

  while (mip_filter_antenna_offset(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, offset) != MIP_INTERFACE_OK) {
  }

  // Read back the transformation
  while (mip_filter_antenna_offset(&device_interface, MIP_FUNCTION_SELECTOR_READ, readback_offset) !=
         MIP_INTERFACE_OK) {
  }

  if ((abs(readback_offset[0] - offset[0]) < 0.001) && (abs(readback_offset[1] - offset[1]) < 0.001) &&
      (abs(readback_offset[2] - offset[2]) < 0.001)) {
    printf("Offset successfully set.\n");
  } else {
    printf("ERROR: Failed to set offset!!!\n");
    printf("Sent offset:     %f X %f Y %f Z\n", offset[0], offset[1], offset[2]);
    printf("Returned offset: %f X %f Y %f Z\n", readback_offset[0], readback_offset[1], readback_offset[2]);
  }

  printf("\n\nLoading the default antenna offset....\n\n");

  while (mip_filter_antenna_offset(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
  }

  ///
  // Set/Read the estimation control flags
  ///

  printf("\n\nCycle through available Estimation Control Values....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 0; i <= 1; i++) {
    // Set the estimation control
    estimation_control = i;

    while (mip_filter_estimation_control(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &estimation_control) !=
           MIP_INTERFACE_OK) {
    }

    // Read back the estimation control
    while (mip_filter_estimation_control(&device_interface, MIP_FUNCTION_SELECTOR_READ, &estimation_control) !=
           MIP_INTERFACE_OK) {
    }

    if (estimation_control == i) {
      printf("Estimation control successfully set to %d\n", estimation_control);
    } else {
      printf("ERROR: Failed to set estimation control to %d!!!\n", estimation_control);
    }
  }

  printf("\n\nLoading the default estimation control....\n\n");

  while (mip_filter_estimation_control(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) !=
         MIP_INTERFACE_OK) {
  }

  ///
  // Set/Read the gps source
  ///

  printf("\n\nCycle through available GPS sources....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 2; i >= 1; i--) {
    // Set the GPS source
    gps_source = i;

    while (mip_filter_gps_source(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &gps_source) != MIP_INTERFACE_OK) {
    }

    // Read back the GPS source
    while (mip_filter_gps_source(&device_interface, MIP_FUNCTION_SELECTOR_READ, &gps_source) != MIP_INTERFACE_OK) {
    }

    if (gps_source == i) {
      printf("GPS source successfully set to %d\n", gps_source);
    } else {
      printf("ERROR: Failed to set GPS source to %d!!!\n", gps_source);
    }
  }

  printf("\n\nLoading the default gps source....\n\n");

  while (mip_filter_gps_source(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
  }

  ///
  // Cycle through the available heading sources
  ///

  printf("\n\nCycle through available Heading sources....\n\n");

  // Loop through valid heading sources and set/read
  for (i = 3; i >= 0; i--) {
    // Set the heading source
    heading_source = i;

    while (mip_filter_heading_source(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &heading_source) !=
           MIP_INTERFACE_OK) {
    }

    // Read back the heading source
    while (mip_filter_heading_source(&device_interface, MIP_FUNCTION_SELECTOR_READ, &heading_source) !=
           MIP_INTERFACE_OK) {
    }

    if (heading_source == i) {
      printf("Heading source successfully set to %d\n", heading_source);
    } else {
      printf("ERROR: Failed to set heading source to %d!!!\n", heading_source);
    }
  }

  printf("\n\nLoading the default heading source....\n\n");

  while (mip_filter_heading_source(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
  }

  ///
  // Cycle through the available auto-init values
  ///

  printf("\n\nCycle through available auto-init values....\n\n");

  // Loop through valid heading sources and set/read
  for (i = 1; i >= 0; i--) {
    // Set the auto-init value
    auto_init = i;

    while (mip_filter_auto_initialization(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &auto_init) !=
           MIP_INTERFACE_OK) {
    }

    // Read back the auto-init value
    while (mip_filter_auto_initialization(&device_interface, MIP_FUNCTION_SELECTOR_READ, &auto_init) !=
           MIP_INTERFACE_OK) {
    }

    if (auto_init == i) {
      printf("Auto-init successfully set to %d\n", auto_init);
    } else {
      printf("ERROR: Failed to set auto-init to %d!!!\n", auto_init);
    }
  }

  printf("\n\nLoading the default auto-init value....\n\n");

  while (mip_filter_auto_initialization(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) !=
         MIP_INTERFACE_OK) {
  }

  ///
  // Set/Read the accel noise values
  ///

  printf("\n\nSetting the accel noise values....\n\n");

  noise[0] = 0.1;
  noise[1] = 0.2;
  noise[2] = 0.3;

  while (mip_filter_accel_noise(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, noise) != MIP_INTERFACE_OK) {
  }

  // Read back the accel noise values
  while (mip_filter_accel_noise(&device_interface, MIP_FUNCTION_SELECTOR_READ, readback_noise) != MIP_INTERFACE_OK) {
  }

  if ((abs(readback_noise[0] - noise[0]) < 0.001) && (abs(readback_noise[1] - noise[1]) < 0.001) &&
      (abs(readback_noise[2] - noise[2]) < 0.001)) {
    printf("Accel noise values successfully set.\n");
  } else {
    printf("ERROR: Failed to set accel noise values!!!\n");
    printf("Sent values:     %f X %f Y %f Z\n", noise[0], noise[1], noise[2]);
    printf("Returned values: %f X %f Y %f Z\n", readback_noise[0], readback_noise[1], readback_noise[2]);
  }

  printf("\n\nLoading the default accel noise values....\n\n");

  while (mip_filter_accel_noise(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
  }

  ///
  // Set/Read the gyro noise values
  ///

  printf("\n\nSetting the gyro noise values....\n\n");

  noise[0] = 0.1;
  noise[1] = 0.2;
  noise[2] = 0.3;

  while (mip_filter_gyro_noise(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, noise) != MIP_INTERFACE_OK) {
  }

  // Read back the gyro noise values
  while (mip_filter_gyro_noise(&device_interface, MIP_FUNCTION_SELECTOR_READ, readback_noise) != MIP_INTERFACE_OK) {
  }

  if ((abs(readback_noise[0] - noise[0]) < 0.001) && (abs(readback_noise[1] - noise[1]) < 0.001) &&
      (abs(readback_noise[2] - noise[2]) < 0.001)) {
    printf("Gyro noise values successfully set.\n");
  } else {
    printf("ERROR: Failed to set gyro noise values!!!\n");
    printf("Sent values:     %f X %f Y %f Z\n", noise[0], noise[1], noise[2]);
    printf("Returned values: %f X %f Y %f Z\n", readback_noise[0], readback_noise[1], readback_noise[2]);
  }

  printf("\n\nLoading the default gyro noise values....\n\n");

  while (mip_filter_gyro_noise(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
  }

  ///
  // Set/Read the gyro bias model values
  ///

  printf("\n\nSetting the gyro bias model values....\n\n");

  noise[0] = 0.1;
  noise[1] = 0.2;
  noise[2] = 0.3;

  beta[0] = 0.1;
  beta[1] = 0.2;
  beta[2] = 0.3;

  while (mip_filter_gyro_bias_model(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, beta, noise) != MIP_INTERFACE_OK) {
  }

  // Read back the gyro bias model values
  while (mip_filter_gyro_bias_model(&device_interface, MIP_FUNCTION_SELECTOR_READ, readback_beta, readback_noise) !=
         MIP_INTERFACE_OK) {
  }

  if ((abs(readback_noise[0] - noise[0]) < 0.001) && (abs(readback_noise[1] - noise[1]) < 0.001) &&
      (abs(readback_noise[2] - noise[2]) < 0.001) && (abs(readback_beta[0] - beta[0]) < 0.001) &&
      (abs(readback_beta[1] - beta[1]) < 0.001) && (abs(readback_beta[2] - beta[2]) < 0.001)) {
    printf("Gyro bias model values successfully set.\n");
  } else {
    printf("ERROR: Failed to set gyro bias model values!!!\n");
    printf("Sent values:     Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", beta[0], beta[1], beta[2], noise[0],
           noise[1], noise[2]);
    printf("Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", readback_beta[0], readback_beta[1],
           readback_beta[2], readback_noise[0], readback_noise[1], readback_noise[2]);
  }

  printf("\n\nLoading the default gyro bias model values....\n\n");

  while (mip_filter_gyro_bias_model(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL, NULL) !=
         MIP_INTERFACE_OK) {
  }

  ////////////////////////////////////////////////////////////////////////////////////
  //
  // 3DM Command Tests
  //
  ////////////////////////////////////////////////////////////////////////////////////

  ///
  // Get AHRS Base Rate
  ///

  printf("\n\nGetting the AHRS datastream base rate....\n");

  while (mip_3dm_cmd_get_ahrs_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK) {
  }

  printf("\nAHRS Base Rate => %d Hz\n\n", base_rate);

  ///
  // Get GPS Base Rate
  ///

  printf("\n\nGetting the GPS datastream base rate....\n");

  while (mip_3dm_cmd_get_gps_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK) {
  }

  printf("\nGPS Base Rate => %d Hz\n\n", base_rate);

  ///
  // Get FILTER Base Rate
  ///

  printf("\n\nGetting the FILTER datastream base rate....\n");

  while (mip_3dm_cmd_get_filter_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK) {
  }

  printf("\nFILTER Base Rate => %d Hz\n\n", base_rate);

  ///
  // Setup the FILTER datastream format
  ///

  printf("\n\nSetting the FILTER datastream format....\n");

  data_stream_format_descriptors[0] = MIP_FILTER_DATA_LLH_POS;
  data_stream_format_descriptors[1] = MIP_FILTER_DATA_NED_VEL;
  data_stream_format_descriptors[2] = MIP_FILTER_DATA_ATT_EULER_ANGLES;

  data_stream_format_decimation[0] = 0x01;
  data_stream_format_decimation[1] = 0x01;
  data_stream_format_decimation[2] = 0x01;

  data_stream_format_num_entries = 3;

  while (mip_3dm_cmd_filter_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                           &data_stream_format_num_entries, data_stream_format_descriptors,
                                           data_stream_format_decimation) != MIP_INTERFACE_OK) {
  }

  ///
  // Setup the AHRS datastream format
  ///

  printf("\n\nSetting the AHRS datastream format....\n");

  data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
  data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
  data_stream_format_descriptors[2] = MIP_AHRS_DATA_MAG_SCALED;

  data_stream_format_decimation[0] = 0x01;
  data_stream_format_decimation[1] = 0x01;
  data_stream_format_decimation[2] = 0x01;

  data_stream_format_num_entries = 3;

  while (mip_3dm_cmd_ahrs_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                         &data_stream_format_num_entries, data_stream_format_descriptors,
                                         data_stream_format_decimation) != MIP_INTERFACE_OK) {
  }

  ///
  // Setup the GPS datastream format
  ///

  printf("\n\nSetting the GPS datastream format....\n");

  data_stream_format_descriptors[0] = MIP_GPS_DATA_LLH_POS;
  data_stream_format_descriptors[1] = MIP_GPS_DATA_NED_VELOCITY;
  data_stream_format_descriptors[2] = MIP_GPS_DATA_GPS_TIME;

  data_stream_format_decimation[0] = 0x01;
  data_stream_format_decimation[1] = 0x01;
  data_stream_format_decimation[2] = 0x01;

  while (mip_3dm_cmd_gps_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,
                                        data_stream_format_descriptors,
                                        data_stream_format_decimation) != MIP_INTERFACE_OK) {
  }

  ///
  // Enable the FILTER datastream
  ///

  printf("\n\nEnable the FILTER datastream....\n");

  while (mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM,
                                            &enable) != MIP_INTERFACE_OK) {
  }

  ///
  // Enable the AHRS datastream
  ///

  printf("\n\nEnable the AHRS datastream....\n");

  while (mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM,
                                            &enable) != MIP_INTERFACE_OK) {
  }

  ///
  // Enable the GPS datastream
  ///

  printf("\n\nEnable the GPS datastream....\n");

  while (mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_GPS_DATASTREAM,
                                            &enable) != MIP_INTERFACE_OK) {
  }

  printf("\n\n");

  ///
  // Setup the GX3-45 dataset callbacks (FILTER, AHRS, GPS)
  ///

  if (mip_interface_add_descriptor_set_callback(&device_interface, MIP_FILTER_DATA_SET, NULL,
                                                &filter_packet_callback) != MIP_INTERFACE_OK)
    return -1;

  if (mip_interface_add_descriptor_set_callback(&device_interface, MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback) !=
      MIP_INTERFACE_OK)
    return -1;

  if (mip_interface_add_descriptor_set_callback(&device_interface, MIP_GPS_DATA_SET, NULL, &gps_packet_callback) !=
      MIP_INTERFACE_OK)
    return -1;

  ///
  // Wait for packets to arrive
  ///

  while (1) {
    // Update the parser (this function reads the port and parses the bytes
    mip_interface_update(&device_interface);

    // Be nice to other programs
    Sleep(1);
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// Filter Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void filter_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  mip_field_header* field_header;
  u8* field_data;
  u16 field_offset = 0;

  // The packet callback can have several types, process them all
  switch (callback_type) {
      ///
      // Handle valid packets
      ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
      filter_valid_packet_count++;

      ///
      // Loop through all of the data fields
      ///

      while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK) {
        ///
        // Decode the field
        ///

        switch (field_header->descriptor) {
            ///
            // Estimated LLH Position
            ///

          case MIP_FILTER_DATA_LLH_POS: {
            memcpy(&curr_filter_pos, field_data, sizeof(mip_filter_llh_pos));

            // For little-endian targets, byteswap the data field
            mip_filter_llh_pos_byteswap(&curr_filter_pos);

          } break;

            ///
            // Estimated NED Velocity
            ///

          case MIP_FILTER_DATA_NED_VEL: {
            memcpy(&curr_filter_vel, field_data, sizeof(mip_filter_ned_velocity));

            // For little-endian targets, byteswap the data field
            mip_filter_ned_velocity_byteswap(&curr_filter_vel);

          } break;

            ///
            // Estimated Attitude, Euler Angles
            ///

          case MIP_FILTER_DATA_ATT_EULER_ANGLES: {
            memcpy(&curr_filter_angles, field_data, sizeof(mip_filter_attitude_euler_angles));

            // For little-endian targets, byteswap the data field
            mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles);

          } break;

          default:
            break;
        }
      }
    } break;

      ///
      // Handle checksum error packets
      ///

    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR: {
      filter_checksum_error_packet_count++;
    } break;

      ///
      // Handle timeout packets
      ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT: {
      filter_timeout_packet_count++;
    } break;
    default:
      break;
  }

  print_packet_stats();
}

////////////////////////////////////////////////////////////////////////////////
//
// AHRS Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void ahrs_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  mip_field_header* field_header;
  u8* field_data;
  u16 field_offset = 0;

  // The packet callback can have several types, process them all
  switch (callback_type) {
      ///
      // Handle valid packets
      ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
      ahrs_valid_packet_count++;

      ///
      // Loop through all of the data fields
      ///

      while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK) {
        ///
        // Decode the field
        ///

        switch (field_header->descriptor) {
            ///
            // Scaled Accelerometer
            ///

          case MIP_AHRS_DATA_ACCEL_SCALED: {
            memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));

            // For little-endian targets, byteswap the data field
            mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);

          } break;

            ///
            // Scaled Gyro
            ///

          case MIP_AHRS_DATA_GYRO_SCALED: {
            memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));

            // For little-endian targets, byteswap the data field
            mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);

          } break;

            ///
            // Scaled Magnetometer
            ///

          case MIP_AHRS_DATA_MAG_SCALED: {
            memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));

            // For little-endian targets, byteswap the data field
            mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);

          } break;

          default:
            break;
        }
      }
    } break;

      ///
      // Handle checksum error packets
      ///

    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR: {
      ahrs_checksum_error_packet_count++;
    } break;

      ///
      // Handle timeout packets
      ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT: {
      ahrs_timeout_packet_count++;
    } break;
    default:
      break;
  }

  print_packet_stats();
}

////////////////////////////////////////////////////////////////////////////////
//
// GPS Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void gps_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  mip_field_header* field_header;
  u8* field_data;
  u16 field_offset = 0;

  // The packet callback can have several types, process them all
  switch (callback_type) {
      ///
      // Handle valid packets
      ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
      gps_valid_packet_count++;

      ///
      // Loop through all of the data fields
      ///

      while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK) {
        ///
        // Decode the field
        ///

        switch (field_header->descriptor) {
            ///
            // LLH Position
            ///

          case MIP_GPS_DATA_LLH_POS: {
            memcpy(&curr_llh_pos, field_data, sizeof(mip_gps_llh_pos));

            // For little-endian targets, byteswap the data field
            mip_gps_llh_pos_byteswap(&curr_llh_pos);

          } break;

            ///
            // NED Velocity
            ///

          case MIP_GPS_DATA_NED_VELOCITY: {
            memcpy(&curr_ned_vel, field_data, sizeof(mip_gps_ned_vel));

            // For little-endian targets, byteswap the data field
            mip_gps_ned_vel_byteswap(&curr_ned_vel);

          } break;

            ///
            // GPS Time
            ///

          case MIP_GPS_DATA_GPS_TIME: {
            memcpy(&curr_gps_time, field_data, sizeof(mip_gps_time));

            // For little-endian targets, byteswap the data field
            mip_gps_time_byteswap(&curr_gps_time);

          } break;

          default:
            break;
        }
      }
    } break;

      ///
      // Handle checksum error packets
      ///

    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR: {
      gps_checksum_error_packet_count++;
    } break;

      ///
      // Handle timeout packets
      ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT: {
      gps_timeout_packet_count++;
    } break;
    default:
      break;
  }

  print_packet_stats();
}

////////////////////////////////////////////////////////////////////////////////
//
// Print Command-Line Help
//
////////////////////////////////////////////////////////////////////////////////

void print_command_line_usage() {
  printf("\n\n");
  printf("Usage:\n");
  printf("-----------------------------------------------------------------------\n\n");

  printf("   GX3_45_Test [com_port_num] [baudrate]\n");
  printf("\n\n");
  printf("   Example: \"GX3_45_Test 1 115200\", Opens a connection to the \n");
  printf("             GX3-45 on COM1, with a baudrate of 115200.\n");
  printf("\n\n");
  printf("   [ ] - required command input.\n");
  printf("\n-----------------------------------------------------------------------\n");
  printf("\n\n");
}

////////////////////////////////////////////////////////////////////////////////
//
// Print Header
//
////////////////////////////////////////////////////////////////////////////////

void print_header() {
  printf("\n");
  printf("GX3-45 Test Program\n");
  printf("Copyright 2011. Microstrain, Inc.\n\n");
}

////////////////////////////////////////////////////////////////////////////////
//
// Print Packet Statistics
//
////////////////////////////////////////////////////////////////////////////////

void print_packet_stats() {
  printf("\r%u FILTER (%u errors)    %u AHRS (%u errors)    %u GPS (%u errors) Packets", filter_valid_packet_count,
         filter_timeout_packet_count + filter_checksum_error_packet_count, ahrs_valid_packet_count,
         ahrs_timeout_packet_count + ahrs_checksum_error_packet_count, gps_valid_packet_count,
         gps_timeout_packet_count + gps_checksum_error_packet_count);
}
