/////////////////////////////////////////////////////////////////////////////
//
// GX3_35_Test.c
//
// Test program for the GX3-35
//
// Notes:  This program runs through most of the sdk functions supported
//         by the GX3-35.  It does not permanently alter any of the device
//         settings.
//
//         THIS SHOULD ONLY BE RUN WITH THE RS-232 INTERFACE!!!
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
#include "mip_gx3_35.h"
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
u32 ahrs_valid_packet_count = 0;
u32 gps_valid_packet_count = 0;

u32 ahrs_timeout_packet_count = 0;
u32 gps_timeout_packet_count = 0;

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
  u8 data_stream_format_descriptors[10] = {0};
  u16 data_stream_format_decimation[10] = {0};
  u8 data_stream_format_num_entries = 0;
  u8 readback_data_stream_format_descriptors[10] = {0};
  u16 readback_data_stream_format_decimation[10] = {0};
  u8 readback_data_stream_format_num_entries = 0;
  u16 base_rate = 0;
  u16 device_descriptors[128] = {0};
  u16 device_descriptors_size = 128 * 2;
  s16 i;
  u8 com_mode = 0;
  u8 datastream_format = 0;
  u8 gps_dynamics_mode = 0;
  u8 ahrs_time_selector = 0;
  u8 readback_ahrs_time_selector = 0;
  u32 ahrs_time_value = 0;
  mip_ahrs_signal_settings signal_conditioning_settings = {0}, readback_signal_conditioning_settings = {0};
  u8 power_state = 0, readback_power_state = 0;
  gx3_35_device_status_field gx3_35_status;
  u16 gx3_35_status_size = sizeof(gx3_35_device_status_field);

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
  // Put the GX3-35 into idle mode
  ///

  printf("\n\nPutting Device Into Idle Mode....\n");

  while (mip_base_cmd_idle(&device_interface) != MIP_INTERFACE_OK) {
  }

  ///
  // Try to ping the GX3-35
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

  // Loop through valid modes and set/read current come mode
  for (i = 3; i >= 1; i--) {
    // Set the com mode
    com_mode = i;

    while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK) {
    }

    // Reset the com_mode variable
    com_mode = 0;

    // Read back the com mode
    while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK) {
    }

    if (com_mode == i) {
      printf("Com mode successfully set to %d\n", com_mode);
    } else {
      printf("ERROR: Failed to set com mode to %d!!!\n", i);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////
  //
  // 3DM Command Tests
  //
  ////////////////////////////////////////////////////////////////////////////////////

  ///
  // Set/Read the AHRS power state
  ///

  printf("\n\nTurn off/on AHRS and verify state....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 0; i <= 1; i++) {
    // Set the power state (off, then on)
    if (i == 0)
      power_state = MIP_3DM_POWER_STATE_OFF;
    else if (i == 1)
      power_state = MIP_3DM_POWER_STATE_ON;

    while (mip_3dm_cmd_power_state(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_POWER_STATE_DEVICE_AHRS,
                                   &power_state) != MIP_INTERFACE_OK) {
    }

    // Reset the readback_power_state variable
    readback_power_state = 0;

    // Read back the com mode
    while (mip_3dm_cmd_power_state(&device_interface, MIP_FUNCTION_SELECTOR_READ, MIP_3DM_POWER_STATE_DEVICE_AHRS,
                                   &readback_power_state) != MIP_INTERFACE_OK) {
    }

    if (power_state == readback_power_state) {
      printf("Power state successfully set to %d\n", power_state);
    } else {
      printf("ERROR: Failed to set power state to %d!!!\n", power_state);
    }
  }

  ///
  // Set/Read the GPS power state
  ///

  printf("\n\nTurn off/on GPS and verify state....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 0; i <= 1; i++) {
    // Set the power state (off, then on)
    if (i == 0)
      power_state = MIP_3DM_POWER_STATE_OFF;
    else if (i == 1)
      power_state = MIP_3DM_POWER_STATE_ON;

    while (mip_3dm_cmd_power_state(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_POWER_STATE_DEVICE_GPS,
                                   &power_state) != MIP_INTERFACE_OK) {
    }

    // Reset the readback_power_state variable
    readback_power_state = 0;

    // Read back the com mode
    while (mip_3dm_cmd_power_state(&device_interface, MIP_FUNCTION_SELECTOR_READ, MIP_3DM_POWER_STATE_DEVICE_GPS,
                                   &readback_power_state) != MIP_INTERFACE_OK) {
    }

    if (power_state == readback_power_state) {
      printf("Power state successfully set to %d\n", power_state);
    } else {
      printf("ERROR: Failed to set power state to %d!!!\n", power_state);
    }
  }

  ///
  // Get GX3-35 diagnostic status packet
  ///

  printf("\n\nGetting the GX3-35 diagnostic status packet....\n");

  while (mip_3dm_cmd_device_status(&device_interface, GX3_35_MODEL_NUMBER, GX3_35_DIAGNOSTICS_STATUS_SEL,
                                   &gx3_35_status, &gx3_35_status_size) != MIP_INTERFACE_OK) {
  }

  ///
  // Get AHRS Base Rate
  ///

  printf("\n\nGetting the AHRS datastream base rate....\n");

  while (mip_3dm_cmd_get_ahrs_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK) {
  }

  printf("\nAHRS Base Rate => %d Hz\n", base_rate);

  ///
  // Get GPS Base Rate
  ///

  printf("\n\nGetting the GPS datastream base rate....\n");

  while (mip_3dm_cmd_get_gps_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK) {
  }

  printf("\nGPS Base Rate => %d Hz\n", base_rate);

  ///
  // Poll for an AHRS packet
  ///

  printf("\n\nPolling for an AHRS packet....\n\n");

  data_stream_format_descriptors[0] = 0x04;
  data_stream_format_descriptors[1] = 0x05;
  data_stream_format_num_entries = 2;

  // Poll for the AHRS packet
  while (mip_3dm_cmd_poll_ahrs(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries,
                               data_stream_format_descriptors) != MIP_INTERFACE_OK) {
  }

  printf("Got an ACK.\n");

  ///
  // Poll for a GPS packet
  ///

  printf("\n\nPolling for a GPS packet....\n\n");

  data_stream_format_descriptors[0] = 0x03;
  data_stream_format_descriptors[1] = 0x05;
  data_stream_format_descriptors[2] = 0x08;
  data_stream_format_num_entries = 3;

  // Poll for the GPS packet
  while (mip_3dm_cmd_poll_gps(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries,
                              data_stream_format_descriptors) != MIP_INTERFACE_OK) {
  }

  printf("Got an ACK.\n");

  ///
  // Setup the AHRS message format and verify via read-back
  ///

  printf("\n\nSetting the AHRS datastream format....\n\n");

  data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
  data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
  data_stream_format_descriptors[2] = MIP_AHRS_DATA_MAG_SCALED;

  data_stream_format_decimation[0] = 0x0A;
  data_stream_format_decimation[1] = 0x0A;
  data_stream_format_decimation[2] = 0x0A;

  data_stream_format_num_entries = 3;

  // Set the message format
  while (mip_3dm_cmd_ahrs_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                         &data_stream_format_num_entries, data_stream_format_descriptors,
                                         data_stream_format_decimation) != MIP_INTERFACE_OK) {
  }

  // On entry to the read, we set the maximum number of entries we can accept
  readback_data_stream_format_num_entries = 10;

  // Read the message format
  while (mip_3dm_cmd_ahrs_message_format(
             &device_interface, MIP_FUNCTION_SELECTOR_READ, &readback_data_stream_format_num_entries,
             readback_data_stream_format_descriptors, readback_data_stream_format_decimation) != MIP_INTERFACE_OK) {
  }

  // Compare the message formats
  if (data_stream_format_num_entries == readback_data_stream_format_num_entries) {
    printf("Number of fields match: %d fields\n", data_stream_format_num_entries);
  } else {
    printf("ERROR: Number of fields mismatch: %d fields written, %d fields read!!!\n", data_stream_format_num_entries,
           readback_data_stream_format_num_entries);
  }

  for (i = 0; i < data_stream_format_num_entries; i++) {
    if ((data_stream_format_descriptors[i] == readback_data_stream_format_descriptors[i]) &&
        (data_stream_format_decimation[i] == readback_data_stream_format_decimation[i])) {
      printf("Descriptor information for field %d matches\n", i);
    } else {
      printf("ERROR: Descriptor information for field %d mismatch!!!! Written: %d %d, Read: %d %d\n", i,
             data_stream_format_descriptors[i], data_stream_format_decimation[i],
             readback_data_stream_format_descriptors[i], readback_data_stream_format_decimation[i]);
    }
  }

  ///
  // Setup the GPS message format and verify via read-back
  ///

  printf("\n\nSetting the GPS datastream format....\n\n");

  data_stream_format_descriptors[0] = MIP_GPS_DATA_LLH_POS;
  data_stream_format_descriptors[1] = MIP_GPS_DATA_NED_VELOCITY;
  data_stream_format_descriptors[2] = MIP_GPS_DATA_GPS_TIME;

  data_stream_format_decimation[0] = 0x01;
  data_stream_format_decimation[1] = 0x01;
  data_stream_format_decimation[2] = 0x01;

  data_stream_format_num_entries = 3;

  while (mip_3dm_cmd_gps_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,
                                        data_stream_format_descriptors,
                                        data_stream_format_decimation) != MIP_INTERFACE_OK) {
  }

  // On entry to the read, we set the maximum number of entries we can accept
  readback_data_stream_format_num_entries = 10;

  // Read the message format
  while (mip_3dm_cmd_gps_message_format(
             &device_interface, MIP_FUNCTION_SELECTOR_READ, &readback_data_stream_format_num_entries,
             readback_data_stream_format_descriptors, readback_data_stream_format_decimation) != MIP_INTERFACE_OK) {
  }

  // Compare the message formats
  if (data_stream_format_num_entries == readback_data_stream_format_num_entries) {
    printf("Number of fields match: %d fields\n", data_stream_format_num_entries);
  } else {
    printf("ERROR: Number of fields mismatch: %d fields written, %d fields read!!!\n", data_stream_format_num_entries,
           readback_data_stream_format_num_entries);
  }

  for (i = 0; i < data_stream_format_num_entries; i++) {
    if ((data_stream_format_descriptors[i] == readback_data_stream_format_descriptors[i]) &&
        (data_stream_format_decimation[i] == readback_data_stream_format_decimation[i])) {
      printf("Descriptor information for field %d matches\n", i);
    } else {
      printf("ERROR: Descriptor information for field %d mismatch!!!! Written: %d %d, Read: %d %d\n", i,
             data_stream_format_descriptors[i], data_stream_format_decimation[i],
             readback_data_stream_format_descriptors[i], readback_data_stream_format_decimation[i]);
    }
  }

  ///
  // Set/Read the AHRS datastream format
  ///

  printf("\n\nCycle through available AHRS datastream formats....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 2; i >= 1; i--) {
    // Set the datastream format
    datastream_format = i;

    while (mip_3dm_cmd_datastream_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM,
                                         &datastream_format) != MIP_INTERFACE_OK) {
    }

    // Reset the datastream_format variable
    datastream_format = 0;

    // Read back the com mode
    while (mip_3dm_cmd_datastream_format(&device_interface, MIP_FUNCTION_SELECTOR_READ, MIP_3DM_AHRS_DATASTREAM,
                                         &datastream_format) != MIP_INTERFACE_OK) {
    }

    if (datastream_format == i) {
      printf("Datastream format successfully set to %d\n", datastream_format);
    } else {
      printf("ERROR: Failed to set datastream format to %d!!!\n", datastream_format);
    }
  }

  ///
  // Set/Read the GPS datastream format
  ///

  printf("\n\nCycle through available GPS datastream formats....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 2; i >= 1; i--) {
    // Set the datastream format
    datastream_format = i;

    while (mip_3dm_cmd_datastream_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_GPS_DATASTREAM,
                                         &datastream_format) != MIP_INTERFACE_OK) {
    }

    // Reset the datastream_format variable
    datastream_format = 0;

    // Read back the com mode
    while (mip_3dm_cmd_datastream_format(&device_interface, MIP_FUNCTION_SELECTOR_READ, MIP_3DM_GPS_DATASTREAM,
                                         &datastream_format) != MIP_INTERFACE_OK) {
    }

    if (datastream_format == i) {
      printf("Datastream format successfully set to %d\n", datastream_format);
    } else {
      printf("ERROR: Failed to set datastream format to %d!!!\n", datastream_format);
    }
  }

  ///
  // Set/Read the GPS dynamics mode
  ///

  printf("\n\nCycle through available GPS Dynamics Modes....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 8; i >= 0; i--) {
    // Skip 1, per DCP
    if (i != 1) {
      // Set the dynamics mode
      gps_dynamics_mode = i;

      while (mip_3dm_cmd_gps_dynamics_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &gps_dynamics_mode) !=
             MIP_INTERFACE_OK) {
      }

      // Reset the gps_dynamics_mode variable
      gps_dynamics_mode = 0;

      // Read back the com mode
      while (mip_3dm_cmd_gps_dynamics_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &gps_dynamics_mode) !=
             MIP_INTERFACE_OK) {
      }

      if (gps_dynamics_mode == i) {
        printf("GPS dynamics mode successfully set to %d\n", gps_dynamics_mode);
      } else {
        printf("ERROR: Failed to set GPS dynamics mode to %d!!!\n", gps_dynamics_mode);
      }
    }
  }

  ///
  // Set/Read the AHRS signal conditioning settings
  ///

  printf("\n\nSet the AHRS signal conditioning settings....\n\n");

  // Read the signal conditioning settings (for a starting point)
  while (mip_3dm_cmd_ahrs_signal_conditioning(&device_interface, MIP_FUNCTION_SELECTOR_READ,
                                              &signal_conditioning_settings) != MIP_INTERFACE_OK) {
  }

  // Set the signal conditioning settings
  signal_conditioning_settings.mag_filter_width = 32;
  signal_conditioning_settings.inertial_filter_width = 32;

  // Write the signal conditioning settings
  while (mip_3dm_cmd_ahrs_signal_conditioning(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                              &signal_conditioning_settings) != MIP_INTERFACE_OK) {
  }

  // Read back the signal conditioning settings
  while (mip_3dm_cmd_ahrs_signal_conditioning(&device_interface, MIP_FUNCTION_SELECTOR_READ,
                                              &readback_signal_conditioning_settings) != MIP_INTERFACE_OK) {
  }

  if ((readback_signal_conditioning_settings.mag_filter_width == signal_conditioning_settings.mag_filter_width) &&
      (readback_signal_conditioning_settings.inertial_filter_width ==
       signal_conditioning_settings.inertial_filter_width)) {
    printf("Signal conditioning settings successfully set\n");
  } else {
    printf("ERROR: Failed to set signal conditioning settings!!!\n");
  }

  ///
  // Set/Read the AHRS timestamp format
  ///

  printf("\n\nCycle through available AHRS timestamp format values....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 2; i >= 1; i--) {
    // Set the enable flag
    ahrs_time_selector = i;

    while (mip_3dm_cmd_ahrs_timestamp(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &ahrs_time_selector,
                                      &ahrs_time_value) != MIP_INTERFACE_OK) {
    }

    readback_ahrs_time_selector = i;

    // Read back the com mode
    while (mip_3dm_cmd_ahrs_timestamp(&device_interface, MIP_FUNCTION_SELECTOR_READ, &readback_ahrs_time_selector,
                                      &ahrs_time_value) != MIP_INTERFACE_OK) {
    }

    printf("Timestamp format test successful with selector %d\n", ahrs_time_selector);
  }

  ///
  // Set/Read the AHRS continuous datastream enable
  ///

  printf("\n\nCycle through available AHRS continuous datastream enable values....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 0; i <= 1; i++) {
    // Set the enable flag
    enable = i;

    while (mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM,
                                              &enable) != MIP_INTERFACE_OK) {
    }

    // Reset the enable variable
    enable = 0;

    // Read back the com mode
    while (mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_READ, MIP_3DM_AHRS_DATASTREAM,
                                              &enable) != MIP_INTERFACE_OK) {
    }

    if (enable == i) {
      printf("Continuous datastream enable successfully set to %d\n", enable);
    } else {
      printf("ERROR: Failed to set continuous datastream enable to %d!!!\n", enable);
    }
  }

  ///
  // Set/Read the GPS continuous datastream enable
  ///

  printf("\n\nCycle through available GPS continuous datastream enable values....\n\n");

  // Loop through valid modes and set/read current come mode
  for (i = 0; i <= 1; i++) {
    // Set the enable flag
    enable = i;

    while (mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_GPS_DATASTREAM,
                                              &enable) != MIP_INTERFACE_OK) {
    }

    // Reset the enable variable
    enable = 0;

    // Read back the com mode
    while (mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_READ, MIP_3DM_GPS_DATASTREAM,
                                              &enable) != MIP_INTERFACE_OK) {
    }

    if (enable == i) {
      printf("\nContinuous datastream enable successfully set to %d\n", enable);
    } else {
      printf("\nERROR: Failed to set continuous datastream enable to %d!!!\n", enable);
    }
  }

  ///
  // Resume the GX3-35 datastreams
  ///

  printf("\n\nResuming device datastreams....\n");

  while (mip_base_cmd_resume(&device_interface) != MIP_INTERFACE_OK) {
  }

  printf("\n\n");

  ///
  // Setup the GX3-35 dataset callbacks (AHRS, GPS), note: no user data is required for this example so we set the
  // pointer to NULL.
  ///

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

  printf("   GX3_35_Test [com_port_num] [baudrate]\n");
  printf("\n\n");
  printf("   Example: \"GX3_35_Test 1 115200\", Opens a connection to the \n");
  printf("             GX3-35 on COM1, with a baudrate of 115200.\n");
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
  printf("GX3-35 Test Program\n");
  printf("Copyright 2011. Microstrain, Inc.\n\n");
}

////////////////////////////////////////////////////////////////////////////////
//
// Print Packet Statistics
//
////////////////////////////////////////////////////////////////////////////////

void print_packet_stats() {
  printf("\r%u AHRS (%u errors)    %u GPS (%u errors) Packets", ahrs_valid_packet_count,
         ahrs_timeout_packet_count + ahrs_checksum_error_packet_count, gps_valid_packet_count,
         gps_timeout_packet_count + gps_checksum_error_packet_count);
}
