/////////////////////////////////////////////////////////////////////////////
//
// GX4-15_Test.c
//
// Test program for the GX4-15
//
// Notes:  This program runs through most of the sdk functions supported
//         by the GX4-15.  It does not permanently alter any of the device
//         settings.
//
//
// External dependencies:
//
//  mip_sdk.h
//  GX4-15_Test.h
//
// Written By: Nathan Miller and Gregg Carpenter
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

#include "GX4-15_Test.h"

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Globals
//
////////////////////////////////////////////////////////////////////////////////

u8 enable_data_stats_output = 0;

// The primary device interface structure
mip_interface device_interface;

// Packet Counters (valid, timeout, and checksum errors)
u32 filter_valid_packet_count = 0;
u32 ahrs_valid_packet_count = 0;

u32 filter_timeout_packet_count = 0;
u32 ahrs_timeout_packet_count = 0;

u32 filter_checksum_error_packet_count = 0;
u32 ahrs_checksum_error_packet_count = 0;

// Example data field storage

// AHRS
mip_ahrs_scaled_gyro curr_ahrs_gyro;
mip_ahrs_scaled_accel curr_ahrs_accel;
mip_ahrs_scaled_mag curr_ahrs_mag;

// FILTER
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
  u16 j;
  u8 com_mode = 0;
  u8 readback_com_mode = 0;
  float angles[3] = {0};
  float readback_angles[3] = {0};
  float offset[3] = {0};
  float readback_offset[3] = {0};
  u16 estimation_control = 0, estimation_control_readback = 0;
  u8 heading_source = 0;
  u8 auto_init = 0;
  float noise[3] = {0};
  float readback_noise[3] = {0};
  float beta[3] = {0};
  float readback_beta[3] = {0};
  mip_low_pass_filter_settings filter_settings;
  float bias_vector[3] = {0};
  u16 duration = 0;
  gx4_imu_diagnostic_device_status_field imu_diagnostic_field;
  gx4_imu_basic_status_field imu_basic_field;
  gx4_15_diagnostic_device_status_field diagnostic_field;
  gx4_15_basic_status_field basic_field;
  mip_filter_zero_update_command zero_update_control, zero_update_readback;
  mip_complementary_filter_settings comp_filter_command, comp_filter_readback;

  mip_filter_accel_magnitude_error_adaptive_measurement_command accel_magnitude_error_command,
      accel_magnitude_error_readback;
  u8 reference_position_enable_command, reference_position_enable_readback;
  double reference_position_command[3], reference_position_readback[3];

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

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // IMU-Direct Mode Testing
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  com_mode = MIP_SDK_GX4_15_IMU_DIRECT_MODE;

  ///
  // Set communication mode
  ///

  printf("----------------------------------------------------------------------\n");
  printf("Attempting to set communications mode to IMU Direct mode\n");
  printf("----------------------------------------------------------------------\n\n");

  while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK) {
  }

  ///
  // Verify device mode setting
  ///

  while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK) {
  }

  if (com_mode == MIP_SDK_GX4_15_IMU_DIRECT_MODE) {
    printf("Communications mode IMU Direct.\n");

    printf("\n\n");
    Sleep(1500);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Base Command Tests
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///
    // Put the GX4-15 into idle mode
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Idling Device\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_idle(&device_interface) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Try to ping the GX4-15
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Pinging Device\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_ping(&device_interface) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Get the device information
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting Device Information\n");
    printf("----------------------------------------------------------------------\n\n");

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

    printf("Firmware Version => %d.%d.%.2d\n\n", (device_info.firmware_version) / 1000,
           (device_info.firmware_version) % 1000 / 100, (device_info.firmware_version) % 100);

    printf("\n\n");
    Sleep(1500);

    ///
    // Get the supported descriptors
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting Supported descriptors\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_get_device_supported_descriptors(&device_interface, (u8*)device_descriptors,
                                                         &device_descriptors_size) != MIP_INTERFACE_OK) {
    }

    printf("\n\nSupported descriptors:\n\n");

    for (i = 0; i < device_descriptors_size / 2; i++) {
      printf("Descriptor Set: %02x, Descriptor: %02x\n", device_descriptors[i] >> 8, device_descriptors[i] & 0xFF);
      Sleep(100);
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Peform a built-in-test
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Running Built In Test\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_built_in_test(&device_interface, &bit_result) != MIP_INTERFACE_OK) {
    }

    printf("\nBIT Result (should be 0x00000000) => 0x%08x\n\n", bit_result);

    printf("\n\n");
    Sleep(1500);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // 3DM Command Tests
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///
    // Get AHRS Base Rate
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting the AHRS datastream base rate\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_3dm_cmd_get_ahrs_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK) {
    }

    printf("\nAHRS Base Rate => %d Hz\n\n", base_rate);

    printf("\n\n");
    Sleep(1500);

    ///
    // Get Device Status
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Requesting BASIC Status Report:\n");
    printf("----------------------------------------------------------------------\n\n");

    // Request basic status report
    while (mip_3dm_cmd_hw_specific_imu_device_status(&device_interface, GX4_IMU_MODEL_NUMBER, GX4_IMU_BASIC_STATUS_SEL,
                                                     &imu_basic_field) != MIP_INTERFACE_OK) {
    }

    printf("Model Number: \t\t\t\t\t%04u\n", imu_basic_field.device_model);
    printf("Status Selector: \t\t\t\t%s\n", imu_basic_field.status_selector == GX4_IMU_BASIC_STATUS_SEL
                                                ? "Basic Status Report"
                                                : "Diagnostic Status Report");
    printf("Status Flags: \t\t\t\t\t0x%08x\n", imu_basic_field.status_flags);
    printf("System Millisecond Timer Count: \t\t%llu ms\n\n", imu_basic_field.system_timer_ms);

    printf("Requesting DIAGNOSTIC Status Report:\n");

    // Request diagnostic status report
    while (mip_3dm_cmd_hw_specific_imu_device_status(&device_interface, GX4_IMU_MODEL_NUMBER,
                                                     GX4_IMU_DIAGNOSTICS_STATUS_SEL,
                                                     &imu_diagnostic_field) != MIP_INTERFACE_OK) {
    }

    printf("Model Number: \t\t\t\t\t%04u\n", imu_diagnostic_field.device_model);
    printf("Status Selector: \t\t\t\t%s\n", imu_diagnostic_field.status_selector == GX4_IMU_BASIC_STATUS_SEL
                                                ? "Basic Status Report"
                                                : "Diagnostic Status Report");
    printf("Status Flags: \t\t\t\t\t0x%08x\n", imu_diagnostic_field.status_flags);
    printf("System Millisecond Timer Count: \t\t%llu ms\n", imu_diagnostic_field.system_timer_ms);
    printf("Magnetometer: \t\t\t\t\t%s\n", imu_diagnostic_field.has_mag == 1 ? "DETECTED" : "NOT-DETECTED");
    printf("Pressure Sensor: \t\t\t\t%s\n", imu_diagnostic_field.has_pressure == 1 ? "DETECTED" : "NOT-DETECTED");
    printf("Gyro Range Reported: \t\t\t\t%u deg/s\n", imu_diagnostic_field.gyro_range);
    printf("Accel Range Reported: \t\t\t\t%u G\n", imu_diagnostic_field.accel_range);
    printf("Magnetometer Range Reported: \t\t\t%f Gs\n", imu_diagnostic_field.mag_range);
    printf("Pressure Range Reported: \t\t\t%f hPa\n", imu_diagnostic_field.pressure_range);
    printf("Measured Internal Temperature: \t\t\t%f degrees C\n", imu_diagnostic_field.temp_degc);
    printf("Last Temperature Measured: \t\t\t%u ms\n", imu_diagnostic_field.last_temp_read_ms);
    printf("Bad Temperature Sensor Detected: \t\t%s\n", imu_diagnostic_field.temp_sensor_error == 1 ? "TRUE" : "FALSE");
    printf("Number Received GPS Pulse-Per-Second Pulses: \t%u Pulses\n", imu_diagnostic_field.num_gps_pps_triggers);
    printf("Time of Last GPS Pulse-Per-Second Pulse: \t%u ms\n", imu_diagnostic_field.last_gps_pps_trigger_ms);
    printf("Data Streaming Enabled: \t\t\t%s\n", imu_diagnostic_field.stream_enabled == 1 ? "TRUE" : "FALSE");
    printf("Number of Dropped Communication Packets: \t%u packets\n", imu_diagnostic_field.dropped_packets);
    printf("Communications Port Bytes Written: \t\t%u Bytes\n", imu_diagnostic_field.com_port_bytes_written);
    printf("Communications Port Bytes Read: \t\t%u Bytes\n", imu_diagnostic_field.com_port_bytes_read);
    printf("Communications Port Write Overruns: \t\t%u Bytes\n", imu_diagnostic_field.com_port_write_overruns);
    printf("Communications Port Read Overruns: \t\t%u Bytes\n", imu_diagnostic_field.com_port_read_overruns);

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read Coning and Sculling Compensation
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Disabling Coning and Sculling compensation\n");
    printf("----------------------------------------------------------------------\n\n");

    enable = MIP_3DM_CONING_AND_SCULLING_DISABLE;

    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &enable) !=
           MIP_INTERFACE_OK) {
    }

    printf("Reading Coning and Sculling compensation enabled state:\n");

    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface, MIP_FUNCTION_SELECTOR_READ, &enable) !=
           MIP_INTERFACE_OK) {
    }

    printf("%s\n\n", enable == MIP_3DM_CONING_AND_SCULLING_DISABLE ? "DISABLED" : "ENABLED");

    printf("Enabling Coning and Sculling compensation.\n");

    enable = MIP_3DM_CONING_AND_SCULLING_ENABLE;

    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &enable) !=
           MIP_INTERFACE_OK) {
    }

    printf("Reading Coning and Sculling compensation enabled state:\n");

    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface, MIP_FUNCTION_SELECTOR_READ, &enable) !=
           MIP_INTERFACE_OK) {
    }

    printf("%s\n\n", enable == MIP_3DM_CONING_AND_SCULLING_DISABLE ? "DISABLED" : "ENABLED");

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read Accel Bias
    ///

    bias_vector[0] = 1.0f;
    bias_vector[1] = 2.0f;
    bias_vector[2] = 3.0f;

    printf("----------------------------------------------------------------------\n");
    printf("Accel Bias Vector\n");
    printf("----------------------------------------------------------------------\n\n");

    printf("Setting Accel Bias Vector:\n");
    printf("bias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1],
           bias_vector[2]);

    while (mip_3dm_cmd_accel_bias(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, bias_vector) != MIP_INTERFACE_OK) {
    }

    memset(bias_vector, 0, 3 * sizeof(float));

    printf("Reading current Accel Bias Vector:\n");

    while (mip_3dm_cmd_accel_bias(&device_interface, MIP_FUNCTION_SELECTOR_READ, bias_vector) != MIP_INTERFACE_OK) {
    }

    printf("bias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1],
           bias_vector[2]);

    printf("Resetting Accel Bias to default state.\n\n");

    while (mip_3dm_cmd_accel_bias(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read Gyro Bias
    ///

    bias_vector[0] = 4.0f;
    bias_vector[1] = 5.0f;
    bias_vector[2] = 6.0f;

    printf("----------------------------------------------------------------------\n");
    printf("Gyro Bias Vector\n");
    printf("----------------------------------------------------------------------\n\n");

    printf("Setting Gyro Bias Vector:\n");
    printf("bias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1],
           bias_vector[2]);

    while (mip_3dm_cmd_gyro_bias(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, bias_vector) != MIP_INTERFACE_OK) {
    }

    memset(bias_vector, 0, 3 * sizeof(float));

    printf("Reading current Gyro Bias Vector:\n");

    while (mip_3dm_cmd_gyro_bias(&device_interface, MIP_FUNCTION_SELECTOR_READ, bias_vector) != MIP_INTERFACE_OK) {
    }

    printf("bias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1],
           bias_vector[2]);

    printf("Resetting Gyro Bias to default state.\n\n");

    while (mip_3dm_cmd_gyro_bias(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Capture Gyro Bias
    ///

    printf("----------------------------------------------------------------------\n");
    printf(
        "Performing Gyro Bias capture.\nPlease keep device stationary during the 5 second gyro bias capture "
        "interval\n");
    printf("----------------------------------------------------------------------\n\n");

    duration = 5000;  // milliseconds

    while (mip_3dm_cmd_capture_gyro_bias(&device_interface, duration, bias_vector) != MIP_INTERFACE_OK) {
    }

    printf("Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0],
           bias_vector[1], bias_vector[2]);

    printf("\n\n");
    Sleep(1500);

    ///
    // Setup the AHRS datastream format
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting the AHRS message format\n");
    printf("----------------------------------------------------------------------\n\n");

    data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
    data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;

    data_stream_format_decimation[0] = 0x64;
    data_stream_format_decimation[1] = 0x64;

    data_stream_format_num_entries = 2;

    while (mip_3dm_cmd_ahrs_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                           &data_stream_format_num_entries, data_stream_format_descriptors,
                                           data_stream_format_decimation) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Poll the AHRS data
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Polling AHRS Data.\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_3dm_cmd_poll_ahrs(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries,
                                 data_stream_format_descriptors) != MIP_INTERFACE_OK) {
    }

  } else {
    printf("ERROR: IMU_Direct mode not established\n\n");
  }

  printf("\n\n");
  Sleep(1500);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // Standard Mode Tests
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  device_descriptors_size = 128 * 2;
  com_mode = MIP_SDK_GX4_15_IMU_STANDARD_MODE;

  printf("----------------------------------------------------------------------\n");
  printf("Putting Device Into Standard Mode\n");
  printf("----------------------------------------------------------------------\n\n");

  // Set communication mode
  while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK) {
  }

  // Verify device mode setting
  while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK) {
  }

  printf("\n\n");
  Sleep(1500);

  if (com_mode == MIP_SDK_GX4_15_IMU_STANDARD_MODE) {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Base Command Tests
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///
    // Put the GX4-15 into idle mode
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Idling Device\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_idle(&device_interface) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Try to ping the GX4-15
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Pinging Device\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_ping(&device_interface) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Get the device information
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting Device Information\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_get_device_info(&device_interface, &device_info) != MIP_INTERFACE_OK) {
    }

    printf("Device Info:\n");
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

    printf("Firmware Version => %d.%d.%.2d\n\n", (device_info.firmware_version) / 1000,
           (device_info.firmware_version) % 1000 / 100, (device_info.firmware_version) % 100);

    printf("\n\n");
    Sleep(1500);

    ///
    // Get the supported descriptors
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting Supported descriptors\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_get_device_supported_descriptors(&device_interface, (u8*)device_descriptors,
                                                         &device_descriptors_size) != MIP_INTERFACE_OK) {
    }

    printf("\n\nSupported descriptors:\n\n");

    for (i = 0; i < device_descriptors_size / 2; i++) {
      printf("Descriptor Set: %02x, Descriptor: %02x\n", device_descriptors[i] >> 8, device_descriptors[i] & 0xFF);
      Sleep(100);
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Peform a built-in-test
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Running Built In Test\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_built_in_test(&device_interface, &bit_result) != MIP_INTERFACE_OK) {
    }

    printf("\nBIT Result (should be 0x00000000) => 0x%08x\n\n", bit_result);

    printf("\n\n");
    Sleep(1500);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // 3DM Command Tests
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///
    // Get AHRS Base Rate
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting the AHRS datastream base rate\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_3dm_cmd_get_ahrs_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK) {
    }

    printf("\nAHRS Base Rate => %d Hz\n", base_rate);

    printf("\n\n");
    Sleep(1500);

    ///
    // Get Estimation Filter Base Rate
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting the Estimation Filter datastream base rate\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_3dm_cmd_get_filter_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK) {
    }

    printf("\nFILTER Base Rate => %d Hz\n", base_rate);

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read Coning and Sculling Compensation
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Toggling Coning and Sculling compensation\n");
    printf("----------------------------------------------------------------------\n\n");

    enable = MIP_3DM_CONING_AND_SCULLING_DISABLE;

    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &enable) !=
           MIP_INTERFACE_OK) {
    }

    printf("Reading Coning and Sculling compensation enabled state:\n");

    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface, MIP_FUNCTION_SELECTOR_READ, &enable) !=
           MIP_INTERFACE_OK) {
    }

    printf("%s\n\n", enable == MIP_3DM_CONING_AND_SCULLING_DISABLE ? "DISABLED" : "ENABLED");

    printf("Enabling Coning and Sculling compensation.\n");

    enable = MIP_3DM_CONING_AND_SCULLING_ENABLE;

    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &enable) !=
           MIP_INTERFACE_OK) {
    }

    printf("Reading Coning and Sculling compensation enabled state:\n");

    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface, MIP_FUNCTION_SELECTOR_READ, &enable) !=
           MIP_INTERFACE_OK) {
    }

    printf("%s\n\n", enable == MIP_3DM_CONING_AND_SCULLING_DISABLE ? "DISABLED" : "ENABLED");

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read Accel Bias
    ///

    bias_vector[0] = 1.0f;
    bias_vector[1] = 2.0f;
    bias_vector[2] = 3.0f;

    printf("----------------------------------------------------------------------\n");
    printf("Accel Bias Vector\n");
    printf("----------------------------------------------------------------------\n\n");

    printf("Setting Accel Bias Vector:\n");
    printf("bias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1],
           bias_vector[2]);

    while (mip_3dm_cmd_accel_bias(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, bias_vector) != MIP_INTERFACE_OK) {
    }

    memset(bias_vector, 0, 3 * sizeof(float));

    printf("Reading current Accel Bias Vector:\n");

    while (mip_3dm_cmd_accel_bias(&device_interface, MIP_FUNCTION_SELECTOR_READ, bias_vector) != MIP_INTERFACE_OK) {
    }

    printf("bias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1],
           bias_vector[2]);

    printf("Resetting Accel Bias to default state.\n\n");

    while (mip_3dm_cmd_accel_bias(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read Gyro Bias
    ///

    bias_vector[0] = 4.0f;
    bias_vector[1] = 5.0f;
    bias_vector[2] = 6.0f;

    printf("----------------------------------------------------------------------\n");
    printf("Gyro Bias Vector\n");
    printf("----------------------------------------------------------------------\n\n");

    printf("Setting Gyro Bias Vector:\n");
    printf("bias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1],
           bias_vector[2]);

    while (mip_3dm_cmd_gyro_bias(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, bias_vector) != MIP_INTERFACE_OK) {
    }

    memset(bias_vector, 0, 3 * sizeof(float));

    printf("Reading current Gyro Bias Vector:\n");

    while (mip_3dm_cmd_gyro_bias(&device_interface, MIP_FUNCTION_SELECTOR_READ, bias_vector) != MIP_INTERFACE_OK) {
    }

    printf("bias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1],
           bias_vector[2]);

    printf("Resetting Gyro Bias to default state.\n\n");

    while (mip_3dm_cmd_gyro_bias(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Capture Gyro Bias
    ///

    printf("----------------------------------------------------------------------\n");
    printf(
        "Performing Gyro Bias capture.\nPlease keep device stationary during the 5 second gyro bias capture "
        "interval\n");
    printf("----------------------------------------------------------------------\n\n");

    duration = 5000;  // milliseconds

    while (mip_3dm_cmd_capture_gyro_bias(&device_interface, duration, bias_vector) != MIP_INTERFACE_OK) {
    }

    printf("Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0],
           bias_vector[1], bias_vector[2]);

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read the complementary filter values
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting the complementary filter values\n");
    printf("----------------------------------------------------------------------\n\n");

    comp_filter_command.north_compensation_enable = 0;
    comp_filter_command.up_compensation_enable = 0;

    comp_filter_command.north_compensation_time_constant = 30;
    comp_filter_command.up_compensation_time_constant = 30;

    while (mip_3dm_cmd_complementary_filter_settings(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                                     &comp_filter_command) != MIP_INTERFACE_OK) {
    }

    // Read back the complementary filter values
    while (mip_3dm_cmd_complementary_filter_settings(&device_interface, MIP_FUNCTION_SELECTOR_READ,
                                                     &comp_filter_readback) != MIP_INTERFACE_OK) {
    }

    if ((comp_filter_command.north_compensation_enable == comp_filter_readback.north_compensation_enable) &&
        (comp_filter_command.up_compensation_enable == comp_filter_readback.up_compensation_enable) &&
        (abs(comp_filter_command.north_compensation_time_constant -
             comp_filter_readback.north_compensation_time_constant) < 0.001) &&
        (abs(comp_filter_command.up_compensation_time_constant - comp_filter_readback.up_compensation_time_constant) <
         0.001)) {
      printf("Complementary filter values successfully set.\n");
    } else {
      printf("ERROR: Failed to set complementary filter values!!!\n");
      printf("Sent values:     Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
             comp_filter_command.up_compensation_enable, comp_filter_command.north_compensation_enable,
             comp_filter_command.up_compensation_time_constant, comp_filter_command.north_compensation_time_constant);
      printf("Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
             comp_filter_readback.up_compensation_enable, comp_filter_readback.north_compensation_enable,
             comp_filter_readback.up_compensation_time_constant, comp_filter_readback.north_compensation_time_constant);
    }

    printf("\n\nLoading the default complementary filter values.\n\n");

    while (mip_3dm_cmd_complementary_filter_settings(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) !=
           MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Get Device Status
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Requesting BASIC Status Report\n");
    printf("----------------------------------------------------------------------\n\n");

    // Request basic status report
    while (mip_3dm_cmd_hw_specific_device_status(&device_interface, GX4_15_MODEL_NUMBER, GX4_15_BASIC_STATUS_SEL,
                                                 &basic_field) != MIP_INTERFACE_OK) {
    }

    printf("Model Number: \t\t\t\t\t%04u\n", basic_field.device_model);
    printf("Status Selector: \t\t\t\t%s\n",
           basic_field.status_selector == GX4_15_BASIC_STATUS_SEL ? "Basic Status Report" : "Diagnostic Status Report");
    printf("Status Flags: \t\t\t\t\t0x%08x\n", basic_field.status_flags);
    printf("System Microsecond Timer Count: \t\t%llu ms\n\n", basic_field.system_timer_ms);

    printf("Requesting DIAGNOSTIC Status Report:\n");

    // Request diagnostic status report
    while (mip_3dm_cmd_hw_specific_device_status(&device_interface, GX4_15_MODEL_NUMBER, GX4_15_DIAGNOSTICS_STATUS_SEL,
                                                 &diagnostic_field) != MIP_INTERFACE_OK) {
    }

    printf("Model Number: \t\t\t\t\t%04u\n", diagnostic_field.device_model);
    printf("Status Selector: \t\t\t\t%s\n", diagnostic_field.status_selector == GX4_15_BASIC_STATUS_SEL
                                                ? "Basic Status Report"
                                                : "Diagnostic Status Report");
    printf("Status Flags: \t\t\t\t\t0x%08x\n", diagnostic_field.status_flags);
    printf("System Millisecond Timer Count: \t\t%llu ms\n", diagnostic_field.system_timer_ms);
    printf("IMU Streaming Enabled: \t\t\t\t%s\n", diagnostic_field.imu_stream_enabled == 1 ? "TRUE" : "FALSE");
    printf("FILTER Streaming Enabled: \t\t\t\t%s\n", diagnostic_field.filter_stream_enabled == 1 ? "TRUE" : "FALSE");
    printf("Number of Dropped IMU Packets: \t\t\t%u packets\n", diagnostic_field.imu_dropped_packets);
    printf("Number of Dropped FILTER Packets: \t\t\t%u packets\n", diagnostic_field.filter_dropped_packets);
    printf("Communications Port Bytes Written: \t\t%u Bytes\n", diagnostic_field.com1_port_bytes_written);
    printf("Communications Port Bytes Read: \t\t%u Bytes\n", diagnostic_field.com1_port_bytes_read);
    printf("Communications Port Write Overruns: \t\t%u Bytes\n", diagnostic_field.com1_port_write_overruns);
    printf("Communications Port Read Overruns: \t\t%u Bytes\n", diagnostic_field.com1_port_read_overruns);
    printf("IMU Parser Errors: \t\t\t\t%u Errors\n", diagnostic_field.imu_parser_errors);
    printf("IMU Message Count: \t\t\t\t%u Messages\n", diagnostic_field.imu_message_count);
    printf("IMU Last Message Received: \t\t\t%u ms\n", diagnostic_field.imu_last_message_ms);

    printf("\n\n");
    Sleep(1500);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Filter Command Tests
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///
    // Reset the filter
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Resetting the Filter\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Initialize the filter with Euler Angles
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Initializing the Filter with Euler angles\n");
    printf("----------------------------------------------------------------------\n\n");

    angles[0] = angles[1] = angles[2] = 0;

    while (mip_filter_set_init_attitude(&device_interface, angles) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Reset the filter
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Resetting the Filter\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Initialize the filter with a heading
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Initializing the Filter with a heading angle\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_filter_set_init_heading(&device_interface, angles[0]) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Reset the filter
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Resetting the Filter\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read Estimation control bits
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Cycling through Estimation Control Flags\n");
    printf("----------------------------------------------------------------------\n\n");

    // negate successive bitfields
    for (j = 0; j < 0x002; j++) {
      estimation_control = (0xFFFF & ~j);

      // Set control bits
      while (mip_filter_estimation_control(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &estimation_control) !=
             MIP_INTERFACE_OK) {
      }

      // Readback set control bits
      while (mip_filter_estimation_control(&device_interface, MIP_FUNCTION_SELECTOR_READ,
                                           &estimation_control_readback) != MIP_INTERFACE_OK) {
      }

      if (estimation_control != estimation_control_readback) {
        printf("ERROR:\n");

        if ((estimation_control_readback & FILTER_ESTIMATION_CONTROL_GYRO_BIAS) != 0 &&
            (estimation_control & FILTER_ESTIMATION_CONTROL_GYRO_BIAS) == 0)
          printf("Gyroscope Bias Estimation NOT DISABLED\n");
      }
    }

    printf("\n\nResetting Estimation Control Flags to default state.\n\n");

    while (mip_filter_estimation_control(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, &estimation_control) !=
           MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Cycle through the available auto-init values
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Cycle through available auto-init values\n");
    printf("----------------------------------------------------------------------\n\n");

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
        printf("Auto-init successfully set to %d\n", i);
      } else {
        printf("ERROR: Failed to set auto-init to %d!!!\n", i);
      }
    }

    printf("\n\nLoading the default auto-init value.\n\n");

    while (mip_filter_auto_initialization(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) !=
           MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read the accel noise values
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting the accel noise values\n");
    printf("----------------------------------------------------------------------\n\n");

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

    printf("\n\nLoading the default accel noise values.\n\n");

    while (mip_filter_accel_noise(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read the gyro noise values
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting the gyro noise values\n");
    printf("----------------------------------------------------------------------\n\n");

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

    printf("\n\nLoading the default gyro noise values.\n\n");

    while (mip_filter_gyro_noise(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read the gyro bias model values
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting the gyro bias model values\n");
    printf("----------------------------------------------------------------------\n\n");

    noise[0] = 0.1;
    noise[1] = 0.2;
    noise[2] = 0.3;

    beta[0] = 0.1;
    beta[1] = 0.2;
    beta[2] = 0.3;

    while (mip_filter_gyro_bias_model(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, beta, noise) !=
           MIP_INTERFACE_OK) {
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
      printf("Sent values:     Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", beta[0], beta[1], beta[2],
             noise[0], noise[1], noise[2]);
      printf("Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", readback_beta[0],
             readback_beta[1], readback_beta[2], readback_noise[0], readback_noise[1], readback_noise[2]);
    }

    printf("\n\nLoading the default gyro bias model values.\n\n");

    while (mip_filter_gyro_bias_model(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL, NULL) !=
           MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Angular Rate Zero Update Control
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting Zero Angular-Rate-Update threshold\n");
    printf("----------------------------------------------------------------------\n\n");

    zero_update_control.threshold = 0.05;  // rads/s
    zero_update_control.enable = 1;        // enable zero-angular-rate update

    // Set ZUPT parameters
    while (mip_filter_zero_angular_rate_update_control(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                                       &zero_update_control) != MIP_INTERFACE_OK) {
    }

    // Read back parameter settings
    while (mip_filter_zero_angular_rate_update_control(&device_interface, MIP_FUNCTION_SELECTOR_READ,
                                                       &zero_update_readback) != MIP_INTERFACE_OK) {
    }

    if (zero_update_control.enable != zero_update_readback.enable ||
        zero_update_control.threshold != zero_update_readback.threshold)
      printf("ERROR configuring Zero Angular Rate Update.\n");

    printf("\n\nResetting Zero Angular Rate Update Control to default parameters.\n\n");

    // Reset to default value
    while (mip_filter_zero_angular_rate_update_control(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) !=
           MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Tare Orientation
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Performing Tare Orientation Command\n");
    printf("----------------------------------------------------------------------\n\n");

    printf("Re-initializing filter (required for tare)\n\n");

    // Re-initialize the filter
    angles[0] = angles[1] = angles[2] = 0;

    while (mip_filter_set_init_attitude(&device_interface, angles) != MIP_INTERFACE_OK) {
    }

    // Wait for Filter to re-establish running state
    Sleep(5000);

    // Cycle through axes combinations
    for (i = 1; i < 8; i++) {
      if (mip_filter_tare_orientation(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, i) != MIP_INTERFACE_OK) {
        printf("ERROR: Failed Axis - ");

        if (i & FILTER_TARE_ROLL_AXIS) printf(" Roll Axis ");

        if (i & FILTER_TARE_PITCH_AXIS) printf(" Pitch Axis ");

        if (i & FILTER_TARE_YAW_AXIS) printf(" Yaw Axis ");
      } else {
        printf("Tare Configuration = %d\n", i);

        printf("Tared -");

        if (i & FILTER_TARE_ROLL_AXIS) printf(" Roll Axis ");

        if (i & FILTER_TARE_PITCH_AXIS) printf(" Pitch Axis ");

        if (i & FILTER_TARE_YAW_AXIS) printf(" Yaw Axis ");

        printf("\n\n");
      }

      Sleep(1000);
    }

    printf("\n\nRestoring Orientation to default value.\n\n");

    // Restore to default value
    while (mip_filter_tare_orientation(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, 0) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Commanded Zero Angular-Rate Update
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Performing Commanded Zero-Angular-Rate Update\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_filter_commanded_zero_angular_rate_update(&device_interface) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read the Reference position
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Reference Position\n");
    printf("----------------------------------------------------------------------\n\n");

    reference_position_enable_command = 1;

    reference_position_command[0] = 1.0;
    reference_position_command[1] = 2.0;
    reference_position_command[2] = 3.0;

    while (mip_filter_reference_position(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                         &reference_position_enable_command,
                                         reference_position_command) != MIP_INTERFACE_OK) {
    }

    // Read back the reference position
    while (mip_filter_reference_position(&device_interface, MIP_FUNCTION_SELECTOR_READ,
                                         &reference_position_enable_readback,
                                         reference_position_readback) != MIP_INTERFACE_OK) {
    }

    if ((reference_position_enable_command == reference_position_enable_readback) &&
        (abs(reference_position_command[0] - reference_position_readback[0]) < 0.001) &&
        (abs(reference_position_command[1] - reference_position_readback[1]) < 0.001) &&
        (abs(reference_position_command[2] - reference_position_readback[2]) < 0.001)) {
      printf("Reference position successfully set\n");
    } else {
      printf("ERROR: Failed to set the reference position!!!\n");
    }

    printf("\n\nLoading the default declination source.\n\n");

    while (mip_filter_reference_position(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL, NULL) !=
           MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Set/Read the accel magnitude error adaptive measurement values
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting the accel magnitude error adaptive measurement values\n");
    printf("----------------------------------------------------------------------\n\n");

    accel_magnitude_error_command.enable = 0;
    accel_magnitude_error_command.low_pass_cutoff = 10;
    accel_magnitude_error_command.min_1sigma = 2.0;
    accel_magnitude_error_command.low_limit = -2.0;
    accel_magnitude_error_command.high_limit = 2.0;
    accel_magnitude_error_command.low_limit_1sigma = 4.0;
    accel_magnitude_error_command.high_limit_1sigma = 4.0;

    while (mip_filter_accel_magnitude_error_adaptive_measurement(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                                                 &accel_magnitude_error_command) != MIP_INTERFACE_OK) {
    }

    // Read back the accel magnitude error adaptive measurement values
    while (mip_filter_accel_magnitude_error_adaptive_measurement(&device_interface, MIP_FUNCTION_SELECTOR_READ,
                                                                 &accel_magnitude_error_readback) != MIP_INTERFACE_OK) {
    }

    if ((accel_magnitude_error_command.enable == accel_magnitude_error_readback.enable) &&
        (abs(accel_magnitude_error_command.low_pass_cutoff - accel_magnitude_error_readback.low_pass_cutoff) < 0.001) &&
        (abs(accel_magnitude_error_command.min_1sigma - accel_magnitude_error_readback.min_1sigma) < 0.001) &&
        (abs(accel_magnitude_error_command.low_limit - accel_magnitude_error_readback.low_limit) < 0.001) &&
        (abs(accel_magnitude_error_command.high_limit - accel_magnitude_error_readback.high_limit) < 0.001) &&
        (abs(accel_magnitude_error_command.low_limit_1sigma - accel_magnitude_error_readback.low_limit_1sigma) <
         0.001) &&
        (abs(accel_magnitude_error_command.high_limit_1sigma - accel_magnitude_error_readback.high_limit_1sigma) <
         0.001)) {
      printf("accel magnitude error adaptive measurement values successfully set.\n");
    } else {
      printf("ERROR: Failed to set accel magnitude error adaptive measurement values!!!\n");
      printf("Sent values:     Enable: %i, Parameters: %f %f %f %f %f %f\n", accel_magnitude_error_command.enable,
             accel_magnitude_error_command.low_pass_cutoff, accel_magnitude_error_command.min_1sigma,
             accel_magnitude_error_command.low_limit, accel_magnitude_error_command.high_limit,
             accel_magnitude_error_command.low_limit_1sigma, accel_magnitude_error_command.high_limit_1sigma);

      printf("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f\n", accel_magnitude_error_readback.enable,
             accel_magnitude_error_readback.low_pass_cutoff, accel_magnitude_error_readback.min_1sigma,
             accel_magnitude_error_readback.low_limit, accel_magnitude_error_readback.high_limit,
             accel_magnitude_error_readback.low_limit_1sigma, accel_magnitude_error_readback.high_limit_1sigma);
    }

    printf("\n\nLoading the default accel magnitude error adaptive measurement values.\n\n");

    while (mip_filter_accel_magnitude_error_adaptive_measurement(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT,
                                                                 NULL) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Streaming Data Tests
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///
    // Setup the GX4-15 dataset callbacks
    ///

    if (mip_interface_add_descriptor_set_callback(&device_interface, MIP_FILTER_DATA_SET, NULL,
                                                  &filter_packet_callback) != MIP_INTERFACE_OK)
      return -1;

    if (mip_interface_add_descriptor_set_callback(&device_interface, MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback) !=
        MIP_INTERFACE_OK)
      return -1;

    // Enable the output of data statistics
    enable_data_stats_output = 1;

    ///
    // Setup the AHRS datastream format
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting the AHRS message format\n");
    printf("----------------------------------------------------------------------\n\n");

    data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
    data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;

    data_stream_format_decimation[0] = 0x32;
    data_stream_format_decimation[1] = 0x32;

    data_stream_format_num_entries = 2;

    while (mip_3dm_cmd_ahrs_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                           &data_stream_format_num_entries, data_stream_format_descriptors,
                                           data_stream_format_decimation) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Poll the AHRS data
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Polling AHRS Data.\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_3dm_cmd_poll_ahrs(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries,
                                 data_stream_format_descriptors) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Setup the FILTER datastream format
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting the Estimation Filter datastream format\n");
    printf("----------------------------------------------------------------------\n\n");

    data_stream_format_descriptors[0] = MIP_FILTER_DATA_ATT_EULER_ANGLES;

    data_stream_format_decimation[0] = 0x32;

    data_stream_format_num_entries = 1;

    while (mip_3dm_cmd_filter_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
                                             &data_stream_format_num_entries, data_stream_format_descriptors,
                                             data_stream_format_decimation) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);

    ///
    // Poll the Estimation Filter data
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Polling Estimation Filter Data.\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_3dm_cmd_poll_filter(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries,
                                   data_stream_format_descriptors) != MIP_INTERFACE_OK) {
    }

    printf("\n\n");
    Sleep(1500);
  } else {
    printf("ERROR: Standard mode not established\n\n");
  }

  // Disable the output of data statistics
  enable_data_stats_output = 0;

  ///
  // Enable the AHRS datastream
  ///

  printf("\n----------------------------------------------------------------------\n");
  printf("Enable the AHRS datastream\n");
  printf("----------------------------------------------------------------------\n\n");

  enable = 0x01;

  while (mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM,
                                            &enable) != MIP_INTERFACE_OK) {
  }

  printf("\n\n");
  Sleep(1500);

  ///
  // Enable the FILTER datastream
  ///

  printf("----------------------------------------------------------------------\n");
  printf("Enable the FILTER datastream\n");
  printf("----------------------------------------------------------------------\n\n");

  enable = 0x01;

  while (mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM,
                                            &enable) != MIP_INTERFACE_OK) {
  }

  printf("\n\n");
  Sleep(1500);

  ///
  // Wait for packets to arrive
  ///

  printf("----------------------------------------------------------------------\n");
  printf("Processing incoming packets\n");
  printf("----------------------------------------------------------------------\n\n\n");

  // Enable the output of data statistics
  enable_data_stats_output = 1;

  while (1) {
    // Update the parser (this function reads the port and parses the bytes
    mip_interface_update(&device_interface);

    // Be nice to other programs
    Sleep(1);
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// FILTER Packet Callback
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
// Print Command-Line Help
//
////////////////////////////////////////////////////////////////////////////////

void print_command_line_usage() {
  printf("\n\n");
  printf("Usage:\n");
  printf("-----------------------------------------------------------------------\n\n");

  printf("   GX4-15_Test [com_port_num] [baudrate]\n");
  printf("\n\n");
  printf("   Example: \"GX4-15_Test 1 115200\", Opens a connection to the \n");
  printf("             GX4-15 on COM1, with a baudrate of 115200.\n");
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
  printf("GX4-15 Test Program\n");
  printf("Copyright 2014. LORD Microstrain Sensing Systems\n\n");
}

////////////////////////////////////////////////////////////////////////////////
//
// Print Packet Statistics
//
////////////////////////////////////////////////////////////////////////////////

void print_packet_stats() {
  if (enable_data_stats_output) {
    printf("\r%u FILTER (%u errors)    %u AHRS (%u errors)   Packets", filter_valid_packet_count,
           filter_timeout_packet_count + filter_checksum_error_packet_count, ahrs_valid_packet_count,
           ahrs_timeout_packet_count + ahrs_checksum_error_packet_count);
  }
}

///////////////////////////////////////////////////////////////////////////////
//
// Device specific Status Acquisition Routines
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector,
//! u8 *response_buffer)
//
//! @section DESCRIPTION
//! Requests GX4-IMU Basic or Diagnostic Status Message.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u16 model_number - LORD Microstrain Sensing Systems model number for GX4 IMU (6237)
//! @param [in] u8 status selector - specifies which type of status message is being requested.
//! @paran [out] u8 *response_buffer - pointer to the location to store response bytes.
//
//! @retval MIP_INTERFACE_ERROR  Interface not initialized or device not in IMU Direct Mode.\n
//! @retval MIP_INTERFACE_OK     Status message successfully recieved.\n
//
//! @section NOTES
//!
//! This function should only be called in IMU Direct Mode.
///////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface* device_interface, u16 model_number, u8 status_selector,
                                              u8* response_buffer) {
  gx4_imu_basic_status_field* basic_ptr;
  gx4_imu_diagnostic_device_status_field* diagnostic_ptr;
  u16 response_size = MIP_FIELD_HEADER_SIZE;

  if (status_selector == GX4_IMU_BASIC_STATUS_SEL)
    response_size += sizeof(gx4_imu_basic_status_field);
  else if (status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL)
    response_size += sizeof(gx4_imu_diagnostic_device_status_field);

  while (mip_3dm_cmd_device_status(device_interface, model_number, status_selector, response_buffer, &response_size) !=
         MIP_INTERFACE_OK) {
  }

  if (status_selector == GX4_IMU_BASIC_STATUS_SEL) {
    if (response_size != sizeof(gx4_imu_basic_status_field))
      return MIP_INTERFACE_ERROR;
    else if (MIP_SDK_CONFIG_BYTESWAP) {
      basic_ptr = (gx4_imu_basic_status_field*)response_buffer;

      byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
      byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
      byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
    }
  } else if (status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL) {
    if (response_size != sizeof(gx4_imu_diagnostic_device_status_field))
      return MIP_INTERFACE_ERROR;
    else if (MIP_SDK_CONFIG_BYTESWAP) {
      diagnostic_ptr = (gx4_imu_diagnostic_device_status_field*)response_buffer;

      byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
      byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
      byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
      byteswap_inplace(&diagnostic_ptr->gyro_range, sizeof(diagnostic_ptr->gyro_range));
      byteswap_inplace(&diagnostic_ptr->mag_range, sizeof(diagnostic_ptr->mag_range));
      byteswap_inplace(&diagnostic_ptr->pressure_range, sizeof(diagnostic_ptr->pressure_range));
      byteswap_inplace(&diagnostic_ptr->temp_degc, sizeof(diagnostic_ptr->temp_degc));
      byteswap_inplace(&diagnostic_ptr->last_temp_read_ms, sizeof(diagnostic_ptr->last_temp_read_ms));
      byteswap_inplace(&diagnostic_ptr->num_gps_pps_triggers, sizeof(diagnostic_ptr->num_gps_pps_triggers));
      byteswap_inplace(&diagnostic_ptr->last_gps_pps_trigger_ms, sizeof(diagnostic_ptr->last_gps_pps_trigger_ms));
      byteswap_inplace(&diagnostic_ptr->dropped_packets, sizeof(diagnostic_ptr->dropped_packets));
      byteswap_inplace(&diagnostic_ptr->com_port_bytes_written, sizeof(diagnostic_ptr->com_port_bytes_written));
      byteswap_inplace(&diagnostic_ptr->com_port_bytes_read, sizeof(diagnostic_ptr->com_port_bytes_read));
      byteswap_inplace(&diagnostic_ptr->com_port_write_overruns, sizeof(diagnostic_ptr->com_port_write_overruns));
      byteswap_inplace(&diagnostic_ptr->com_port_read_overruns, sizeof(diagnostic_ptr->com_port_read_overruns));
    }
  } else
    return MIP_INTERFACE_ERROR;

  return MIP_INTERFACE_OK;
}

///////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8
//! *response_buffer)
//
//! @section DESCRIPTION
//! Requests GX4-15 Basic or Diagnostic Status Message.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u16 model_number - LORD Microstrain Sensing Systems model number for GX4-15 (6233)
//! @param [in] u8 status selector - specifies which type of status message is being requested.
//! @paran [out] u8 *response_buffer - pointer to the location to store response bytes.
//
//! @retval MIP_INTERFACE_ERROR  Interface not initialized or device not in IMU Direct Mode.\n
//! @retval MIP_INTERFACE_OK     Status message successfully recieved.\n
//
//! @section NOTES
//!
///////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_hw_specific_device_status(mip_interface* device_interface, u16 model_number, u8 status_selector,
                                          u8* response_buffer) {
  gx4_15_basic_status_field* basic_ptr;
  gx4_15_diagnostic_device_status_field* diagnostic_ptr;
  u16 response_size = MIP_FIELD_HEADER_SIZE;

  if (status_selector == GX4_15_BASIC_STATUS_SEL)
    response_size += sizeof(gx4_15_basic_status_field);
  else if (status_selector == GX4_15_DIAGNOSTICS_STATUS_SEL)
    response_size += sizeof(gx4_15_diagnostic_device_status_field);

  while (mip_3dm_cmd_device_status(device_interface, model_number, status_selector, response_buffer, &response_size) !=
         MIP_INTERFACE_OK) {
  }

  if (status_selector == GX4_15_BASIC_STATUS_SEL) {
    if (response_size != sizeof(gx4_15_basic_status_field))
      return MIP_INTERFACE_ERROR;
    else if (MIP_SDK_CONFIG_BYTESWAP) {
      basic_ptr = (gx4_15_basic_status_field*)response_buffer;

      byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
      byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
      byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
    }

  } else if (status_selector == GX4_15_DIAGNOSTICS_STATUS_SEL) {
    if (response_size != sizeof(gx4_15_diagnostic_device_status_field))
      return MIP_INTERFACE_ERROR;
    else if (MIP_SDK_CONFIG_BYTESWAP) {
      diagnostic_ptr = (gx4_15_diagnostic_device_status_field*)response_buffer;

      byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
      byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
      byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
      byteswap_inplace(&diagnostic_ptr->imu_dropped_packets, sizeof(diagnostic_ptr->imu_dropped_packets));
      byteswap_inplace(&diagnostic_ptr->filter_dropped_packets, sizeof(diagnostic_ptr->filter_dropped_packets));
      byteswap_inplace(&diagnostic_ptr->com1_port_bytes_written, sizeof(diagnostic_ptr->com1_port_bytes_written));
      byteswap_inplace(&diagnostic_ptr->com1_port_bytes_read, sizeof(diagnostic_ptr->com1_port_bytes_read));
      byteswap_inplace(&diagnostic_ptr->com1_port_write_overruns, sizeof(diagnostic_ptr->com1_port_write_overruns));
      byteswap_inplace(&diagnostic_ptr->com1_port_read_overruns, sizeof(diagnostic_ptr->com1_port_read_overruns));
      byteswap_inplace(&diagnostic_ptr->imu_parser_errors, sizeof(diagnostic_ptr->imu_parser_errors));
      byteswap_inplace(&diagnostic_ptr->imu_message_count, sizeof(diagnostic_ptr->imu_message_count));
      byteswap_inplace(&diagnostic_ptr->imu_last_message_ms, sizeof(diagnostic_ptr->imu_last_message_ms));
    }
  } else
    return MIP_INTERFACE_ERROR;

  return MIP_INTERFACE_OK;
}
