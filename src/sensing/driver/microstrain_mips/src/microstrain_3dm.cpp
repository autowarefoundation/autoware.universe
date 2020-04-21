/*

Copyright (c) 2017, Brian Bingham

This code is licensed under MIT license (see LICENSE file for details)

*/

#include <tf2/LinearMath/Transform.h>
#include <time.h>
#include <algorithm>
#include <string>

#include <vector>
#include "microstrain_diagnostic_updater.h"
#include "microstrain_mips/status_msg.h"

namespace Microstrain {
Microstrain::Microstrain()
    :  // Initialization list
      filter_valid_packet_count_(0),
      ahrs_valid_packet_count_(0),
      gps_valid_packet_count_(0),
      filter_timeout_packet_count_(0),
      ahrs_timeout_packet_count_(0),
      gps_timeout_packet_count_(0),
      filter_checksum_error_packet_count_(0),
      ahrs_checksum_error_packet_count_(0),
      gps_checksum_error_packet_count_(0),
      gps_frame_id_("gps_frame"),
      imu_frame_id_("imu_frame"),
      odom_frame_id_("odom_frame"),
      odom_child_frame_id_("odom_frame"),
      publish_gps_(true),
      publish_imu_(true),
      publish_odom_(true),
      publish_filtered_imu_(false),
      remove_imu_gravity_(false),
      frame_based_enu_(false),
      imu_linear_cov_(std::vector<double>(9, 0.0)),
      imu_angular_cov_(std::vector<double>(9, 0.0)),
      imu_orientation_cov_(std::vector<double>(9, 0.0)) {
  // pass
}
Microstrain::~Microstrain() {
  // pass
}
void Microstrain::run() {
  // Variables for device configuration, ROS parameters, etc.
  u32 com_port, baudrate;
  bool device_setup = false;
  bool readback_settings = true;
  bool save_settings = true;
  bool auto_init = true;
  u8 auto_init_u8 = 1;
  u8 readback_headingsource = 0;
  u8 readback_auto_init = 0;
  int declination_source;
  u8 declination_source_u8;
  u8 readback_declination_source;
  double declination;

  // Variables
  base_device_info_field device_info;
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
  u8 gps_source = 0;
  u8 heading_source = 0x1;
  mip_low_pass_filter_settings filter_settings;
  u16 duration = 0;
  mip_filter_external_gps_update_command external_gps_update;
  mip_filter_external_heading_update_command external_heading_update;
  mip_filter_external_heading_with_time_command external_heading_with_time;

  com_mode = 0;

  // Device model flags
  GX5_15 = false;
  GX5_25 = false;
  GX5_35 = false;
  GX5_45 = false;

  // ROS setup
  ros::Time::init();
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // ROS Parameters
  // Comms Parameters
  std::string port;
  int baud, pdyn_mode;
  private_nh.param("port", port, std::string("/dev/ttyACM1"));
  private_nh.param("baudrate", baud, 115200);
  baudrate = (u32)baud;
  // Configuration Parameters
  private_nh.param("device_setup", device_setup, false);
  private_nh.param("readback_settings", readback_settings, true);
  private_nh.param("save_settings", save_settings, true);

  private_nh.param("auto_init", auto_init, true);
  private_nh.param("gps_rate", gps_rate_, 1);
  private_nh.param("imu_rate", imu_rate_, 10);
  private_nh.param("nav_rate", nav_rate_, 10);
  private_nh.param("dynamics_mode", pdyn_mode, 1);

  dynamics_mode = (u8)pdyn_mode;
  if (dynamics_mode < 1 || dynamics_mode > 3) {
    ROS_WARN("dynamics_mode can't be %#04X, must be 1, 2 or 3.  Setting to 1.", dynamics_mode);
    dynamics_mode = 1;
  }

  private_nh.param("declination_source", declination_source, 2);
  if (declination_source < 1 || declination_source > 3) {
    ROS_WARN("declination_source can't be %#04X, must be 1, 2 or 3.  Setting to 2.", declination_source);
    declination_source = 2;
  }
  declination_source_u8 = (u8)declination_source;

  // declination_source_command=(u8)declination_source;
  private_nh.param("declination", declination, 0.23);
  private_nh.param("gps_frame_id", gps_frame_id_, std::string("wgs84"));
  private_nh.param("imu_frame_id", imu_frame_id_, std::string("base_link"));
  private_nh.param("odom_frame_id", odom_frame_id_, std::string("wgs84"));
  private_nh.param("odom_child_frame_id", odom_child_frame_id_, std::string("base_link"));

  private_nh.param("publish_imu", publish_imu_, true);
  private_nh.param("publish_bias", publish_bias_, true);
  private_nh.param("publish_filtered_imu", publish_filtered_imu_, false);
  private_nh.param("remove_imu_gravity", remove_imu_gravity_, false);
  private_nh.param("frame_based_enu", frame_based_enu_, false);

  // Covariance parameters to set the sensor_msg/IMU covariance values
  std::vector<double> default_cov(9, 0.0);
  private_nh.param("imu_orientation_cov", imu_orientation_cov_, default_cov);
  private_nh.param("imu_linear_cov", imu_linear_cov_, default_cov);
  private_nh.param("imu_angular_cov", imu_angular_cov_, default_cov);

  // ROS publishers and subscribers
  if (publish_imu_) imu_pub_ = node.advertise<sensor_msgs::Imu>("imu/data", 100);
  if (publish_filtered_imu_) filtered_imu_pub_ = node.advertise<sensor_msgs::Imu>("filtered/imu/data", 100);

  // Publishes device status
  device_status_pub_ = node.advertise<microstrain_mips::status_msg>("device/status", 100);

  // Services to set/get device functions
  ros::ServiceServer reset_filter = node.advertiseService("reset_kf", &Microstrain::reset_callback, this);
  ros::ServiceServer device_report_service = node.advertiseService("device_report", &Microstrain::device_report, this);
  ros::ServiceServer gyro_bias_capture_service =
      node.advertiseService("gyro_bias_capture", &Microstrain::gyro_bias_capture, this);
  ros::ServiceServer set_soft_iron_matrix_service =
      node.advertiseService("set_soft_iron_matrix", &Microstrain::set_soft_iron_matrix, this);
  ros::ServiceServer set_complementary_filter_service =
      node.advertiseService("set_complementary_filter", &Microstrain::set_complementary_filter, this);
  ros::ServiceServer set_filter_euler_service =
      node.advertiseService("set_filter_euler", &Microstrain::set_filter_euler, this);
  ros::ServiceServer set_filter_heading_service =
      node.advertiseService("set_filter_heading", &Microstrain::set_filter_heading, this);
  ros::ServiceServer set_accel_bias_model_service =
      node.advertiseService("set_accel_bias_model", &Microstrain::set_accel_bias_model, this);
  ros::ServiceServer set_accel_adaptive_vals_service =
      node.advertiseService("set_accel_adaptive_vals", &Microstrain::set_accel_adaptive_vals, this);
  ros::ServiceServer set_sensor_vehicle_frame_trans_service =
      node.advertiseService("set_sensor_vehicle_frame_trans", &Microstrain::set_sensor_vehicle_frame_trans, this);
  ros::ServiceServer set_sensor_vehicle_frame_offset_service =
      node.advertiseService("set_sensor_vehicle_frame_offset", &Microstrain::set_sensor_vehicle_frame_offset, this);
  ros::ServiceServer set_accel_bias_service =
      node.advertiseService("set_accel_bias", &Microstrain::set_accel_bias, this);
  ros::ServiceServer set_gyro_bias_service = node.advertiseService("set_gyro_bias", &Microstrain::set_gyro_bias, this);
  ros::ServiceServer set_hard_iron_values_service =
      node.advertiseService("set_hard_iron_values", &Microstrain::set_hard_iron_values, this);
  ros::ServiceServer get_accel_bias_service =
      node.advertiseService("get_accel_bias", &Microstrain::get_accel_bias, this);
  ros::ServiceServer get_gyro_bias_service = node.advertiseService("get_gyro_bias", &Microstrain::get_gyro_bias, this);
  ros::ServiceServer get_hard_iron_values_service =
      node.advertiseService("get_hard_iron_values", &Microstrain::get_hard_iron_values, this);
  ros::ServiceServer get_soft_iron_matrix_service =
      node.advertiseService("get_soft_iron_matrix", &Microstrain::get_soft_iron_matrix, this);
  ros::ServiceServer get_sensor_vehicle_frame_trans_service =
      node.advertiseService("get_sensor_vehicle_frame_trans", &Microstrain::get_sensor_vehicle_frame_trans, this);
  ros::ServiceServer get_complementary_filter_service =
      node.advertiseService("get_complementary_filter", &Microstrain::get_complementary_filter, this);
  ros::ServiceServer set_reference_position_service =
      node.advertiseService("set_reference_position", &Microstrain::set_reference_position, this);
  ros::ServiceServer get_reference_position_service =
      node.advertiseService("get_reference_position", &Microstrain::get_reference_position, this);
  ros::ServiceServer set_coning_sculling_comp_service =
      node.advertiseService("set_coning_sculling_comp", &Microstrain::set_coning_sculling_comp, this);
  ros::ServiceServer get_coning_sculling_comp_service =
      node.advertiseService("get_coning_sculling_comp", &Microstrain::get_coning_sculling_comp, this);
  ros::ServiceServer set_estimation_control_flags_service =
      node.advertiseService("set_estimation_control_flags", &Microstrain::set_estimation_control_flags, this);
  ros::ServiceServer get_estimation_control_flags_service =
      node.advertiseService("get_estimation_control_flags", &Microstrain::get_estimation_control_flags, this);
  ros::ServiceServer set_dynamics_mode_service =
      node.advertiseService("set_dynamics_mode", &Microstrain::set_dynamics_mode, this);
  ros::ServiceServer get_basic_status_service =
      node.advertiseService("get_basic_status", &Microstrain::get_basic_status, this);
  ros::ServiceServer get_diagnostic_report_service =
      node.advertiseService("get_diagnostic_report", &Microstrain::get_diagnostic_report, this);
  ros::ServiceServer set_zero_angle_update_threshold_service =
      node.advertiseService("set_zero_angle_update_threshold", &Microstrain::set_zero_angle_update_threshold, this);
  ros::ServiceServer get_zero_angle_update_threshold_service =
      node.advertiseService("get_zero_angle_update_threshold", &Microstrain::get_zero_angle_update_threshold, this);
  ros::ServiceServer set_tare_orientation_service =
      node.advertiseService("set_tare_orientation", &Microstrain::set_tare_orientation, this);
  ros::ServiceServer set_accel_noise_service =
      node.advertiseService("set_accel_noise", &Microstrain::set_accel_noise, this);
  ros::ServiceServer get_accel_noise_service =
      node.advertiseService("get_accel_noise", &Microstrain::get_accel_noise, this);
  ros::ServiceServer set_gyro_noise_service =
      node.advertiseService("set_gyro_noise", &Microstrain::set_gyro_noise, this);
  ros::ServiceServer get_gyro_noise_service =
      node.advertiseService("get_gyro_noise", &Microstrain::get_gyro_noise, this);
  ros::ServiceServer set_mag_noise_service = node.advertiseService("set_mag_noise", &Microstrain::set_mag_noise, this);
  ros::ServiceServer get_mag_noise_service = node.advertiseService("get_mag_noise", &Microstrain::get_mag_noise, this);
  ros::ServiceServer set_gyro_bias_model_service =
      node.advertiseService("set_gyro_bias_model", &Microstrain::set_gyro_bias_model, this);
  ros::ServiceServer get_gyro_bias_model_service =
      node.advertiseService("get_gyro_bias_model", &Microstrain::get_gyro_bias_model, this);
  ros::ServiceServer get_accel_adaptive_vals_service =
      node.advertiseService("get_accel_adaptive_vals", &Microstrain::get_accel_adaptive_vals, this);
  ros::ServiceServer set_mag_adaptive_vals_service =
      node.advertiseService("set_mag_adaptive_vals", &Microstrain::set_mag_adaptive_vals, this);
  ros::ServiceServer get_mag_adaptive_vals_service =
      node.advertiseService("get_mag_adaptive_vals", &Microstrain::get_mag_adaptive_vals, this);
  ros::ServiceServer set_mag_dip_adaptive_vals_service =
      node.advertiseService("set_mag_dip_adaptive_vals", &Microstrain::set_mag_dip_adaptive_vals, this);
  ros::ServiceServer get_accel_bias_model_service =
      node.advertiseService("get_accel_bias_model", &Microstrain::get_accel_bias_model, this);
  ros::ServiceServer get_mag_dip_adaptive_vals_service =
      node.advertiseService("get_mag_dip_adaptive_vals", &Microstrain::get_mag_dip_adaptive_vals, this);
  ros::ServiceServer get_sensor_vehicle_frame_offset_service =
      node.advertiseService("get_sensor_vehicle_frame_offset", &Microstrain::get_sensor_vehicle_frame_offset, this);
  ros::ServiceServer get_dynamics_mode_service =
      node.advertiseService("get_dynamics_mode", &Microstrain::get_dynamics_mode, this);

  // Initialize the serial interface to the device
  ROS_INFO("Attempting to open serial port <%s> at <%d> \n", port.c_str(), baudrate);
  if (mip_interface_init(port.c_str(), baudrate, &device_interface_, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
    ROS_FATAL("Couldn't open serial port!  Is it plugged in?");
  }

  // We want to get the default device info even if we don't setup the device
  // Get device info
  start = clock();
  while (mip_base_cmd_get_device_info(&device_interface_, &device_info) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_base_cmd_get_device_info function timed out.");
      break;
    }
  }

  // Get device model name
  memset(temp_string, 0, 20 * sizeof(char));
  memcpy(temp_string, device_info.model_name, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  ROS_INFO("Model Name  => %s\n", temp_string);
  std::string model_name;

  for (int i = 6; i < 20; i++) {
    model_name += temp_string[i];
  }

  // Set device model flag
  model_name = model_name.c_str();
  if (model_name == GX5_45_DEVICE) {
    GX5_45 = true;
  }
  if (model_name == GX5_35_DEVICE) {
    GX5_35 = true;
  }
  if (model_name == GX5_25_DEVICE) {
    GX5_25 = true;
  }
  if (model_name == GX5_15_DEVICE) {
    GX5_15 = true;
  }

  ////////////////////////////////////////
  // Device setup
  float dT = 0.5;  // common sleep time after setup communications
  if (device_setup) {
    // Put device into standard mode - we never really use "direct mode"
    ROS_INFO("Putting device communications into 'standard mode'");
    device_descriptors_size = 128 * 2;
    com_mode = MIP_SDK_GX4_45_IMU_STANDARD_MODE;
    start = clock();
    while (mip_system_com_mode(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_system_com_mode function timed out.");
        break;
      }
    }

    // Verify device mode setting
    ROS_INFO("Verify comm's mode");
    start = clock();
    while (mip_system_com_mode(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_system_com_mode function timed out.");
        break;
      }
    }

    ROS_INFO("Sleep for a second...");
    ros::Duration(dT).sleep();
    ROS_INFO("Right mode?");
    if (com_mode != MIP_SDK_GX4_45_IMU_STANDARD_MODE) {
      ROS_ERROR("Appears we didn't get into standard mode!");
    }

    // Set GPS publishing to true if IMU model has GPS
    if (GX5_45 || GX5_35) {
      private_nh.param("publish_gps", publish_gps_, true);
      private_nh.param("publish_odom", publish_odom_, true);
    } else {
      private_nh.param("publish_gps", publish_gps_, false);
      private_nh.param("publish_odom", publish_odom_, false);
    }

    if (publish_gps_) {
      gps_pub_ = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 100);
    }

    if (publish_odom_) {
      nav_pub_ = node.advertise<nav_msgs::Odometry>("nav/odom", 100);
    }

    // This is the EKF filter status, not just navigation/odom status
    if (publish_odom_ || publish_filtered_imu_) {
      nav_status_pub_ = node.advertise<std_msgs::Int16MultiArray>("nav/status", 100);
    }

    // Setup device callbacks
    if (mip_interface_add_descriptor_set_callback(&device_interface_, MIP_FILTER_DATA_SET, this,
                                                  &filter_packet_callback_wrapper) != MIP_INTERFACE_OK) {
      ROS_FATAL("Can't setup filter callback!");
      return;
    }

    if (mip_interface_add_descriptor_set_callback(&device_interface_, MIP_AHRS_DATA_SET, this,
                                                  &ahrs_packet_callback_wrapper) != MIP_INTERFACE_OK) {
      ROS_FATAL("Can't setup ahrs callbacks!");
      return;
    }

    if (mip_interface_add_descriptor_set_callback(&device_interface_, MIP_GPS_DATA_SET, this,
                                                  &gps_packet_callback_wrapper) != MIP_INTERFACE_OK) {
      ROS_FATAL("Can't setup gpscallbacks!");
      return;
    }

    // Put into idle mode
    ROS_INFO("Idling Device: Stopping data streams and/or waking from sleep");
    start = clock();
    while (mip_base_cmd_idle(&device_interface_) != MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_base_cmd_idle function timed out.");
        break;
      }
    }
    ros::Duration(dT).sleep();

    // AHRS Setup
    // Get base rate
    if (publish_imu_ || publish_filtered_imu_) {
      start = clock();
      while (mip_3dm_cmd_get_ahrs_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_get_ahrs_base_rate function timed out.");
          break;
        }
      }

      ROS_INFO("AHRS Base Rate => %d Hz", base_rate);
      ros::Duration(dT).sleep();
      // Deterimine decimation to get close to goal rate (We use the highest of the imu rates)
      int rate = (imu_rate_ > nav_rate_) ? imu_rate_ : nav_rate_;
      u8 imu_decimation = (u8)(static_cast<float>(base_rate) / static_cast<float>(rate));
      ROS_INFO("AHRS decimation set to %#04X", imu_decimation);

      // AHRS Message Format
      // Set message format
      ROS_INFO("Setting the AHRS message format");
      data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
      data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
      data_stream_format_descriptors[2] = MIP_AHRS_DATA_QUATERNION;
      data_stream_format_decimation[0] = imu_decimation;  // 0x32;
      data_stream_format_decimation[1] = imu_decimation;  // 0x32;
      data_stream_format_decimation[2] = imu_decimation;  // 0x32;
      data_stream_format_num_entries = 3;

      start = clock();
      while (mip_3dm_cmd_ahrs_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                             &data_stream_format_num_entries, data_stream_format_descriptors,
                                             data_stream_format_decimation) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_ahrs_message_format function timed out.");
          break;
        }
      }

      ros::Duration(dT).sleep();
      // Poll to verify
      ROS_INFO("Poll AHRS data to verify");
      start = clock();
      while (mip_3dm_cmd_poll_ahrs(&device_interface_, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries,
                                   data_stream_format_descriptors) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_poll_ahrs function timed out.");
          break;
        }
      }

      ros::Duration(dT).sleep();
      // Save
      if (save_settings) {
        ROS_INFO("Saving AHRS data settings");
        start = clock();
        while (mip_3dm_cmd_ahrs_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL, NULL) !=
               MIP_INTERFACE_OK) {
          if (clock() - start > 5000) {
            ROS_INFO("mip_3dm_cmd_ahrs_message_format function timed out.");
            break;
          }
        }
        ros::Duration(dT).sleep();
      }

      // Declination Source
      // Set declination
      ROS_INFO("Setting declination source to %#04X", declination_source_u8);
      start = clock();
      while (mip_filter_declination_source(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &declination_source_u8) !=
             MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_filter_declination_source function timed out.");
          break;
        }
      }

      ros::Duration(dT).sleep();
      // Read back the declination source
      ROS_INFO("Reading back declination source");
      start = clock();
      while (mip_filter_declination_source(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                           &readback_declination_source) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_filter_declination_source function timed out.");
          break;
        }
      }

      if (declination_source_u8 == readback_declination_source) {
        ROS_INFO("Success: Declination source set to %#04X", declination_source_u8);
      } else {
        ROS_WARN("Failed to set the declination source to %#04X!", declination_source_u8);
      }

      ros::Duration(dT).sleep();
      if (save_settings) {
        ROS_INFO("Saving declination source settings to EEPROM");
        start = clock();
        while (mip_filter_declination_source(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, NULL) !=
               MIP_INTERFACE_OK) {
          if (clock() - start > 5000) {
            ROS_INFO("mip_filter_declination_source function timed out.");
            break;
          }
        }

        ros::Duration(dT).sleep();
      }
    }  // end of AHRS setup

    // GPS Setup
    if (publish_gps_) {
      start = clock();
      while (mip_3dm_cmd_get_gps_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_get_gps_base_rate function timed out.");
          break;
        }
      }

      ROS_INFO("GPS Base Rate => %d Hz", base_rate);
      u8 gps_decimation = (u8)(static_cast<float>(base_rate) / static_cast<float>(gps_rate_));
      ros::Duration(dT).sleep();

      ////////// GPS Message Format
      // Set
      ROS_INFO("Setting GPS stream format");
      data_stream_format_descriptors[0] = MIP_GPS_DATA_LLH_POS;
      data_stream_format_descriptors[1] = MIP_GPS_DATA_NED_VELOCITY;
      data_stream_format_descriptors[2] = MIP_GPS_DATA_GPS_TIME;
      data_stream_format_decimation[0] = gps_decimation;  // 0x01; // 0x04;
      data_stream_format_decimation[1] = gps_decimation;  // 0x01; // 0x04;
      data_stream_format_decimation[2] = gps_decimation;  // 0x01; // 0x04;
      data_stream_format_num_entries = 3;
      start = clock();

      while (mip_3dm_cmd_gps_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                            &data_stream_format_num_entries, data_stream_format_descriptors,
                                            data_stream_format_decimation) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_gps_message_format function timed out.");
          break;
        }
      }

      ros::Duration(dT).sleep();
      // Save
      if (save_settings) {
        ROS_INFO("Saving GPS data settings");
        start = clock();
        while (mip_3dm_cmd_gps_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL, NULL) !=
               MIP_INTERFACE_OK) {
          if (clock() - start > 5000) {
            ROS_INFO("mip_3dm_cmd_gps_message_format function timed out.");
            break;
          }
        }
        ros::Duration(dT).sleep();
      }
    }  // end of GPS setup

    // Filter setup
    if (publish_odom_ || publish_filtered_imu_) {
      start = clock();
      while (mip_3dm_cmd_get_filter_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_get_filter_base_rate function timed out.");
          break;
        }
      }

      ROS_INFO("FILTER Base Rate => %d Hz", base_rate);
      // If we have made it this far in this statement, we know we want to publish filtered data
      // from the IMU. We need to make sure to set the data rate to the correct speed dependent
      // upon which filtered field we are after. Thus make sure we get the fastest data rate.
      int rate = nav_rate_;
      if (publish_filtered_imu_) {
        // Set filter rate based on max of filter topic rates
        rate = (imu_rate_ > nav_rate_) ? imu_rate_ : nav_rate_;
      }

      u8 nav_decimation = (u8)(static_cast<float>(base_rate) / static_cast<float>(rate));
      ros::Duration(dT).sleep();

      ////////// Filter Message Format
      // Set
      ROS_INFO("Setting Filter stream format");

      // Order doesn't matter since we parse them with a case statement below.
      // First start by loading the common values.
      data_stream_format_descriptors[0] = MIP_FILTER_DATA_ATT_QUATERNION;
      data_stream_format_descriptors[1] = MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER;
      data_stream_format_descriptors[2] = MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE;
      data_stream_format_descriptors[3] = MIP_FILTER_DATA_FILTER_STATUS;
      data_stream_format_decimation[0] = nav_decimation;  // 0x32;
      data_stream_format_decimation[1] = nav_decimation;  // 0x32;
      data_stream_format_decimation[2] = nav_decimation;  // 0x32;
      data_stream_format_decimation[3] = nav_decimation;  // 0x32;

      // If we want the odometry add that data
      if (publish_odom_ && publish_filtered_imu_) {
        // Size is up to 10 elements
        data_stream_format_descriptors[4] = MIP_FILTER_DATA_LLH_POS;
        data_stream_format_descriptors[5] = MIP_FILTER_DATA_NED_VEL;
        // data_stream_format_descriptors[2] = MIP_FILTER_DATA_ATT_EULER_ANGLES;
        data_stream_format_descriptors[6] = MIP_FILTER_DATA_POS_UNCERTAINTY;
        data_stream_format_descriptors[7] = MIP_FILTER_DATA_VEL_UNCERTAINTY;

        // The filter has one message that removes gravity and one that does not
        if (remove_imu_gravity_) {
          data_stream_format_descriptors[8] = MIP_FILTER_DATA_LINEAR_ACCELERATION;
        } else {
          data_stream_format_descriptors[8] = MIP_FILTER_DATA_COMPENSATED_ACCELERATION;
        }

        data_stream_format_decimation[4] = nav_decimation;  // 0x32;
        data_stream_format_decimation[5] = nav_decimation;  // 0x32;
        data_stream_format_decimation[6] = nav_decimation;  // 0x32;
        data_stream_format_decimation[7] = nav_decimation;  // 0x32;
        data_stream_format_decimation[8] = nav_decimation;  // 0x32;
        data_stream_format_num_entries = 9;
      } else if (publish_odom_ && !publish_filtered_imu_) {
        data_stream_format_descriptors[4] = MIP_FILTER_DATA_LLH_POS;
        data_stream_format_descriptors[5] = MIP_FILTER_DATA_NED_VEL;
        // data_stream_format_descriptors[2] = MIP_FILTER_DATA_ATT_EULER_ANGLES;
        data_stream_format_descriptors[6] = MIP_FILTER_DATA_POS_UNCERTAINTY;
        data_stream_format_descriptors[7] = MIP_FILTER_DATA_VEL_UNCERTAINTY;

        data_stream_format_decimation[4] = nav_decimation;  // 0x32;
        data_stream_format_decimation[5] = nav_decimation;  // 0x32;
        data_stream_format_decimation[6] = nav_decimation;  // 0x32;
        data_stream_format_decimation[7] = nav_decimation;  // 0x32;
        data_stream_format_num_entries = 8;
      } else {
        // The filter has one message that removes gravity and one that does not
        if (remove_imu_gravity_) {
          data_stream_format_descriptors[4] = MIP_FILTER_DATA_LINEAR_ACCELERATION;
        } else {
          data_stream_format_descriptors[4] = MIP_FILTER_DATA_COMPENSATED_ACCELERATION;
        }
        data_stream_format_decimation[4] = nav_decimation;  // 0x32;
        data_stream_format_num_entries = 5;
      }

      start = clock();
      while (mip_3dm_cmd_filter_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                               &data_stream_format_num_entries, data_stream_format_descriptors,
                                               data_stream_format_decimation) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_filter_message_format function timed out.");
          break;
        }
      }

      ros::Duration(dT).sleep();
      // Poll to verify
      ROS_INFO("Poll filter data to test stream");
      start = clock();
      while (mip_3dm_cmd_poll_filter(&device_interface_, MIP_3DM_POLLING_ENABLE_ACK_NACK,
                                     data_stream_format_num_entries,
                                     data_stream_format_descriptors) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_poll_filter function timed out.");
          break;
        }
      }

      ros::Duration(dT).sleep();
      // Save
      if (save_settings) {
        ROS_INFO("Saving Filter data settings");
        start = clock();
        while (mip_3dm_cmd_filter_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL,
                                                 NULL) != MIP_INTERFACE_OK) {
          if (clock() - start > 5000) {
            ROS_INFO("mip_3dm_cmd_filter_message_format function timed out.");
            break;
          }
        }
        ros::Duration(dT).sleep();
      }

      // GX5_25 doesn't appear to suport this feature thus GX5_15 probably won't either
      if (GX5_35 == true || GX5_45 == true) {
        // Dynamics Mode
        // Set dynamics mode
        ROS_INFO("Setting dynamics mode to %#04X", dynamics_mode);
        start = clock();
        while (mip_filter_vehicle_dynamics_mode(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &dynamics_mode) !=
               MIP_INTERFACE_OK) {
          if (clock() - start > 5000) {
            ROS_INFO("mip_filter_vehicle_dynamics_mode function timed out.");
            break;
          }
        }

        ros::Duration(dT).sleep();
        // Readback dynamics mode
        if (readback_settings) {
          // Read the settings back
          ROS_INFO("Reading back dynamics mode setting");
          start = clock();
          while (mip_filter_vehicle_dynamics_mode(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                  &readback_dynamics_mode) != MIP_INTERFACE_OK) {
            if (clock() - start > 5000) {
              ROS_INFO("mip_filter_vehicle_dynamics_mode function timed out.");
              break;
            }
          }

          ros::Duration(dT).sleep();
          if (dynamics_mode == readback_dynamics_mode)
            ROS_INFO("Success: Dynamics mode setting is: %#04X", readback_dynamics_mode);
          else
            ROS_ERROR("Failure: Dynamics mode set to be %#04X, but reads as %#04X", dynamics_mode,
                      readback_dynamics_mode);
        }

        if (save_settings) {
          ROS_INFO("Saving dynamics mode settings to EEPROM");
          start = clock();
          while (mip_filter_vehicle_dynamics_mode(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, NULL) !=
                 MIP_INTERFACE_OK) {
            if (clock() - start > 5000) {
              ROS_INFO("mip_filter_vehicle_dynamics_mode function timed out.");
              break;
            }
          }
          ros::Duration(dT).sleep();
        }
      }

      // Set heading Source
      ROS_INFO("Set heading source to internal mag.");
      heading_source = 0x1;
      ROS_INFO("Setting heading source to %#04X", heading_source);
      start = clock();
      while (mip_filter_heading_source(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &heading_source) !=
             MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_filter_heading_source function timed out.");
          break;
        }
      }
      // Read back heading source
      ros::Duration(dT).sleep();
      ROS_INFO("Read back heading source...");
      start = clock();
      while (mip_filter_heading_source(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &readback_headingsource) !=
             MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_filter_heading_source function timed out.");
          break;
        }
      }

      ROS_INFO("Heading source = %#04X", readback_headingsource);
      ros::Duration(dT).sleep();

      if (save_settings) {
        ROS_INFO("Saving heading source to EEPROM");
        start = clock();
        while (mip_filter_heading_source(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, NULL) !=
               MIP_INTERFACE_OK) {
          if (clock() - start > 5000) {
            ROS_INFO("mip_filter_heading_source function timed out.");
            break;
          }
        }
        ros::Duration(dT).sleep();
      }
    }  // end of Filter setup

    // Set auto-initialization based on ROS parameter
    ROS_INFO("Setting auto-initinitalization to: %#04X", auto_init);
    auto_init_u8 = auto_init;  // convert bool to u8
    start = clock();
    while (mip_filter_auto_initialization(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &auto_init_u8) !=
           MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_filter_auto_initialization function timed out.");
        break;
      }
    }
    ros::Duration(dT).sleep();

    if (readback_settings) {
      // Read the settings back
      ROS_INFO("Reading back auto-initialization value");
      start = clock();
      while (mip_filter_auto_initialization(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &readback_auto_init) !=
             MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_filter_auto_initialization function timed out.");
          break;
        }
      }

      ros::Duration(dT).sleep();
      if (auto_init == readback_auto_init)
        ROS_INFO("Success: Auto init. setting is: %#04X", readback_auto_init);
      else
        ROS_ERROR("Failure: Auto init. setting set to be %#04X, but reads as %#04X", auto_init, readback_auto_init);
    }

    if (save_settings) {
      ROS_INFO("Saving auto init. settings to EEPROM");
      start = clock();
      while (mip_filter_auto_initialization(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, NULL) !=
             MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_filter_auto_initialization function timed out.");
          break;
        }
      }
      ros::Duration(dT).sleep();
    }

    // Enable Data streams
    dT = 0.25;
    if (publish_imu_ || publish_filtered_imu_) {
      ROS_INFO("Enabling AHRS stream");
      enable = 0x01;
      start = clock();
      while (mip_3dm_cmd_continuous_data_stream(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                                MIP_3DM_AHRS_DATASTREAM, &enable) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_continuous_data_stream function timed out.");
          break;
        }
      }
      ros::Duration(dT).sleep();
    }

    if (publish_odom_) {
      ROS_INFO("Enabling Filter stream");
      enable = 0x01;
      start = clock();
      while (mip_3dm_cmd_continuous_data_stream(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM,
                                                &enable) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_continuous_data_stream function timed out.");
          break;
        }
      }
      ros::Duration(dT).sleep();
    }

    if (publish_gps_) {
      ROS_INFO("Enabling GPS stream");
      enable = 0x01;
      start = clock();
      while (mip_3dm_cmd_continuous_data_stream(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_GPS_DATASTREAM,
                                                &enable) != MIP_INTERFACE_OK) {
        if (clock() - start > 5000) {
          ROS_INFO("mip_3dm_cmd_continuous_data_stream function timed out.");
          break;
        }
      }
      ros::Duration(dT).sleep();
    }

    ROS_INFO("End of device setup - starting streaming");
  } else {
    ROS_INFO("Skipping device setup and listing for existing streams");
  }  // end of device_setup

  // Reset filter - should be for either the KF or CF
  ROS_INFO("Reset filter");
  start = clock();
  while (mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_reset_filter function timed out.");
      break;
    }
  }
  ros::Duration(dT).sleep();

  // Loop
  // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
  int max_rate = 1;
  if (publish_imu_) {
    max_rate = std::max(max_rate, imu_rate_);
  }
  if (publish_filtered_imu_) {
    max_rate = std::max(std::max(max_rate, nav_rate_), imu_rate_);
  }
  if (publish_gps_) {
    max_rate = std::max(max_rate, gps_rate_);
  }
  if (publish_odom_) {
    max_rate = std::max(max_rate, nav_rate_);
  }
  int spin_rate = std::min(3 * max_rate, 1000);
  ROS_INFO("Setting spin rate to <%d>", spin_rate);
  ros::Rate r(spin_rate);  // Rate in Hz

  microstrain_mips::RosDiagnosticUpdater ros_diagnostic_updater(this);

  while (ros::ok()) {
    // Update the parser (this function reads the port and parses the bytes
    mip_interface_update(&device_interface_);

    if (GX5_25) {
      device_status_callback();
    }

    ros::spinOnce();  // take care of service requests.
    r.sleep();
  }  // end loop

  // close serial port
  mip_sdk_port_close(device_interface_.port_handle);
}  // End of ::run()

bool Microstrain::reset_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  ROS_INFO("Reseting the filter");
  start = clock();
  while (mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_reset_filter function timed out.");
      break;
    }
  }
  return true;
}

// Services to get/set values on devices
// Set accel bias values
bool Microstrain::set_accel_bias(microstrain_mips::SetAccelBias::Request& req,
                                 microstrain_mips::SetAccelBias::Response& res) {
  ROS_INFO("Setting accel bias values");
  memset(field_data, 0, 3 * sizeof(float));
  start = clock();
  while (mip_3dm_cmd_accel_bias(&device_interface_, MIP_FUNCTION_SELECTOR_READ, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_accel_bias function timed out.");
      break;
    }
  }

  ROS_INFO("Accel bias vector values are: %f %f %f", field_data[0], field_data[1], field_data[2]);
  ROS_INFO("Client request values are: %.2f %.2f %.2f", req.bias.x, req.bias.y, req.bias.z);

  field_data[0] = req.bias.x;
  field_data[1] = req.bias.y;
  field_data[2] = req.bias.z;

  start = clock();
  while (mip_3dm_cmd_accel_bias(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_accel_bias function timed out.");
      break;
    }
  }

  memset(field_data, 0, 3 * sizeof(float));
  start = clock();
  while (mip_3dm_cmd_accel_bias(&device_interface_, MIP_FUNCTION_SELECTOR_READ, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_accel_bias function timed out.");
      break;
    }
  }
  ROS_INFO("New accel bias vector values are: %.2f %.2f %.2f", field_data[0], field_data[1], field_data[2]);

  res.success = true;
  return true;
}

// Get accel bias values
bool Microstrain::get_accel_bias(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("Getting accel bias values");
  memset(field_data, 0, 3 * sizeof(float));

  start = clock();
  while (mip_3dm_cmd_accel_bias(&device_interface_, MIP_FUNCTION_SELECTOR_READ, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_accel_bias function timed out.");
      break;
    }
  }
  ROS_INFO("Accel bias vector values are: %f %f %f", field_data[0], field_data[1], field_data[2]);

  res.success = true;
  return true;
}

// Set gyro bias values
bool Microstrain::set_gyro_bias(microstrain_mips::SetGyroBias::Request& req,
                                microstrain_mips::SetGyroBias::Response& res) {
  ROS_INFO("Setting gyro bias values");
  memset(field_data, 0, 3 * sizeof(float));

  start = clock();
  while (mip_3dm_cmd_gyro_bias(&device_interface_, MIP_FUNCTION_SELECTOR_READ, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_gyro_bias function timed out.");
      break;
    }
  }

  ROS_INFO("Gyro bias vector values are: %f %f %f", field_data[0], field_data[1], field_data[2]);
  ROS_INFO("Client request values are: %.2f %.2f %.2f", req.bias.x, req.bias.y, req.bias.z);

  field_data[0] = req.bias.x;
  field_data[1] = req.bias.y;
  field_data[2] = req.bias.z;

  start = clock();
  while (mip_3dm_cmd_gyro_bias(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_gyro_bias function timed out.");
      break;
    }
  }
  memset(field_data, 0, 3 * sizeof(float));
  start = clock();
  while (mip_3dm_cmd_gyro_bias(&device_interface_, MIP_FUNCTION_SELECTOR_READ, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_gyro_bias function timed out.");
      break;
    }
  }
  ROS_INFO("New gyro bias vector values are: %.2f %.2f %.2f", field_data[0], field_data[1], field_data[2]);

  res.success = true;
  return true;
}

// Get gyro bias values
bool Microstrain::get_gyro_bias(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("Getting gyro bias values");
  memset(field_data, 0, 3 * sizeof(float));

  start = clock();
  while (mip_3dm_cmd_gyro_bias(&device_interface_, MIP_FUNCTION_SELECTOR_READ, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_gyro_bias function timed out.");
      break;
    }
  }
  ROS_INFO("Gyro bias vector values are: %f %f %f", field_data[0], field_data[1], field_data[2]);

  res.success = true;
  return true;
}

// Set hard iron values
bool Microstrain::set_hard_iron_values(microstrain_mips::SetHardIronValues::Request& req,
                                       microstrain_mips::SetHardIronValues::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  ROS_INFO("Setting hard iron values");
  float field_data[3] = {0};

  start = clock();
  while (mip_3dm_cmd_hard_iron(&device_interface_, MIP_FUNCTION_SELECTOR_READ, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_hard_iron function timed out.");
      break;
    }
  }

  ROS_INFO("Hard iron values are: %f %f %f", field_data[0], field_data[1], field_data[2]);
  ROS_INFO("Client request values are: %.2f %.2f %.2f", req.bias.x, req.bias.y, req.bias.z);

  field_data[0] = req.bias.x;
  field_data[1] = req.bias.y;
  field_data[2] = req.bias.z;

  start = clock();
  while (mip_3dm_cmd_hard_iron(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_hard_iron function timed out.");
      break;
    }
  }

  memset(field_data, 0, 3 * sizeof(float));
  start = clock();
  while (mip_3dm_cmd_hard_iron(&device_interface_, MIP_FUNCTION_SELECTOR_READ, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_hard_iron function timed out.");
      break;
    }
  }
  ROS_INFO("New hard iron values are: %.2f %.2f %.2f", field_data[0], field_data[1], field_data[2]);

  res.success = true;
  return true;
}

// Get hard iron values
bool Microstrain::get_hard_iron_values(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  ROS_INFO("Getting hard iron values");
  memset(field_data, 0, 3 * sizeof(float));

  start = clock();
  while (mip_3dm_cmd_hard_iron(&device_interface_, MIP_FUNCTION_SELECTOR_READ, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_hard_iron function timed out.");
      break;
    }
  }
  ROS_INFO("Hard iron values are: %f %f %f", field_data[0], field_data[1], field_data[2]);

  res.success = true;
  return true;
}

// Get device report
bool Microstrain::device_report(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  start = clock();
  while (mip_base_cmd_get_device_info(&device_interface_, &device_info) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_base_cmd_get_device_info function timed out.");
      break;
    }
  }

  ROS_INFO("\n\nDevice Info:\n");

  memset(temp_string, 0, 20 * sizeof(int));

  memcpy(temp_string, device_info.model_name, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  ROS_INFO("Model Name       => %s\n", temp_string);

  memcpy(temp_string, device_info.model_number, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  ROS_INFO("Model Number     => %s\n", temp_string);

  memcpy(temp_string, device_info.serial_number, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  ROS_INFO("Serial Number    => %s\n", temp_string);

  memcpy(temp_string, device_info.lotnumber, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  ROS_INFO("Lot Number       => %s\n", temp_string);

  memcpy(temp_string, device_info.device_options, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
  ROS_INFO("Options          => %s\n", temp_string);

  ROS_INFO("Firmware Version => %d.%d.%.2d\n\n", (device_info.firmware_version) / 1000,
           (device_info.firmware_version) % 1000 / 100, (device_info.firmware_version) % 100);

  res.success = true;
  return true;
}

// Capture gyro bias values
bool Microstrain::gyro_bias_capture(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  memset(field_data, 0, 3 * sizeof(float));
  ROS_INFO(
      "Performing Gyro Bias capture.\nPlease keep device stationary during the 5 second gyro bias capture interval\n");
  duration = 5000;  // milliseconds
  start = clock();
  while (mip_3dm_cmd_capture_gyro_bias(&device_interface_, duration, field_data) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_capture_gyro_bias function timed out.");
      break;
    }
  }
  ROS_INFO("Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", field_data[0],
           field_data[1], field_data[2]);

  res.success = true;
  return true;
}

// Set soft iron matrix values
bool Microstrain::set_soft_iron_matrix(microstrain_mips::SetSoftIronMatrix::Request& req,
                                       microstrain_mips::SetSoftIronMatrix::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  memset(soft_iron, 0, 9 * sizeof(float));
  memset(soft_iron_readback, 0, 9 * sizeof(float));

  ROS_INFO("Setting the soft iron matrix values\n");

  soft_iron[0] = req.soft_iron_1.x;
  soft_iron[1] = req.soft_iron_1.y;
  soft_iron[2] = req.soft_iron_1.z;
  soft_iron[3] = req.soft_iron_2.x;
  soft_iron[4] = req.soft_iron_2.y;
  soft_iron[5] = req.soft_iron_2.z;
  soft_iron[6] = req.soft_iron_3.x;
  soft_iron[7] = req.soft_iron_3.y;
  soft_iron[8] = req.soft_iron_3.z;

  start = clock();
  while (mip_3dm_cmd_soft_iron(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, soft_iron) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_soft_iron function timed out.");
      break;
    }
  }

  // Read back the soft iron matrix values
  start = clock();
  while (mip_3dm_cmd_soft_iron(&device_interface_, MIP_FUNCTION_SELECTOR_READ, soft_iron_readback) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_soft_iron function timed out.");
      break;
    }
  }

  if ((abs(soft_iron_readback[0] - soft_iron[0]) < 0.001) && (abs(soft_iron_readback[1] - soft_iron[1]) < 0.001) &&
      (abs(soft_iron_readback[2] - soft_iron[2]) < 0.001) && (abs(soft_iron_readback[3] - soft_iron[3]) < 0.001) &&
      (abs(soft_iron_readback[4] - soft_iron[4]) < 0.001) && (abs(soft_iron_readback[5] - soft_iron[5]) < 0.001) &&
      (abs(soft_iron_readback[6] - soft_iron[6]) < 0.001) && (abs(soft_iron_readback[7] - soft_iron[7]) < 0.001) &&
      (abs(soft_iron_readback[8] - soft_iron[8]) < 0.001)) {
    ROS_INFO("Soft iron matrix values successfully set.\n");
    ROS_INFO("Sent values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n", soft_iron[0], soft_iron[1], soft_iron[2],
             soft_iron[3], soft_iron[4], soft_iron[5], soft_iron[6], soft_iron[7], soft_iron[8]);
    ROS_INFO("Returned values: [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n", soft_iron_readback[0], soft_iron_readback[1],
             soft_iron_readback[2], soft_iron_readback[3], soft_iron_readback[4], soft_iron_readback[5],
             soft_iron_readback[6], soft_iron_readback[7], soft_iron_readback[8]);
  } else {
    ROS_INFO("ERROR: Failed to set hard iron values!!!\n");
    ROS_INFO("Sent values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n", soft_iron[0], soft_iron[1], soft_iron[2],
             soft_iron[3], soft_iron[4], soft_iron[5], soft_iron[6], soft_iron[7], soft_iron[8]);
    ROS_INFO("Returned values: [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n", soft_iron_readback[0], soft_iron_readback[1],
             soft_iron_readback[2], soft_iron_readback[3], soft_iron_readback[4], soft_iron_readback[5],
             soft_iron_readback[6], soft_iron_readback[7], soft_iron_readback[8]);
  }
  res.success = true;
  return true;
}

// Get soft iron matrix values
bool Microstrain::get_soft_iron_matrix(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  memset(soft_iron, 0, 9 * sizeof(float));
  memset(soft_iron_readback, 0, 9 * sizeof(float));

  ROS_INFO("Getting the soft iron matrix values\n");

  start = clock();
  while (mip_3dm_cmd_soft_iron(&device_interface_, MIP_FUNCTION_SELECTOR_READ, soft_iron_readback) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_soft_iron function timed out.");
      break;
    }
  }

  ROS_INFO("Soft iron matrix values: [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n", soft_iron_readback[0],
           soft_iron_readback[1], soft_iron_readback[2], soft_iron_readback[3], soft_iron_readback[4],
           soft_iron_readback[5], soft_iron_readback[6], soft_iron_readback[7], soft_iron_readback[8]);
  res.success = true;
  return true;
}

// Set complementary filter values
bool Microstrain::set_complementary_filter(microstrain_mips::SetComplementaryFilter::Request& req,
                                           microstrain_mips::SetComplementaryFilter::Response& res) {
  ROS_INFO("Setting the complementary filter values\n");

  comp_filter_command.north_compensation_enable = req.north_comp_enable;
  comp_filter_command.up_compensation_enable = req.up_comp_enable;
  comp_filter_command.north_compensation_time_constant = req.north_comp_time_const;
  comp_filter_command.up_compensation_time_constant = req.up_comp_time_const;

  start = clock();
  while (mip_3dm_cmd_complementary_filter_settings(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                                   &comp_filter_command) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_complementary_filter_settings function timed out.");
      break;
    }
  }

  // Read back the complementary filter values
  start = clock();
  while (mip_3dm_cmd_complementary_filter_settings(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                   &comp_filter_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_complementary_filter_settings function timed out.");
      break;
    }
  }

  if ((comp_filter_command.north_compensation_enable == comp_filter_readback.north_compensation_enable) &&
      (comp_filter_command.up_compensation_enable == comp_filter_readback.up_compensation_enable) &&
      (abs(comp_filter_command.north_compensation_time_constant -
           comp_filter_readback.north_compensation_time_constant) < 0.001) &&
      (abs(comp_filter_command.up_compensation_time_constant - comp_filter_readback.up_compensation_time_constant) <
       0.001)) {
    ROS_INFO("Complementary filter values successfully set.\n");
    ROS_INFO("Sent values:     Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
             comp_filter_command.up_compensation_enable, comp_filter_command.north_compensation_enable,
             comp_filter_command.up_compensation_time_constant, comp_filter_command.north_compensation_time_constant);
    ROS_INFO("Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
             comp_filter_readback.up_compensation_enable, comp_filter_readback.north_compensation_enable,
             comp_filter_readback.up_compensation_time_constant, comp_filter_readback.north_compensation_time_constant);
  } else {
    ROS_INFO("ERROR: Failed to set complementary filter values!!!\n");
    ROS_INFO("Sent values:     Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
             comp_filter_command.up_compensation_enable, comp_filter_command.north_compensation_enable,
             comp_filter_command.up_compensation_time_constant, comp_filter_command.north_compensation_time_constant);
    ROS_INFO("Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
             comp_filter_readback.up_compensation_enable, comp_filter_readback.north_compensation_enable,
             comp_filter_readback.up_compensation_time_constant, comp_filter_readback.north_compensation_time_constant);
  }
  res.success = true;
  return true;
}

// Get complementary filter values
bool Microstrain::get_complementary_filter(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // Read back the complementary filter values
  start = clock();
  while (mip_3dm_cmd_complementary_filter_settings(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                   &comp_filter_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_complementary_filter_settings function timed out.");
      break;
    }
  }
  ROS_INFO("Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
           comp_filter_readback.up_compensation_enable, comp_filter_readback.north_compensation_enable,
           comp_filter_readback.up_compensation_time_constant, comp_filter_readback.north_compensation_time_constant);
  res.success = true;
  return true;
}

// Initialize filter with Euler angles
bool Microstrain::set_filter_euler(microstrain_mips::SetFilterEuler::Request& req,
                                   microstrain_mips::SetFilterEuler::Response& res) {
  memset(angles, 0, 3 * sizeof(float));
  ROS_INFO("Resetting the Filter\n");

  start = clock();
  while (mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_reset_filter function timed out.");
      break;
    }
  }

  ROS_INFO("Initializing the Filter with Euler angles\n");

  angles[0] = req.angle.x;
  angles[1] = req.angle.y;
  angles[2] = req.angle.z;

  start = clock();
  while (mip_filter_set_init_attitude(&device_interface_, angles) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_set_init_attitude function timed out.");
      break;
    }
  }

  res.success = true;
  return true;
}

// Set filter with heading angle
bool Microstrain::set_filter_heading(microstrain_mips::SetFilterHeading::Request& req,
                                     microstrain_mips::SetFilterHeading::Response& res) {
  ROS_INFO("Resetting the Filter\n");

  start = clock();
  while (mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_reset_filter function timed out.");
      break;
    }
  }

  ROS_INFO("Initializing the Filter with a heading angle\n");

  heading_angle = req.angle;
  start = clock();
  while (mip_filter_set_init_heading(&device_interface_, heading_angle) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_set_init_heading function timed out.");
      break;
    }
  }

  res.success = true;
  return true;
}

// Set sensor to vehicle frame transformation
bool Microstrain::set_sensor_vehicle_frame_trans(microstrain_mips::SetSensorVehicleFrameTrans::Request& req,
                                                 microstrain_mips::SetSensorVehicleFrameTrans::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  memset(angles, 0, 3 * sizeof(float));
  memset(readback_angles, 0, 3 * sizeof(float));

  ROS_INFO("Setting the sensor to vehicle frame transformation\n");

  angles[0] = req.angle.x;
  angles[1] = req.angle.y;
  angles[2] = req.angle.z;

  start = clock();
  while (mip_filter_sensor2vehicle_tranformation(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, angles) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_sensor2vehicle_tranformation function timed out.");
      break;
    }
  }

  // Read back the transformation
  start = clock();
  while (mip_filter_sensor2vehicle_tranformation(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_angles) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_sensor2vehicle_tranformation function timed out.");
      break;
    }
  }

  if ((abs(readback_angles[0] - angles[0]) < 0.001) && (abs(readback_angles[1] - angles[1]) < 0.001) &&
      (abs(readback_angles[2] - angles[2]) < 0.001)) {
    ROS_INFO("Transformation successfully set.\n");
    ROS_INFO("New angles: %f roll %f pitch %f yaw\n", readback_angles[0], readback_angles[1], readback_angles[2]);
  } else {
    ROS_INFO("ERROR: Failed to set transformation!!!\n");
    ROS_INFO("Sent angles:     %f roll %f pitch %f yaw\n", angles[0], angles[1], angles[2]);
    ROS_INFO("Returned angles: %f roll %f pitch %f yaw\n", readback_angles[0], readback_angles[1], readback_angles[2]);
  }
  res.success = true;
  return true;
}

// Get sensor to vehicle frame transformation
bool Microstrain::get_sensor_vehicle_frame_trans(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  memset(readback_angles, 0, 3 * sizeof(float));
  // Read back the transformation
  start = clock();
  while (mip_filter_sensor2vehicle_tranformation(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_angles) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_sensor2vehicle_tranformation function timed out.");
      break;
    }
  }

  ROS_INFO("Sensor Vehicle Frame Transformation Angles: %f roll %f pitch %f yaw\n", readback_angles[0],
           readback_angles[1], readback_angles[2]);

  res.success = true;
  return true;
}

// Set reference position
bool Microstrain::set_reference_position(microstrain_mips::SetReferencePosition::Request& req,
                                         microstrain_mips::SetReferencePosition::Response& res) {
  ROS_INFO("Setting reference Position\n");

  memset(reference_position_command, 0, 3 * sizeof(double));
  memset(reference_position_readback, 0, 3 * sizeof(double));
  reference_position_enable_command = 1;
  reference_position_enable_readback = 1;

  reference_position_command[0] = req.position.x;
  reference_position_command[1] = req.position.y;
  reference_position_command[2] = req.position.z;

  start = clock();
  while (mip_filter_reference_position(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                       &reference_position_enable_command,
                                       reference_position_command) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_reference_position function timed out.");
      break;
    }
  }

  // Read back the reference position
  start = clock();
  while (mip_filter_reference_position(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                       &reference_position_enable_readback,
                                       reference_position_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_reference_position function timed out.");
      break;
    }
  }

  if ((reference_position_enable_command == reference_position_enable_readback) &&
      (abs(reference_position_command[0] - reference_position_readback[0]) < 0.001) &&
      (abs(reference_position_command[1] - reference_position_readback[1]) < 0.001) &&
      (abs(reference_position_command[2] - reference_position_readback[2]) < 0.001)) {
    ROS_INFO("Reference position successfully set\n");
  } else {
    ROS_ERROR("Failed to set the reference position!!!\n");
  }

  res.success = true;
  return true;
}

// Get reference position
bool Microstrain::get_reference_position(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("Getting reference position");
  memset(reference_position_readback, 0, 3 * sizeof(double));
  start = clock();
  while (mip_filter_reference_position(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                       &reference_position_enable_readback,
                                       reference_position_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_reference_position function timed out.");
      break;
    }
  }
  ROS_INFO("Reference position: Lat %f , Long %f, Alt %f", reference_position_readback[0],
           reference_position_readback[1], reference_position_readback[2]);

  res.success = true;
  return true;
}

// Enable or disable coning and sculling compensation
bool Microstrain::set_coning_sculling_comp(microstrain_mips::SetConingScullingComp::Request& req,
                                           microstrain_mips::SetConingScullingComp::Response& res) {
  if (req.enable == 0) {
    ROS_INFO("Disabling Coning and Sculling compensation\n");
    enable_flag = MIP_3DM_CONING_AND_SCULLING_DISABLE;
    start = clock();
    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &enable_flag) !=
           MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_3dm_cmd_coning_sculling_compensation function timed out.");
        break;
      }
    }

    ROS_INFO("Reading Coning and Sculling compensation enabled state:\n");
    start = clock();
    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &enable_flag) !=
           MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_3dm_cmd_coning_sculling_compensation function timed out.");
        break;
      }
    }
    ROS_INFO("%s\n\n", enable_flag == MIP_3DM_CONING_AND_SCULLING_DISABLE ? "DISABLED" : "ENABLED");
  } else if (req.enable == 1) {
    ROS_INFO("Enabling Coning and Sculling compensation\n");
    enable_flag = MIP_3DM_CONING_AND_SCULLING_ENABLE;
    start = clock();
    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &enable_flag) !=
           MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_3dm_cmd_coning_sculling_compensation function timed out.");
        break;
      }
    }

    // Read back enable/disable value
    ROS_INFO("Reading Coning and Sculling compensation enabled state:\n");
    start = clock();
    while (mip_3dm_cmd_coning_sculling_compensation(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &enable_flag) !=
           MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_3dm_cmd_coning_sculling_compensation function timed out.");
        break;
      }
    }
    ROS_INFO("%s\n\n", enable_flag == MIP_3DM_CONING_AND_SCULLING_DISABLE ? "DISABLED" : "ENABLED");
  } else {
    ROS_INFO("Error: Input must be either 0 (disable) or 1 (enable).");
  }

  res.success = false;
  return true;
}

// Get coning and sculling compenastion enabled/disabled state
bool Microstrain::get_coning_sculling_comp(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  start = clock();
  while (mip_3dm_cmd_coning_sculling_compensation(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &enable_flag) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_3dm_cmd_coning_sculling_compensation function timed out.");
      break;
    }
  }
  ROS_INFO("Coning and Sculling compensation is: %s\n\n",
           enable_flag == MIP_3DM_CONING_AND_SCULLING_DISABLE ? "DISABLED" : "ENABLED");
  res.success = true;
  return true;
}

// Set estimation control filter flags
bool Microstrain::set_estimation_control_flags(microstrain_mips::SetEstimationControlFlags::Request& req,
                                               microstrain_mips::SetEstimationControlFlags::Response& res) {
  estimation_control = req.flag;
  start = clock();
  while (mip_filter_estimation_control(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &estimation_control) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_estimation_control function timed out.");
      break;
    }
  }

  start = clock();
  while (mip_filter_estimation_control(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &estimation_control_readback) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_estimation_control function timed out.");
      break;
    }
  }
  ROS_INFO("Estimation control set to: %d", estimation_control_readback);

  res.success = true;
  return true;
}

// Get estimatio control filter flags
bool Microstrain::get_estimation_control_flags(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  start = clock();
  while (mip_filter_estimation_control(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &estimation_control_readback) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_estimation_control function timed out.");
      break;
    }
  }
  ROS_INFO("Estimation control set to: %d", estimation_control_readback);

  res.success = true;
  return true;
}

// Get device basic status. Variables in basic status struct change based on device model
bool Microstrain::get_basic_status(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // Use the basic status struct for the GX-25
  if (GX5_25) {
    u8 response_buffer[sizeof(gx4_25_basic_status_field)];
    start = clock();
    while (mip_3dm_cmd_hw_specific_device_status(&device_interface_, GX4_25_MODEL_NUMBER, GX4_25_BASIC_STATUS_SEL,
                                                 response_buffer) != MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_3dm_cmd_hw_specific_device_status function timed out.");
        break;
      }
    }

    ROS_INFO("Model Number: \t\t\t\t\t%04u\n", basic_field.device_model);
    ROS_INFO("Status Selector: \t\t\t\t%d\n", basic_field.status_selector);
    // == GX4_25_BASIC_STATUS_SEL ? "Basic Status Report" : "Diagnostic Status Report");
    ROS_INFO("Status Flags: \t\t\t\t\t%lu\n", basic_field.status_flags);
    ROS_INFO("System state: \t\t\t\t\t%04u\n", basic_field.system_state);
    ROS_INFO("System Microsecond Timer Count: \t\t%lu ms\n\n", basic_field.system_timer_ms);
  } else {
    ROS_INFO("Command not supported on this model");
  }

  res.success = true;
  return true;
}

// Get diagnostic status of device. Changes based on device model.
bool Microstrain::get_diagnostic_report(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // Use GX5-25 device diagnostic struct
  if (GX5_25) {
    u8 response_buffer[sizeof(gx4_25_diagnostic_device_status_field)];
    start = clock();
    while (mip_3dm_cmd_hw_specific_device_status(&device_interface_, GX4_25_MODEL_NUMBER, GX4_25_DIAGNOSTICS_STATUS_SEL,
                                                 response_buffer) != MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_3dm_cmd_hw_specific_device_status function timed out.");
        break;
      }
    }

    ROS_INFO("Model Number: \t\t\t\t\t%04u\n", diagnostic_field.device_model);
    ROS_INFO("Status Selector: \t\t\t\t%d\n", diagnostic_field.status_selector);
    // == 114 ? "Basic Status Report" : "Diagnostic Status Report");
    ROS_INFO("Status Flags: \t\t\t\t\t%lu\n", diagnostic_field.status_flags);
    ROS_INFO("System Millisecond Timer Count: \t\t%lu ms\n", diagnostic_field.system_timer_ms);
    ROS_INFO("IMU Streaming Enabled: \t\t\t\t%s\n", diagnostic_field.imu_stream_enabled == 1 ? "TRUE" : "FALSE");
    ROS_INFO("FILTER Streaming Enabled: \t\t\t%s\n", diagnostic_field.filter_stream_enabled == 1 ? "TRUE" : "FALSE");
    ROS_INFO("Number of Dropped IMU Packets: \t\t\t%lu packets\n", diagnostic_field.imu_dropped_packets);
    ROS_INFO("Number of Dropped FILTER Packets: \t\t%lu packets\n", diagnostic_field.filter_dropped_packets);
    ROS_INFO("Communications Port Bytes Written: \t\t%lu Bytes\n", diagnostic_field.com1_port_bytes_written);
    ROS_INFO("Communications Port Bytes Read: \t\t%lu Bytes\n", diagnostic_field.com1_port_bytes_read);
    ROS_INFO("Communications Port Write Overruns: \t\t%lu Bytes\n", diagnostic_field.com1_port_write_overruns);
    ROS_INFO("Communications Port Read Overruns: \t\t%lu Bytes\n", diagnostic_field.com1_port_read_overruns);
    ROS_INFO("IMU Parser Errors: \t\t\t\t%lu Errors\n", diagnostic_field.imu_parser_errors);
    ROS_INFO("IMU Message Count: \t\t\t\t%lu Messages\n", diagnostic_field.imu_message_count);
    ROS_INFO("IMU Last Message Received: \t\t\t%lu ms\n", diagnostic_field.imu_last_message_ms);
  }
  // Unable to get this function working for the GX5-45
  else {
    ROS_INFO("Command not supported on this model");
  }

  res.success = true;
  return true;
}

// Set zero angular-rate update threshold
bool Microstrain::set_zero_angle_update_threshold(microstrain_mips::SetZeroAngleUpdateThreshold::Request& req,
                                                  microstrain_mips::SetZeroAngleUpdateThreshold::Response& res) {
  ROS_INFO("Setting Zero Angular-Rate-Update threshold\n");

  zero_update_control.threshold = req.threshold;  // rads/s
  zero_update_control.enable = req.enable;        // enable zero-angular-rate update

  // Set ZUPT parameters
  start = clock();
  while (mip_filter_zero_angular_rate_update_control(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                                     &zero_update_control) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_zero_angular_rate_update_control function timed out.");
      break;
    }
  }

  // Read back parameter settings
  start = clock();
  while (mip_filter_zero_angular_rate_update_control(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                     &zero_update_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_zero_angular_rate_update_control function timed out.");
      break;
    }
  }

  if (zero_update_control.enable != zero_update_readback.enable ||
      zero_update_control.threshold != zero_update_readback.threshold)
    ROS_INFO("ERROR configuring Zero Angular Rate Update.\n");

  ROS_INFO("Enable value set to: %d, Threshold is: %f rad/s", zero_update_readback.enable,
           zero_update_readback.threshold);

  res.success = true;
  return true;
}

// Get zero angular rate update threshold value
bool Microstrain::get_zero_angle_update_threshold(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("Setting Zero Angular-Rate-Update threshold\n");
  // Read back parameter settings

  start = clock();
  while (mip_filter_zero_angular_rate_update_control(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                     &zero_update_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_zero_angular_rate_update_control function timed out.");
      break;
    }
  }
  ROS_INFO("Enable value set to: %d, Threshold is: %f rad/s", zero_update_readback.enable,
           zero_update_readback.threshold);

  res.success = true;
  return true;
}

// Set tare orientation angle values
bool Microstrain::set_tare_orientation(microstrain_mips::SetTareOrientation::Request& req,
                                       microstrain_mips::SetTareOrientation::Response& res) {
  if (req.axis < 1 || req.axis > 7) {
    ROS_INFO(
        "Value must be between 1-7. 1 = Roll, 2 = Pitch, 3 = Roll/Pitch, 4 = Yaw, 5 = Roll/Yaw, 6 = Pitch/Yaw, 7 = "
        "Roll/Pitch/Yaw");
    res.success = false;
  }

  angles[0] = angles[1] = angles[2] = 0;
  int i = req.axis;

  start = clock();
  while (mip_filter_set_init_attitude(&device_interface_, angles) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_set_init_attitude function timed out.");
      break;
    }
  }

  // Wait for Filter to re-establish running state
  Sleep(5000);
  // Cycle through axes combinations
  if (mip_filter_tare_orientation(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, i) != MIP_INTERFACE_OK) {
    ROS_INFO("ERROR: Failed Axis - ");

    if (i & FILTER_TARE_ROLL_AXIS) {
      ROS_INFO(" Roll Axis ");
    }

    if (i & FILTER_TARE_PITCH_AXIS) {
      ROS_INFO(" Pitch Axis ");
    }

    if (i & FILTER_TARE_YAW_AXIS) {
      ROS_INFO(" Yaw Axis ");
    }
  } else {
    ROS_INFO("Tare Configuration = %d\n", i);

    ROS_INFO("Tared -");

    if (i & FILTER_TARE_ROLL_AXIS) {
      ROS_INFO(" Roll Axis ");
    }

    if (i & FILTER_TARE_PITCH_AXIS) {
      ROS_INFO(" Pitch Axis ");
    }

    if (i & FILTER_TARE_YAW_AXIS) {
      ROS_INFO(" Yaw Axis ");
    }

    res.success = true;
    return true;
  }

  Sleep(1000);
}

// Set accel noise values
bool Microstrain::set_accel_noise(microstrain_mips::SetAccelNoise::Request& req,
                                  microstrain_mips::SetAccelNoise::Response& res) {
  ROS_INFO("Setting the accel noise values\n");

  noise[0] = req.noise.x;
  noise[1] = req.noise.y;
  noise[2] = req.noise.z;

  start = clock();
  while (mip_filter_accel_noise(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_accel_noise function timed out.");
      break;
    }
  }

  // Read back the accel noise values
  start = clock();
  while (mip_filter_accel_noise(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_accel_noise function timed out.");
      break;
    }
  }

  if ((abs(readback_noise[0] - noise[0]) < 0.001) && (abs(readback_noise[1] - noise[1]) < 0.001) &&
      (abs(readback_noise[2] - noise[2]) < 0.001)) {
    ROS_INFO("Accel noise values successfully set.\n");
  } else {
    ROS_INFO("ERROR: Failed to set accel noise values!!!\n");
    ROS_INFO("Sent values:     %f X %f Y %f Z\n", noise[0], noise[1], noise[2]);
    ROS_INFO("Returned values: %f X %f Y %f Z\n", readback_noise[0], readback_noise[1], readback_noise[2]);
  }

  res.success;
  return true;
}

// Get accel noise values
bool Microstrain::get_accel_noise(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  start = clock();
  while (mip_filter_accel_noise(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_accel_noise function timed out.");
      break;
    }
  }
  ROS_INFO("Accel noise values: %f X %f Y %f Z\n", readback_noise[0], readback_noise[1], readback_noise[2]);

  res.success = true;
  return true;
}

// Set gyro noise values
bool Microstrain::set_gyro_noise(microstrain_mips::SetGyroNoise::Request& req,
                                 microstrain_mips::SetGyroNoise::Response& res) {
  ROS_INFO("Setting the gyro noise values\n");

  noise[0] = req.noise.x;
  noise[1] = req.noise.y;
  noise[2] = req.noise.z;

  start = clock();
  while (mip_filter_gyro_noise(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_gyro_noise function timed out.");
      break;
    }
  }

  // Read back the gyro noise values
  start = clock();
  while (mip_filter_gyro_noise(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_gyro_noise function timed out.");
      break;
    }
  }

  if ((abs(readback_noise[0] - noise[0]) < 0.001) && (abs(readback_noise[1] - noise[1]) < 0.001) &&
      (abs(readback_noise[2] - noise[2]) < 0.001)) {
    ROS_INFO("Gyro noise values successfully set.\n");
  } else {
    ROS_INFO("ERROR: Failed to set gyro noise values!!!\n");
    ROS_INFO("Sent values:     %f X %f Y %f Z\n", noise[0], noise[1], noise[2]);
    ROS_INFO("Returned values: %f X %f Y %f Z\n", readback_noise[0], readback_noise[1], readback_noise[2]);
  }

  res.success = true;
  return true;
}

// Get gyro noise values
bool Microstrain::get_gyro_noise(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  start = clock();
  while (mip_filter_gyro_noise(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_gyro_noise function timed out.");
      break;
    }
  }
  ROS_INFO("Gyro noise values: %f X %f Y %f Z\n", readback_noise[0], readback_noise[1], readback_noise[2]);

  res.success = true;
  return true;
}

// Set magnetometer noise values
bool Microstrain::set_mag_noise(microstrain_mips::SetMagNoise::Request& req,
                                microstrain_mips::SetMagNoise::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  ROS_INFO("Setting the mag noise values\n");

  noise[0] = req.noise.x;
  noise[1] = req.noise.y;
  noise[2] = req.noise.z;

  start = clock();
  while (mip_filter_mag_noise(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_mag_noise function timed out.");
      break;
    }
  }

  // Read back the mag white noise values
  start = clock();
  while (mip_filter_mag_noise(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_mag_noise function timed out.");
      break;
    }
  }

  if ((abs(readback_noise[0] - noise[0]) < 0.001) && (abs(readback_noise[1] - noise[1]) < 0.001) &&
      (abs(readback_noise[2] - noise[2]) < 0.001)) {
    ROS_INFO("Mag noise values successfully set.\n");
  } else {
    ROS_INFO("ERROR: Failed to set mag noise values!!!\n");
    ROS_INFO("Sent values:     %f X %f Y %f Z\n", noise[0], noise[1], noise[2]);
    ROS_INFO("Returned values: %f X %f Y %f Z\n", readback_noise[0], readback_noise[1], readback_noise[2]);
  }

  res.success = true;
  return true;
}

// Get magnetometer noise values
bool Microstrain::get_mag_noise(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  start = clock();
  while (mip_filter_mag_noise(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_mag_noise function timed out.");
      break;
    }
  }
  ROS_INFO("Returned values: %f X %f Y %f Z\n", readback_noise[0], readback_noise[1], readback_noise[2]);

  res.success = true;
  return true;
}

// Set gyro bias model
bool Microstrain::set_gyro_bias_model(microstrain_mips::SetGyroBiasModel::Request& req,
                                      microstrain_mips::SetGyroBiasModel::Response& res) {
  ROS_INFO("Setting the gyro bias model values\n");

  noise[0] = req.noise_vector.x;
  noise[1] = req.noise_vector.y;
  noise[2] = req.noise_vector.z;

  beta[0] = req.beta_vector.x;
  beta[1] = req.beta_vector.x;
  beta[2] = req.beta_vector.x;

  start = clock();
  while (mip_filter_gyro_bias_model(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, beta, noise) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_gyro_bias_model function timed out.");
      break;
    }
  }

  // Read back the gyro bias model values
  start = clock();
  while (mip_filter_gyro_bias_model(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_beta, readback_noise) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_gyro_bias_model function timed out.");
      break;
    }
  }

  if ((abs(readback_noise[0] - noise[0]) < 0.001) && (abs(readback_noise[1] - noise[1]) < 0.001) &&
      (abs(readback_noise[2] - noise[2]) < 0.001) && (abs(readback_beta[0] - beta[0]) < 0.001) &&
      (abs(readback_beta[1] - beta[1]) < 0.001) && (abs(readback_beta[2] - beta[2]) < 0.001)) {
    ROS_INFO("Gyro bias model values successfully set.\n");
  } else {
    ROS_INFO("ERROR: Failed to set gyro bias model values!!!\n");
    ROS_INFO("Sent values:     Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", beta[0], beta[1], beta[2],
             noise[0], noise[1], noise[2]);
    ROS_INFO("Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", readback_beta[0],
             readback_beta[1], readback_beta[2], readback_noise[0], readback_noise[1], readback_noise[2]);
  }

  res.success = true;
  return true;
}

// Get gyro bias model
bool Microstrain::get_gyro_bias_model(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  start = clock();
  while (mip_filter_gyro_bias_model(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_beta, readback_noise) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_gyro_bias_model function timed out.");
      break;
    }
  }

  ROS_INFO("Gyro bias model values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", readback_beta[0],
           readback_beta[1], readback_beta[2], readback_noise[0], readback_noise[1], readback_noise[2]);
  res.success = true;
  return true;
}

// Get acces bias model
bool Microstrain::get_accel_bias_model(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (GX5_15 == true || GX5_25 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  memset(readback_noise, 0, 3 * sizeof(float));
  memset(readback_beta, 0, 3 * sizeof(float));

  start = clock();
  while (mip_filter_accel_bias_model(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_beta, readback_noise) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_accel_bias_model function timed out.");
      break;
    }
  }

  ROS_INFO("Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", readback_beta[0], readback_beta[1],
           readback_beta[2], readback_noise[0], readback_noise[1], readback_noise[2]);

  res.success = true;
  return true;
}

// Set accel bias model
bool Microstrain::set_accel_bias_model(microstrain_mips::SetAccelBiasModel::Request& req,
                                       microstrain_mips::SetAccelBiasModel::Response& res) {
  if (GX5_15 == true || GX5_25 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  memset(noise, 0, 3 * sizeof(float));
  memset(beta, 0, 3 * sizeof(float));
  memset(readback_noise, 0, 3 * sizeof(float));
  memset(readback_beta, 0, 3 * sizeof(float));
  ROS_INFO("Setting the accel bias model values\n");

  noise[0] = req.noise_vector.x;
  noise[1] = req.noise_vector.y;
  noise[2] = req.noise_vector.z;

  beta[0] = req.beta_vector.x;
  beta[1] = req.beta_vector.x;
  beta[2] = req.beta_vector.x;

  start = clock();
  while (mip_filter_accel_bias_model(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, beta, noise) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_accel_bias_model function timed out.");
      break;
    }
  }

  // Read back the accel bias model values
  start = clock();
  while (mip_filter_accel_bias_model(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_beta, readback_noise) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_accel_bias_model function timed out.");
      break;
    }
  }

  if ((abs(readback_noise[0] - noise[0]) < 0.001) && (abs(readback_noise[1] - noise[1]) < 0.001) &&
      (abs(readback_noise[2] - noise[2]) < 0.001) && (abs(readback_beta[0] - beta[0]) < 0.001) &&
      (abs(readback_beta[1] - beta[1]) < 0.001) && (abs(readback_beta[2] - beta[2]) < 0.001)) {
    ROS_INFO("Accel bias model values successfully set.\n");
  } else {
    ROS_INFO("ERROR: Failed to set accel bias model values!!!\n");
    ROS_INFO("Sent values:     Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", beta[0], beta[1], beta[2],
             noise[0], noise[1], noise[2]);
    ROS_INFO("Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n", readback_beta[0],
             readback_beta[1], readback_beta[2], readback_noise[0], readback_noise[1], readback_noise[2]);
  }

  res.success = true;
  return true;
}

// Set accel magnitude error adaptive measurement values
bool Microstrain::set_accel_adaptive_vals(microstrain_mips::SetAccelAdaptiveVals::Request& req,
                                          microstrain_mips::SetAccelAdaptiveVals::Response& res) {
  ROS_INFO("Setting the accel magnitude error adaptive measurement values\n");

  accel_magnitude_error_command.enable = req.enable;
  accel_magnitude_error_command.low_pass_cutoff = req.low_pass_cutoff;
  accel_magnitude_error_command.min_1sigma = req.min_1sigma;
  accel_magnitude_error_command.low_limit = req.low_limit;
  accel_magnitude_error_command.high_limit = req.high_limit;
  accel_magnitude_error_command.low_limit_1sigma = req.low_limit_1sigma;
  accel_magnitude_error_command.high_limit_1sigma = req.high_limit_1sigma;

  start = clock();
  while (mip_filter_accel_magnitude_error_adaptive_measurement(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                                               &accel_magnitude_error_command) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_accel_magnitude_error_adaptive_measurement function timed out.");
      break;
    }
  }

  // Read back the accel magnitude error adaptive measurement values
  start = clock();
  while (mip_filter_accel_magnitude_error_adaptive_measurement(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                               &accel_magnitude_error_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_accel_magnitude_error_adaptive_measurement function timed out.");
      break;
    }
  }

  if ((accel_magnitude_error_command.enable == accel_magnitude_error_readback.enable) &&
      (abs(accel_magnitude_error_command.low_pass_cutoff - accel_magnitude_error_readback.low_pass_cutoff) < 0.001) &&
      (abs(accel_magnitude_error_command.min_1sigma - accel_magnitude_error_readback.min_1sigma) < 0.001) &&
      (abs(accel_magnitude_error_command.low_limit - accel_magnitude_error_readback.low_limit) < 0.001) &&
      (abs(accel_magnitude_error_command.high_limit - accel_magnitude_error_readback.high_limit) < 0.001) &&
      (abs(accel_magnitude_error_command.low_limit_1sigma - accel_magnitude_error_readback.low_limit_1sigma) < 0.001) &&
      (abs(accel_magnitude_error_command.high_limit_1sigma - accel_magnitude_error_readback.high_limit_1sigma) <
       0.001)) {
    ROS_INFO("accel magnitude error adaptive measurement values successfully set.\n");
  } else {
    ROS_INFO("ERROR: Failed to set accel magnitude error adaptive measurement values!!!");
    ROS_INFO("Sent values: Enable: %i, Parameters: %f %f %f %f %f %f", accel_magnitude_error_command.enable,
             accel_magnitude_error_command.low_pass_cutoff, accel_magnitude_error_command.min_1sigma,
             accel_magnitude_error_command.low_limit, accel_magnitude_error_command.high_limit,
             accel_magnitude_error_command.low_limit_1sigma, accel_magnitude_error_command.high_limit_1sigma);
    ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f", accel_magnitude_error_readback.enable,
             accel_magnitude_error_readback.low_pass_cutoff, accel_magnitude_error_readback.min_1sigma,
             accel_magnitude_error_readback.low_limit, accel_magnitude_error_readback.high_limit,
             accel_magnitude_error_readback.low_limit_1sigma, accel_magnitude_error_readback.high_limit_1sigma);
  }

  res.success = true;
  return true;
}

// Get accep magnitude error adaptive measurement values
bool Microstrain::get_accel_adaptive_vals(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  start = clock();
  while (mip_filter_accel_magnitude_error_adaptive_measurement(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                               &accel_magnitude_error_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_accel_magnitude_error_adaptive_measurement function timed out.");
      break;
    }
  }
  ROS_INFO("Accel magnitude error adaptive measurement values are: Enable: %i, Parameters: %f %f %f %f %f %f",
           accel_magnitude_error_readback.enable, accel_magnitude_error_readback.low_pass_cutoff,
           accel_magnitude_error_readback.min_1sigma, accel_magnitude_error_readback.low_limit,
           accel_magnitude_error_readback.high_limit, accel_magnitude_error_readback.low_limit_1sigma,
           accel_magnitude_error_readback.high_limit_1sigma);

  res.success = true;
  return true;
}

// Set magnetometer magnitude error adaptive measurement values
bool Microstrain::set_mag_adaptive_vals(microstrain_mips::SetMagAdaptiveVals::Request& req,
                                        microstrain_mips::SetMagAdaptiveVals::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  ROS_INFO("Setting the mag magnitude error adaptive measurement values\n");

  mag_magnitude_error_command.enable = req.enable;
  mag_magnitude_error_command.low_pass_cutoff = req.low_pass_cutoff;
  mag_magnitude_error_command.min_1sigma = req.min_1sigma;
  mag_magnitude_error_command.low_limit = req.low_limit;
  mag_magnitude_error_command.high_limit = req.high_limit;
  mag_magnitude_error_command.low_limit_1sigma = req.low_limit_1sigma;
  mag_magnitude_error_command.high_limit_1sigma = req.high_limit_1sigma;

  start = clock();
  while (mip_filter_mag_magnitude_error_adaptive_measurement(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                                             &mag_magnitude_error_command) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_mag_magnitude_error_adaptive_measurement function timed out.");
      break;
    }
  }

  // Read back the mag magnitude error adaptive measurement values
  start = clock();
  while (mip_filter_mag_magnitude_error_adaptive_measurement(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                             &mag_magnitude_error_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_mag_magnitude_error_adaptive_measurement function timed out.");
      break;
    }
  }

  if ((mag_magnitude_error_command.enable == mag_magnitude_error_readback.enable) &&
      (abs(mag_magnitude_error_command.low_pass_cutoff - mag_magnitude_error_readback.low_pass_cutoff) < 0.001) &&
      (abs(mag_magnitude_error_command.min_1sigma - mag_magnitude_error_readback.min_1sigma) < 0.001) &&
      (abs(mag_magnitude_error_command.low_limit - mag_magnitude_error_readback.low_limit) < 0.001) &&
      (abs(mag_magnitude_error_command.high_limit - mag_magnitude_error_readback.high_limit) < 0.001) &&
      (abs(mag_magnitude_error_command.low_limit_1sigma - mag_magnitude_error_readback.low_limit_1sigma) < 0.001) &&
      (abs(mag_magnitude_error_command.high_limit_1sigma - mag_magnitude_error_readback.high_limit_1sigma) < 0.001)) {
    ROS_INFO("mag magnitude error adaptive measurement values successfully set.\n");
  } else {
    ROS_INFO("ERROR: Failed to set mag magnitude error adaptive measurement values!!!\n");
    ROS_INFO("Sent values:     Enable: %i, Parameters: %f %f %f %f %f %f\n", mag_magnitude_error_command.enable,
             mag_magnitude_error_command.low_pass_cutoff, mag_magnitude_error_command.min_1sigma,
             mag_magnitude_error_command.low_limit, mag_magnitude_error_command.high_limit,
             mag_magnitude_error_command.low_limit_1sigma, mag_magnitude_error_command.high_limit_1sigma);
    ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f\n", mag_magnitude_error_readback.enable,
             mag_magnitude_error_readback.low_pass_cutoff, mag_magnitude_error_readback.min_1sigma,
             mag_magnitude_error_readback.low_limit, mag_magnitude_error_readback.high_limit,
             mag_magnitude_error_readback.low_limit_1sigma, mag_magnitude_error_readback.high_limit_1sigma);
  }

  res.success = true;
  return true;
}

// Get magnetometer magnitude error adaptive measurement values
bool Microstrain::get_mag_adaptive_vals(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (GX5_15 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  start = clock();
  while (mip_filter_mag_magnitude_error_adaptive_measurement(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                             &mag_magnitude_error_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_mag_magnitude_error_adaptive_measurement function timed out.");
      break;
    }
  }

  ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f\n", mag_magnitude_error_readback.enable,
           mag_magnitude_error_readback.low_pass_cutoff, mag_magnitude_error_readback.min_1sigma,
           mag_magnitude_error_readback.low_limit, mag_magnitude_error_readback.high_limit,
           mag_magnitude_error_readback.low_limit_1sigma, mag_magnitude_error_readback.high_limit_1sigma);
  res.success = true;
  return true;
}

// Get magnetometer dip angle error adaptive measurement values
bool Microstrain::get_mag_dip_adaptive_vals(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (GX5_15 == true || GX5_25 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  start = clock();
  while (mip_filter_mag_dip_angle_error_adaptive_measurement(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                             &mag_dip_angle_error_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_mag_magnitude_error_adaptive_measurement function timed out.");
      break;
    }
  }

  ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f\n", mag_dip_angle_error_readback.enable,
           mag_dip_angle_error_readback.low_pass_cutoff, mag_dip_angle_error_readback.min_1sigma,
           mag_dip_angle_error_readback.high_limit, mag_dip_angle_error_readback.high_limit_1sigma);

  res.success = true;
  return true;
}

// Get magnetometer dip angle error adaptive measurement values
bool Microstrain::set_mag_dip_adaptive_vals(microstrain_mips::SetMagDipAdaptiveVals::Request& req,
                                            microstrain_mips::SetMagDipAdaptiveVals::Response& res) {
  if (GX5_15 == true || GX5_25 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  ROS_INFO("Setting the mag dip angle error adaptive measurement values\n");

  mag_dip_angle_error_command.enable = req.enable;
  mag_dip_angle_error_command.low_pass_cutoff = req.low_pass_cutoff;
  mag_dip_angle_error_command.min_1sigma = req.min_1sigma;
  mag_dip_angle_error_command.high_limit = req.high_limit;
  mag_dip_angle_error_command.high_limit_1sigma = req.high_limit_1sigma;

  start = clock();
  while (mip_filter_mag_dip_angle_error_adaptive_measurement(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                                                             &mag_dip_angle_error_command) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_mag_magnitude_error_adaptive_measurement function timed out.");
      break;
    }
  }

  // Read back the mag magnitude error adaptive measurement values
  start = clock();
  while (mip_filter_mag_dip_angle_error_adaptive_measurement(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                                                             &mag_dip_angle_error_readback) != MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_mag_magnitude_error_adaptive_measurement function timed out.");
      break;
    }
  }

  if ((mag_dip_angle_error_command.enable == mag_magnitude_error_readback.enable) &&
      (abs(mag_dip_angle_error_command.low_pass_cutoff - mag_dip_angle_error_readback.low_pass_cutoff) < 0.001) &&
      (abs(mag_dip_angle_error_command.min_1sigma - mag_dip_angle_error_readback.min_1sigma) < 0.001) &&
      (abs(mag_dip_angle_error_command.high_limit - mag_dip_angle_error_readback.high_limit) < 0.001) &&
      (abs(mag_dip_angle_error_command.high_limit_1sigma - mag_dip_angle_error_readback.high_limit_1sigma) < 0.001)) {
    ROS_INFO("mag dip angle error adaptive measurement values successfully set.\n");
  } else {
    ROS_INFO("ERROR: Failed to set mag dip angle error adaptive measurement values!!!\n");
    ROS_INFO("Sent values:     Enable: %i, Parameters: %f %f %f %f\n", mag_dip_angle_error_command.enable,
             mag_dip_angle_error_command.low_pass_cutoff, mag_dip_angle_error_command.min_1sigma,
             mag_dip_angle_error_command.high_limit, mag_dip_angle_error_command.high_limit_1sigma);

    ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f\n", mag_dip_angle_error_readback.enable,
             mag_dip_angle_error_readback.low_pass_cutoff, mag_dip_angle_error_readback.min_1sigma,
             mag_dip_angle_error_readback.high_limit, mag_dip_angle_error_readback.high_limit_1sigma);
  }

  res.success = true;
  return true;
}

// Get vehicle dynamics mode
bool Microstrain::get_dynamics_mode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (GX5_15 == true || GX5_25 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  readback_dynamics_mode = 0;
  while (mip_filter_vehicle_dynamics_mode(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &readback_dynamics_mode) !=
         MIP_INTERFACE_OK) {
  }

  ROS_INFO("Vehicle dynamics mode is: %d\n", dynamics_mode);

  res.success = true;
  return true;
}

// Set vehicle dynamics mode. Only in 45 model.
bool Microstrain::set_dynamics_mode(microstrain_mips::SetDynamicsMode::Request& req,
                                    microstrain_mips::SetDynamicsMode::Response& res) {
  if (GX5_15 == true || GX5_25 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  dynamics_mode = req.mode;

  if (dynamics_mode < 1 || dynamics_mode > 3) {
    ROS_INFO("Error: Vehicle dynamics mode must be between 1-3");
    res.success = false;
  } else {
    start = clock();
    while (mip_filter_vehicle_dynamics_mode(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &dynamics_mode) !=
           MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_filter_vehicle_dynamics_mode function timed out.");
        break;
      }
    }

    readback_dynamics_mode = 0;
    while (mip_filter_vehicle_dynamics_mode(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &readback_dynamics_mode) !=
           MIP_INTERFACE_OK) {
    }

    if (dynamics_mode == readback_dynamics_mode) {
      ROS_INFO("Vehicle dynamics mode successfully set to %d\n", dynamics_mode);
      res.success = true;
    } else {
      ROS_INFO("ERROR: Failed to set vehicle dynamics mode to %d!!!\n", dynamics_mode);
      res.success = false;
    }
  }
  return true;
}

// Set sensor to vehicle frame offset. Only in 45
bool Microstrain::set_sensor_vehicle_frame_offset(microstrain_mips::SetSensorVehicleFrameOffset::Request& req,
                                                  microstrain_mips::SetSensorVehicleFrameOffset::Response& res) {
  if (GX5_15 == true || GX5_25 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  memset(offset, 0, 3 * sizeof(float));
  memset(readback_offset, 0, 3 * sizeof(float));
  ROS_INFO("Setting the sensor to vehicle frame offset\n");

  offset[0] = req.offset.x;
  offset[1] = req.offset.y;
  offset[2] = req.offset.z;

  start = clock();
  while (mip_filter_sensor2vehicle_offset(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, offset) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_sensor2vehicle_offset function timed out.");
      break;
    }
  }

  // Read back the transformation
  start = clock();
  while (mip_filter_sensor2vehicle_offset(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_offset) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_sensor2vehicle_offset function timed out.");
      break;
    }
  }

  if ((abs(readback_offset[0] - offset[0]) < 0.001) && (abs(readback_offset[1] - offset[1]) < 0.001) &&
      (abs(readback_offset[2] - offset[2]) < 0.001)) {
    ROS_INFO("Offset successfully set.\n");
  } else {
    ROS_INFO("ERROR: Failed to set offset!!!\n");
    ROS_INFO("Sent offset:     %f X %f Y %f Z\n", offset[0], offset[1], offset[2]);
    ROS_INFO("Returned offset: %f X %f Y %f Z\n", readback_offset[0], readback_offset[1], readback_offset[2]);
  }

  res.success = true;
  return true;
}

// Get sensor to vehicle frame offset. Only in 45 model.
bool Microstrain::get_sensor_vehicle_frame_offset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (GX5_15 == true || GX5_25 == true) {
    ROS_INFO("Device does not support this feature");
    res.success = false;
    return true;
  }

  memset(readback_offset, 0, 3 * sizeof(float));

  start = clock();
  while (mip_filter_sensor2vehicle_offset(&device_interface_, MIP_FUNCTION_SELECTOR_READ, readback_offset) !=
         MIP_INTERFACE_OK) {
    if (clock() - start > 5000) {
      ROS_INFO("mip_filter_sensor2vehicle_offset function timed out.");
      break;
    }
  }

  ROS_INFO("Returned offset: %f X %f Y %f Z\n", readback_offset[0], readback_offset[1], readback_offset[2]);

  res.success = true;
  return true;
}

// Get basic or diagnostic status of device. Called by basic and diagnostic services.
u16 Microstrain::mip_3dm_cmd_hw_specific_device_status(mip_interface* device_interface, u16 model_number,
                                                       u8 status_selector, u8* response_buffer) {
  int total_size = 0;
  if (GX5_25) {
    gx4_25_basic_status_field* basic_ptr;
    gx4_25_diagnostic_device_status_field* diagnostic_ptr;
    u16 response_size = MIP_FIELD_HEADER_SIZE;

    // Set response size based on device model and whether basic or diagnostic status is chosen
    if (status_selector == GX4_25_BASIC_STATUS_SEL) {
      response_size += sizeof(gx4_25_basic_status_field);
    } else if (status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL) {
      response_size += sizeof(gx4_25_diagnostic_device_status_field);
    }

    while (mip_3dm_cmd_device_status(device_interface, model_number, status_selector, response_buffer,
                                     &response_size) != MIP_INTERFACE_OK) {
    }

    if (status_selector == GX4_25_BASIC_STATUS_SEL) {
      if (response_size != sizeof(gx4_25_basic_status_field)) {
        return MIP_INTERFACE_ERROR;
      } else if (MIP_SDK_CONFIG_BYTESWAP) {
        // Perform byteswapping
        byteswap_inplace(&response_buffer[0], sizeof(basic_field.device_model));
        byteswap_inplace(&response_buffer[2], sizeof(basic_field.status_selector));
        byteswap_inplace(&response_buffer[3], sizeof(basic_field.status_flags));
        byteswap_inplace(&response_buffer[7], sizeof(basic_field.system_state));
        byteswap_inplace(&response_buffer[9], sizeof(basic_field.system_timer_ms));
      }

      void* struct_pointer;
      struct_pointer = &basic_field;

      // Copy response from response buffer to basic status struct
      memcpy(struct_pointer, response_buffer, sizeof(basic_field.device_model));
      memcpy((struct_pointer + 2), &(response_buffer[2]), sizeof(basic_field.status_selector));
      memcpy((struct_pointer + 3), &(response_buffer[3]), sizeof(basic_field.status_flags));
      memcpy((struct_pointer + 7), &(response_buffer[7]), sizeof(basic_field.system_state));
      memcpy((struct_pointer + 9), &(response_buffer[9]), sizeof(basic_field.system_timer_ms));
    } else if (status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL) {
      if (response_size != sizeof(gx4_25_diagnostic_device_status_field)) {
        return MIP_INTERFACE_ERROR;
      } else if (MIP_SDK_CONFIG_BYTESWAP) {
        // byteswap and copy response to diagnostic status struct
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.device_model));
        total_size += sizeof(diagnostic_field.device_model);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.status_selector));
        total_size += sizeof(diagnostic_field.status_selector);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.status_flags));
        total_size += sizeof(diagnostic_field.status_flags);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.system_state));
        total_size += sizeof(diagnostic_field.system_state);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.system_timer_ms));
        total_size += sizeof(diagnostic_field.system_timer_ms);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.imu_stream_enabled));
        total_size += sizeof(diagnostic_field.imu_stream_enabled);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.filter_stream_enabled));
        total_size += sizeof(diagnostic_field.filter_stream_enabled);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.imu_dropped_packets));
        total_size += sizeof(diagnostic_field.imu_dropped_packets);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.filter_dropped_packets));
        total_size += sizeof(diagnostic_field.filter_dropped_packets);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.com1_port_bytes_written));
        total_size += sizeof(diagnostic_field.com1_port_bytes_written);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.com1_port_bytes_read));
        total_size += sizeof(diagnostic_field.com1_port_bytes_read);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.com1_port_write_overruns));
        total_size += sizeof(diagnostic_field.com1_port_write_overruns);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.com1_port_read_overruns));
        total_size += sizeof(diagnostic_field.com1_port_read_overruns);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.imu_parser_errors));
        total_size += sizeof(diagnostic_field.imu_parser_errors);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.imu_message_count));
        total_size += sizeof(diagnostic_field.imu_message_count);
        byteswap_inplace(&response_buffer[total_size], sizeof(diagnostic_field.imu_last_message_ms));
      }

      void* struct_pointer;
      struct_pointer = &diagnostic_field;
      total_size = 0;

      memcpy(struct_pointer, response_buffer, sizeof(diagnostic_field.device_model));
      total_size += sizeof(diagnostic_field.device_model);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]), sizeof(diagnostic_field.status_selector));
      total_size += sizeof(diagnostic_field.status_selector);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]), sizeof(diagnostic_field.status_flags));
      total_size += sizeof(diagnostic_field.status_flags);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]), sizeof(diagnostic_field.system_state));
      total_size += sizeof(diagnostic_field.system_state);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]), sizeof(diagnostic_field.system_timer_ms));
      total_size += sizeof(diagnostic_field.system_timer_ms);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
             sizeof(diagnostic_field.imu_stream_enabled));
      total_size += sizeof(diagnostic_field.imu_stream_enabled);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
             sizeof(diagnostic_field.filter_stream_enabled));
      total_size += sizeof(diagnostic_field.filter_stream_enabled);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
             sizeof(diagnostic_field.imu_dropped_packets));
      total_size += sizeof(diagnostic_field.imu_dropped_packets);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
             sizeof(diagnostic_field.filter_dropped_packets));
      total_size += sizeof(diagnostic_field.filter_dropped_packets);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
             sizeof(diagnostic_field.com1_port_bytes_written));
      total_size += sizeof(diagnostic_field.com1_port_bytes_written);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
             sizeof(diagnostic_field.com1_port_bytes_read));
      total_size += sizeof(diagnostic_field.com1_port_bytes_read);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
             sizeof(diagnostic_field.com1_port_write_overruns));
      total_size += sizeof(diagnostic_field.com1_port_write_overruns);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
             sizeof(diagnostic_field.com1_port_read_overruns));
      total_size += sizeof(diagnostic_field.com1_port_read_overruns);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]), sizeof(diagnostic_field.imu_parser_errors));
      total_size += sizeof(diagnostic_field.imu_parser_errors);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]), sizeof(diagnostic_field.imu_message_count));
      total_size += sizeof(diagnostic_field.imu_message_count);
      memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
             sizeof(diagnostic_field.imu_last_message_ms));
      total_size += sizeof(diagnostic_field.imu_last_message_ms);
    } else
      return MIP_INTERFACE_ERROR;

    return MIP_INTERFACE_OK;
  }
}

// Start callbacks for data packets

////////////////////////////////////////////////////////////////////////////////
//
// Filter Packet Callback
//
////////////////////////////////////////////////////////////////////////////////
void Microstrain::filter_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  mip_field_header* field_header;
  u8* field_data;
  u16 field_offset = 0;

  // If we aren't publishing, then return
  if (!publish_odom_ && !publish_filtered_imu_) return;

  // ROS_INFO("Filter callback");
  // The packet callback can have several types, process them all
  switch (callback_type) {
    ///
    // Handle valid packets
    ///
    case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
      filter_valid_packet_count_++;

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
            memcpy(&curr_filter_pos_, field_data, sizeof(mip_filter_llh_pos));

            // For little-endian targets, byteswap the data field
            mip_filter_llh_pos_byteswap(&curr_filter_pos_);

            nav_msg_.header.seq = filter_valid_packet_count_;
            nav_msg_.header.stamp = ros::Time::now();
            nav_msg_.header.frame_id = odom_frame_id_;
            nav_msg_.child_frame_id = odom_child_frame_id_;
            nav_msg_.pose.pose.position.y = curr_filter_pos_.latitude;
            nav_msg_.pose.pose.position.x = curr_filter_pos_.longitude;
            nav_msg_.pose.pose.position.z = curr_filter_pos_.ellipsoid_height;
          } break;

            ///
            // Estimated NED Velocity
            ///

          case MIP_FILTER_DATA_NED_VEL: {
            memcpy(&curr_filter_vel_, field_data, sizeof(mip_filter_ned_velocity));

            // For little-endian targets, byteswap the data field
            mip_filter_ned_velocity_byteswap(&curr_filter_vel_);

            // rotate velocities from NED to sensor coordinates
            // Constructor takes x, y, z , w
            tf2::Quaternion nav_quat(curr_filter_quaternion_.q[2], curr_filter_quaternion_.q[1],
                                     -1.0 * curr_filter_quaternion_.q[3], curr_filter_quaternion_.q[0]);

            tf2::Vector3 vel_enu(curr_filter_vel_.east, curr_filter_vel_.north, -1.0 * curr_filter_vel_.down);
            tf2::Vector3 vel_in_sensor_frame = tf2::quatRotate(nav_quat.inverse(), vel_enu);

            nav_msg_.twist.twist.linear.x = vel_in_sensor_frame[0];  // curr_filter_vel_.east;
            nav_msg_.twist.twist.linear.y = vel_in_sensor_frame[1];  // curr_filter_vel_.north;
            nav_msg_.twist.twist.linear.z = vel_in_sensor_frame[2];  // -1*curr_filter_vel_.down;
          } break;

            ///
            // Estimated Attitude, Euler Angles
            ///

          case MIP_FILTER_DATA_ATT_EULER_ANGLES: {
            memcpy(&curr_filter_angles_, field_data, sizeof(mip_filter_attitude_euler_angles));

            // For little-endian targets, byteswap the data field
            mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles_);
          } break;

          // Quaternion
          case MIP_FILTER_DATA_ATT_QUATERNION: {
            memcpy(&curr_filter_quaternion_, field_data, sizeof(mip_filter_attitude_quaternion));

            // For little-endian targets, byteswap the data field
            mip_filter_attitude_quaternion_byteswap(&curr_filter_quaternion_);

            // If we want the orientation to be based on the reference on the imu
            tf2::Quaternion q(curr_filter_quaternion_.q[1], curr_filter_quaternion_.q[2], curr_filter_quaternion_.q[3],
                              curr_filter_quaternion_.q[0]);
            geometry_msgs::Quaternion quat_msg;

            if (frame_based_enu_) {
              // Create a rotation from NED -> ENU
              tf2::Quaternion q_rotate;
              q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
              // Apply the NED to ENU rotation such that the coordinate frame matches
              q = q_rotate * q;
              quat_msg = tf2::toMsg(q);
            } else {
              // put into ENU - swap X/Y, invert Z
              quat_msg.x = q[1];
              quat_msg.y = q[0];
              quat_msg.z = -1.0 * q[2];
              quat_msg.w = q[3];
            }

            nav_msg_.pose.pose.orientation = quat_msg;

            if (publish_filtered_imu_) {
              // Header
              filtered_imu_msg_.header.seq = filter_valid_packet_count_;
              filtered_imu_msg_.header.stamp = ros::Time::now();
              filtered_imu_msg_.header.frame_id = imu_frame_id_;
              filtered_imu_msg_.orientation = nav_msg_.pose.pose.orientation;
            }
          } break;

          // Angular Rates
          case MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE: {
            memcpy(&curr_filter_angular_rate_, field_data, sizeof(mip_filter_compensated_angular_rate));

            // For little-endian targets, byteswap the data field
            mip_filter_compensated_angular_rate_byteswap(&curr_filter_angular_rate_);

            nav_msg_.twist.twist.angular.x = curr_filter_angular_rate_.x;
            nav_msg_.twist.twist.angular.y = curr_filter_angular_rate_.y;
            nav_msg_.twist.twist.angular.z = curr_filter_angular_rate_.z;

            if (publish_filtered_imu_) {
              filtered_imu_msg_.angular_velocity.x = curr_filter_angular_rate_.x;
              filtered_imu_msg_.angular_velocity.y = curr_filter_angular_rate_.y;
              filtered_imu_msg_.angular_velocity.z = curr_filter_angular_rate_.z;
            }
          } break;

          // Position Uncertainty
          case MIP_FILTER_DATA_POS_UNCERTAINTY: {
            memcpy(&curr_filter_pos_uncertainty_, field_data, sizeof(mip_filter_llh_pos_uncertainty));

            // For little-endian targets, byteswap the data field
            mip_filter_llh_pos_uncertainty_byteswap(&curr_filter_pos_uncertainty_);

            // x-axis
            nav_msg_.pose.covariance[0] = curr_filter_pos_uncertainty_.east * curr_filter_pos_uncertainty_.east;
            nav_msg_.pose.covariance[7] = curr_filter_pos_uncertainty_.north * curr_filter_pos_uncertainty_.north;
            nav_msg_.pose.covariance[14] = curr_filter_pos_uncertainty_.down * curr_filter_pos_uncertainty_.down;
          } break;

          // Velocity Uncertainty
          case MIP_FILTER_DATA_VEL_UNCERTAINTY: {
            memcpy(&curr_filter_vel_uncertainty_, field_data, sizeof(mip_filter_ned_vel_uncertainty));

            // For little-endian targets, byteswap the data field
            mip_filter_ned_vel_uncertainty_byteswap(&curr_filter_vel_uncertainty_);

            nav_msg_.twist.covariance[0] = curr_filter_vel_uncertainty_.east * curr_filter_vel_uncertainty_.east;
            nav_msg_.twist.covariance[7] = curr_filter_vel_uncertainty_.north * curr_filter_vel_uncertainty_.north;
            nav_msg_.twist.covariance[14] = curr_filter_vel_uncertainty_.down * curr_filter_vel_uncertainty_.down;
          } break;

          // Attitude Uncertainty
          case MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER: {
            memcpy(&curr_filter_att_uncertainty_, field_data, sizeof(mip_filter_euler_attitude_uncertainty));

            // For little-endian targets, byteswap the data field
            mip_filter_euler_attitude_uncertainty_byteswap(&curr_filter_att_uncertainty_);
            nav_msg_.pose.covariance[21] = curr_filter_att_uncertainty_.roll * curr_filter_att_uncertainty_.roll;
            nav_msg_.pose.covariance[28] = curr_filter_att_uncertainty_.pitch * curr_filter_att_uncertainty_.pitch;
            nav_msg_.pose.covariance[35] = curr_filter_att_uncertainty_.yaw * curr_filter_att_uncertainty_.yaw;

            if (publish_filtered_imu_) {
              filtered_imu_msg_.orientation_covariance[0] =
                  curr_filter_att_uncertainty_.roll * curr_filter_att_uncertainty_.roll;
              filtered_imu_msg_.orientation_covariance[4] =
                  curr_filter_att_uncertainty_.pitch * curr_filter_att_uncertainty_.pitch;
              filtered_imu_msg_.orientation_covariance[8] =
                  curr_filter_att_uncertainty_.yaw * curr_filter_att_uncertainty_.yaw;
            }
          } break;

          // Filter Status
          case MIP_FILTER_DATA_FILTER_STATUS: {
            memcpy(&curr_filter_status_, field_data, sizeof(mip_filter_status));

            // For little-endian targets, byteswap the data field
            mip_filter_status_byteswap(&curr_filter_status_);

            nav_status_msg_.data.clear();
            ROS_DEBUG_THROTTLE(1.0, "Filter Status: %#06X, Dyn. Mode: %#06X, Filter State: %#06X",
                               curr_filter_status_.filter_state, curr_filter_status_.dynamics_mode,
                               curr_filter_status_.status_flags);
            nav_status_msg_.data.push_back(curr_filter_status_.filter_state);
            nav_status_msg_.data.push_back(curr_filter_status_.dynamics_mode);
            nav_status_msg_.data.push_back(curr_filter_status_.status_flags);
            nav_status_pub_.publish(nav_status_msg_);
          } break;

            ///
            // Scaled Accelerometer
            ///

          case MIP_FILTER_DATA_LINEAR_ACCELERATION: {
            memcpy(&curr_filter_linear_accel_, field_data, sizeof(mip_filter_linear_acceleration));

            // For little-endian targets, byteswap the data field
            mip_filter_linear_acceleration_byteswap(&curr_filter_linear_accel_);

            // If we want gravity removed, use this as acceleration
            if (remove_imu_gravity_) {
              // Stuff into ROS message - acceleration already in m/s^2
              filtered_imu_msg_.linear_acceleration.x = curr_filter_linear_accel_.x;
              filtered_imu_msg_.linear_acceleration.y = curr_filter_linear_accel_.y;
              filtered_imu_msg_.linear_acceleration.z = curr_filter_linear_accel_.z;
            }
            // Otherwise, do nothing with this packet
          } break;

          case MIP_FILTER_DATA_COMPENSATED_ACCELERATION: {
            memcpy(&curr_filter_accel_comp_, field_data, sizeof(mip_filter_compensated_acceleration));

            // For little-endian targets, byteswap the data field
            mip_filter_compensated_acceleration_byteswap(&curr_filter_accel_comp_);

            // If we do not want to have gravity removed, use this as acceleration
            if (!remove_imu_gravity_) {
              // Stuff into ROS message - acceleration already in m/s^2
              filtered_imu_msg_.linear_acceleration.x = curr_filter_accel_comp_.x;
              filtered_imu_msg_.linear_acceleration.y = curr_filter_accel_comp_.y;
              filtered_imu_msg_.linear_acceleration.z = curr_filter_accel_comp_.z;
            }
            // Otherwise, do nothing with this packet
          } break;

          default:
            break;
        }
      }

      // Publish
      if (publish_odom_) {
        nav_pub_.publish(nav_msg_);
      }

      if (publish_filtered_imu_) {
        // Does it make sense to get the angular velocity bias and acceleration bias to populate these?
        // Since the sensor does not produce a covariance for linear acceleration, set it based
        // on our pulled in parameters.
        std::copy(imu_linear_cov_.begin(), imu_linear_cov_.end(),
                  filtered_imu_msg_.linear_acceleration_covariance.begin());
        // Since the sensor does not produce a covariance for angular velocity, set it based
        // on our pulled in parameters.
        std::copy(imu_angular_cov_.begin(), imu_angular_cov_.end(),
                  filtered_imu_msg_.angular_velocity_covariance.begin());
        filtered_imu_pub_.publish(filtered_imu_msg_);
      }
    } break;

      ///
      // Handle checksum error packets
      ///

    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR: {
      filter_checksum_error_packet_count_++;
    } break;

      ///
      // Handle timeout packets
      ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT: {
      filter_timeout_packet_count_++;
    } break;

    default:
      break;
  }

  print_packet_stats();
}  // filter_packet_callback

// Send diagnostic information to device status topic and diagnostic aggregator
void Microstrain::device_status_callback() {
  if (GX5_25) {
    u8 response_buffer[sizeof(gx4_25_diagnostic_device_status_field)];
    start = clock();
    while (mip_3dm_cmd_hw_specific_device_status(&device_interface_, GX4_25_MODEL_NUMBER, GX4_25_DIAGNOSTICS_STATUS_SEL,
                                                 response_buffer) != MIP_INTERFACE_OK) {
      if (clock() - start > 5000) {
        ROS_INFO("mip_3dm_cmd_hw_specific_device_status function timed out.");
        break;
      }
    }

    // ROS_INFO("Adding device diagnostics to status msg");
    // ROS_INFO("adding data to message");
    device_status_msg_.device_model = diagnostic_field.device_model;
    device_status_msg_.status_selector = diagnostic_field.status_selector;
    device_status_msg_.status_flags = diagnostic_field.status_flags;
    device_status_msg_.system_state = diagnostic_field.system_state;
    device_status_msg_.system_timer_ms = diagnostic_field.system_timer_ms;
    device_status_msg_.imu_stream_enabled = diagnostic_field.imu_stream_enabled;
    device_status_msg_.filter_stream_enabled = diagnostic_field.filter_stream_enabled;
    device_status_msg_.imu_dropped_packets = diagnostic_field.imu_dropped_packets;
    device_status_msg_.filter_dropped_packets = diagnostic_field.filter_dropped_packets;
    device_status_msg_.com1_port_bytes_written = diagnostic_field.com1_port_bytes_written;
    device_status_msg_.com1_port_bytes_read = diagnostic_field.com1_port_bytes_read;
    device_status_msg_.com1_port_write_overruns = diagnostic_field.com1_port_write_overruns;
    device_status_msg_.com1_port_read_overruns = diagnostic_field.com1_port_read_overruns;
    device_status_msg_.imu_parser_errors = diagnostic_field.imu_parser_errors;
    device_status_msg_.imu_message_count = diagnostic_field.imu_message_count;
    device_status_msg_.imu_last_message_ms = diagnostic_field.imu_last_message_ms;

    device_status_pub_.publish(device_status_msg_);
  } else {
    ROS_INFO("Device status messages not configured for this model");
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// AHRS Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void Microstrain::ahrs_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  mip_field_header* field_header;
  u8* field_data;
  u16 field_offset = 0;
  // If we aren't publishing, then return
  if (!publish_imu_) return;
  // The packet callback can have several types, process them all
  switch (callback_type) {
      ///
      // Handle valid packets
      ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
      ahrs_valid_packet_count_++;

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
            memcpy(&curr_ahrs_accel_, field_data, sizeof(mip_ahrs_scaled_accel));

            // For little-endian targets, byteswap the data field
            mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel_);

            // Stuff into ROS message - acceleration in m/s^2
            // Header
            imu_msg_.header.seq = ahrs_valid_packet_count_;
            imu_msg_.header.stamp = ros::Time::now();
            imu_msg_.header.frame_id = imu_frame_id_;
            imu_msg_.linear_acceleration.x = 9.81 * curr_ahrs_accel_.scaled_accel[0];
            imu_msg_.linear_acceleration.y = 9.81 * curr_ahrs_accel_.scaled_accel[1];
            imu_msg_.linear_acceleration.z = 9.81 * curr_ahrs_accel_.scaled_accel[2];
            // Since the sensor does not produce a covariance for linear acceleration,
            // set it based on our pulled in parameters.
            std::copy(imu_linear_cov_.begin(), imu_linear_cov_.end(), imu_msg_.linear_acceleration_covariance.begin());
          } break;

            ///
            // Scaled Gyro
            ///

          case MIP_AHRS_DATA_GYRO_SCALED: {
            memcpy(&curr_ahrs_gyro_, field_data, sizeof(mip_ahrs_scaled_gyro));

            // For little-endian targets, byteswap the data field
            mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro_);

            imu_msg_.angular_velocity.x = curr_ahrs_gyro_.scaled_gyro[0];
            imu_msg_.angular_velocity.y = curr_ahrs_gyro_.scaled_gyro[1];
            imu_msg_.angular_velocity.z = curr_ahrs_gyro_.scaled_gyro[2];
            // Since the sensor does not produce a covariance for angular velocity, set it based
            // on our pulled in parameters.
            std::copy(imu_angular_cov_.begin(), imu_angular_cov_.end(), imu_msg_.angular_velocity_covariance.begin());
          } break;

            ///
            // Scaled Magnetometer
            ///

          case MIP_AHRS_DATA_MAG_SCALED: {
            memcpy(&curr_ahrs_mag_, field_data, sizeof(mip_ahrs_scaled_mag));

            // For little-endian targets, byteswap the data field
            mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag_);
          } break;

          // Quaternion
          case MIP_AHRS_DATA_QUATERNION: {
            memcpy(&curr_ahrs_quaternion_, field_data, sizeof(mip_ahrs_quaternion));

            // For little-endian targets, byteswap the data field
            mip_ahrs_quaternion_byteswap(&curr_ahrs_quaternion_);

            // If we want the orientation to be based on the reference on the imu
            tf2::Quaternion q(curr_ahrs_quaternion_.q[1], curr_ahrs_quaternion_.q[2], curr_ahrs_quaternion_.q[3],
                              curr_ahrs_quaternion_.q[0]);
            geometry_msgs::Quaternion quat_msg;

            if (frame_based_enu_) {
              // Create a rotation from NED -> ENU
              tf2::Quaternion q_rotate;
              q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
              // Apply the NED to ENU rotation such that the coordinate frame matches
              q = q_rotate * q;
              quat_msg = tf2::toMsg(q);
            } else {
              // put into ENU - swap X/Y, invert Z
              quat_msg.x = q[1];
              quat_msg.y = q[0];
              quat_msg.z = -1.0 * q[2];
              quat_msg.w = q[3];
            }

            imu_msg_.orientation = quat_msg;

            // Since the MIP_AHRS data does not contain uncertainty values
            // we have to set them based on the parameter values.
            std::copy(imu_orientation_cov_.begin(), imu_orientation_cov_.end(),
                      imu_msg_.orientation_covariance.begin());
          } break;

          default:
            break;
        }
      }

      // Publish
      imu_pub_.publish(imu_msg_);
    } break;

      ///
      // Handle checksum error packets
      ///

    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR: {
      ahrs_checksum_error_packet_count_++;
    } break;

      ///
      // Handle timeout packets
      ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT: {
      ahrs_timeout_packet_count_++;
    } break;

    default:
      break;
  }
  print_packet_stats();
}  // ahrs_packet_callback

////////////////////////////////////////////////////////////////////////////////
//
// GPS Packet Callback
//
////////////////////////////////////////////////////////////////////////////////
void Microstrain::gps_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  mip_field_header* field_header;
  u8* field_data;
  u16 field_offset = 0;
  u8 msgvalid = 1;  // keep track of message validity

  // If we aren't publishing, then return
  if (!publish_gps_) return;
  // The packet callback can have several types, process them all
  switch (callback_type) {
      ///
      // Handle valid packets
      ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
      gps_valid_packet_count_++;

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
            memcpy(&curr_llh_pos_, field_data, sizeof(mip_gps_llh_pos));

            // For little-endian targets, byteswap the data field
            mip_gps_llh_pos_byteswap(&curr_llh_pos_);

            // push into ROS message
            gps_msg_.latitude = curr_llh_pos_.latitude;
            gps_msg_.longitude = curr_llh_pos_.longitude;
            gps_msg_.altitude = curr_llh_pos_.ellipsoid_height;
            gps_msg_.position_covariance_type = 2;  // diagnals known
            gps_msg_.position_covariance[0] = curr_llh_pos_.horizontal_accuracy * curr_llh_pos_.horizontal_accuracy;
            gps_msg_.position_covariance[4] = curr_llh_pos_.horizontal_accuracy * curr_llh_pos_.horizontal_accuracy;
            gps_msg_.position_covariance[8] = curr_llh_pos_.vertical_accuracy * curr_llh_pos_.vertical_accuracy;
            gps_msg_.status.status = curr_llh_pos_.valid_flags - 1;
            gps_msg_.status.service = 1;  // assumed
            // Header
            gps_msg_.header.seq = gps_valid_packet_count_;
            gps_msg_.header.stamp = ros::Time::now();
            gps_msg_.header.frame_id = gps_frame_id_;
          } break;

            ///
            // NED Velocity
            ///

          case MIP_GPS_DATA_NED_VELOCITY: {
            memcpy(&curr_ned_vel_, field_data, sizeof(mip_gps_ned_vel));

            // For little-endian targets, byteswap the data field
            mip_gps_ned_vel_byteswap(&curr_ned_vel_);
          } break;

            ///
            // GPS Time
            ///

          case MIP_GPS_DATA_GPS_TIME: {
            memcpy(&curr_gps_time_, field_data, sizeof(mip_gps_time));

            // For little-endian targets, byteswap the data field
            mip_gps_time_byteswap(&curr_gps_time_);
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
      msgvalid = 0;
      gps_checksum_error_packet_count_++;
    } break;

      ///
      // Handle timeout packets
      ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT: {
      msgvalid = 0;
      gps_timeout_packet_count_++;
    } break;

    default:
      break;
  }

  if (msgvalid) {
    // Publish the message
    gps_pub_.publish(gps_msg_);
  }

  print_packet_stats();
}  // gps_packet_callback

void Microstrain::print_packet_stats() {
  ROS_DEBUG_THROTTLE(1.0, "%u FILTER (%u errors)    %u AHRS (%u errors)    %u GPS (%u errors) Packets",
                     filter_valid_packet_count_, filter_timeout_packet_count_ + filter_checksum_error_packet_count_,
                     ahrs_valid_packet_count_, ahrs_timeout_packet_count_ + ahrs_checksum_error_packet_count_,
                     gps_valid_packet_count_, gps_timeout_packet_count_ + gps_checksum_error_packet_count_);
}  // print_packet_stats

// Wrapper callbacks
void filter_packet_callback_wrapper(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  Microstrain* ustrain = reinterpret_cast<Microstrain*>(user_ptr);
  ustrain->filter_packet_callback(user_ptr, packet, packet_size, callback_type);
}

void ahrs_packet_callback_wrapper(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  Microstrain* ustrain = reinterpret_cast<Microstrain*>(user_ptr);
  ustrain->ahrs_packet_callback(user_ptr, packet, packet_size, callback_type);
}
// Wrapper callbacks
void gps_packet_callback_wrapper(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type) {
  Microstrain* ustrain = reinterpret_cast<Microstrain*>(user_ptr);
  ustrain->gps_packet_callback(user_ptr, packet, packet_size, callback_type);
}

}  // namespace Microstrain
