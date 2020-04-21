/*

Copyright (c) 2017, Brian Bingham

This code is licensed under MIT license (see LICENSE file for details)

*/

#ifndef _MICROSTRAIN_3DM_H
#define _MICROSTRAIN_3DM_H

// Tell compiler that the following MIP SDI are C functions
extern "C" {
#include "GX4-45_Test.h"
#include "byteswap_utilities.h"
#include "mip_gx4_25.h"
#include "mip_gx4_45.h"
#include "mip_gx4_imu.h"
#include "mip_sdk.h"
#include "mip_sdk_3dm.h"
}

#include <time.h>
#include <unistd.h>
#include <cstdio>

// ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "microstrain_mips/status_msg.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"

#include "microstrain_mips/SetAccelAdaptiveVals.h"
#include "microstrain_mips/SetAccelBias.h"
#include "microstrain_mips/SetAccelBiasModel.h"
#include "microstrain_mips/SetAccelNoise.h"
#include "microstrain_mips/SetComplementaryFilter.h"
#include "microstrain_mips/SetConingScullingComp.h"
#include "microstrain_mips/SetDynamicsMode.h"
#include "microstrain_mips/SetEstimationControlFlags.h"
#include "microstrain_mips/SetFilterEuler.h"
#include "microstrain_mips/SetFilterHeading.h"
#include "microstrain_mips/SetGyroBias.h"
#include "microstrain_mips/SetGyroBiasModel.h"
#include "microstrain_mips/SetGyroNoise.h"
#include "microstrain_mips/SetHardIronValues.h"
#include "microstrain_mips/SetMagAdaptiveVals.h"
#include "microstrain_mips/SetMagDipAdaptiveVals.h"
#include "microstrain_mips/SetMagNoise.h"
#include "microstrain_mips/SetReferencePosition.h"
#include "microstrain_mips/SetSensorVehicleFrameOffset.h"
#include "microstrain_mips/SetSensorVehicleFrameTrans.h"
#include "microstrain_mips/SetSoftIronMatrix.h"
#include "microstrain_mips/SetTareOrientation.h"
#include "microstrain_mips/SetZeroAngleUpdateThreshold.h"

#define MIP_SDK_GX4_45_IMU_STANDARD_MODE 0x01
#define MIP_SDK_GX4_45_IMU_DIRECT_MODE 0x02

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS 1000  // milliseconds

// macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x * 1000.0)

#define GX5_45_DEVICE "3DM-GX5-45"
#define GX5_35_DEVICE "3DM-GX5-35"
#define GX5_25_DEVICE "3DM-GX5-25"
#define GX5_15_DEVICE "3DM-GX5-15"

/**
 * \brief Contains functions for micostrain driver
 */
namespace Microstrain {
/**
 * \brief Microstrain class
 *
 */
class Microstrain {
 public:
  /**
   * Contructor
   */
  Microstrain();

  /** Destructor */
  ~Microstrain();

  /**
   * Main run loop
   */
  void run();

  //! Nav estimate callback
  void filter_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type);
  //! @brief AHRS callback
  void ahrs_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type);
  //! @brief GPS callback
  void gps_packet_callback(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type);

  void device_status_callback();

  bool set_accel_bias(microstrain_mips::SetAccelBias::Request& req, microstrain_mips::SetAccelBias::Response& res);

  bool get_accel_bias(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_gyro_bias(microstrain_mips::SetGyroBias::Request& req, microstrain_mips::SetGyroBias::Response& res);

  bool get_gyro_bias(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_hard_iron_values(microstrain_mips::SetHardIronValues::Request& req,
                            microstrain_mips::SetHardIronValues::Response& res);

  bool get_hard_iron_values(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool device_report(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool gyro_bias_capture(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_soft_iron_matrix(microstrain_mips::SetSoftIronMatrix::Request& req,
                            microstrain_mips::SetSoftIronMatrix::Response& res);

  bool get_soft_iron_matrix(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_complementary_filter(microstrain_mips::SetComplementaryFilter::Request& req,
                                microstrain_mips::SetComplementaryFilter::Response& res);

  bool get_complementary_filter(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_filter_euler(microstrain_mips::SetFilterEuler::Request& req,
                        microstrain_mips::SetFilterEuler::Response& res);

  bool set_filter_heading(microstrain_mips::SetFilterHeading::Request& req,
                          microstrain_mips::SetFilterHeading::Response& res);

  bool set_accel_bias_model(microstrain_mips::SetAccelBiasModel::Request& req,
                            microstrain_mips::SetAccelBiasModel::Response& res);

  bool set_accel_adaptive_vals(microstrain_mips::SetAccelAdaptiveVals::Request& req,
                               microstrain_mips::SetAccelAdaptiveVals::Response& res);

  bool set_sensor_vehicle_frame_trans(microstrain_mips::SetSensorVehicleFrameTrans::Request& req,
                                      microstrain_mips::SetSensorVehicleFrameTrans::Response& res);

  bool get_sensor_vehicle_frame_trans(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_sensor_vehicle_frame_offset(microstrain_mips::SetSensorVehicleFrameOffset::Request& req,
                                       microstrain_mips::SetSensorVehicleFrameOffset::Response& res);

  bool get_sensor_vehicle_frame_offset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_reference_position(microstrain_mips::SetReferencePosition::Request& req,
                              microstrain_mips::SetReferencePosition::Response& res);

  bool get_reference_position(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_coning_sculling_comp(microstrain_mips::SetConingScullingComp::Request& req,
                                microstrain_mips::SetConingScullingComp::Response& res);

  bool get_coning_sculling_comp(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_estimation_control_flags(microstrain_mips::SetEstimationControlFlags::Request& req,
                                    microstrain_mips::SetEstimationControlFlags::Response& res);

  bool get_estimation_control_flags(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_dynamics_mode(microstrain_mips::SetDynamicsMode::Request& req,
                         microstrain_mips::SetDynamicsMode::Response& res);

  bool get_dynamics_mode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool get_basic_status(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  u16 mip_3dm_cmd_hw_specific_device_status(mip_interface* device_interface, u16 model_number, u8 status_selector,
                                            u8* response_buffer);

  bool get_diagnostic_report(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_zero_angle_update_threshold(microstrain_mips::SetZeroAngleUpdateThreshold::Request& req,
                                       microstrain_mips::SetZeroAngleUpdateThreshold::Response& res);

  bool get_zero_angle_update_threshold(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_tare_orientation(microstrain_mips::SetTareOrientation::Request& req,
                            microstrain_mips::SetTareOrientation::Response& res);

  bool set_accel_noise(microstrain_mips::SetAccelNoise::Request& req, microstrain_mips::SetAccelNoise::Response& res);

  bool get_accel_noise(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_gyro_noise(microstrain_mips::SetGyroNoise::Request& req, microstrain_mips::SetGyroNoise::Response& res);

  bool get_gyro_noise(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_mag_noise(microstrain_mips::SetMagNoise::Request& req, microstrain_mips::SetMagNoise::Response& res);

  bool get_mag_noise(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_gyro_bias_model(microstrain_mips::SetGyroBiasModel::Request& req,
                           microstrain_mips::SetGyroBiasModel::Response& res);

  bool get_gyro_bias_model(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool get_accel_adaptive_vals(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_mag_adaptive_vals(microstrain_mips::SetMagAdaptiveVals::Request& req,
                             microstrain_mips::SetMagAdaptiveVals::Response& res);

  bool get_mag_adaptive_vals(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool set_mag_dip_adaptive_vals(microstrain_mips::SetMagDipAdaptiveVals::Request& req,
                                 microstrain_mips::SetMagDipAdaptiveVals::Response& res);

  bool get_mag_dip_adaptive_vals(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool get_accel_bias_model(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool get_model_gps() {
    if (Microstrain::GX5_45 || Microstrain::GX5_35)
      return true;
    else
      return false;
  }

 private:
  //! @brief Reset KF service callback
  bool reset_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
  //! @brief Convience for printing packet stats
  void print_packet_stats();

  // Variables/fields
  // The primary device interface structure
  mip_interface device_interface_;
  base_device_info_field device_info;
  u8 temp_string[20];

  // Packet Counters (valid, timeout, and checksum errors)
  u32 filter_valid_packet_count_;
  u32 ahrs_valid_packet_count_;
  u32 gps_valid_packet_count_;

  u32 filter_timeout_packet_count_;
  u32 ahrs_timeout_packet_count_;
  u32 gps_timeout_packet_count_;

  u32 filter_checksum_error_packet_count_;
  u32 ahrs_checksum_error_packet_count_;
  u32 gps_checksum_error_packet_count_;

  // Data field storage
  // AHRS
  mip_ahrs_scaled_gyro curr_ahrs_gyro_;
  mip_ahrs_scaled_accel curr_ahrs_accel_;
  mip_ahrs_scaled_mag curr_ahrs_mag_;
  mip_ahrs_quaternion curr_ahrs_quaternion_;
  // GPS
  mip_gps_llh_pos curr_llh_pos_;
  mip_gps_ned_vel curr_ned_vel_;
  mip_gps_time curr_gps_time_;

  // FILTER
  mip_filter_llh_pos curr_filter_pos_;
  mip_filter_ned_velocity curr_filter_vel_;
  mip_filter_attitude_euler_angles curr_filter_angles_;
  mip_filter_attitude_quaternion curr_filter_quaternion_;
  mip_filter_compensated_angular_rate curr_filter_angular_rate_;
  mip_filter_compensated_acceleration curr_filter_accel_comp_;
  mip_filter_linear_acceleration curr_filter_linear_accel_;
  mip_filter_llh_pos_uncertainty curr_filter_pos_uncertainty_;
  mip_filter_ned_vel_uncertainty curr_filter_vel_uncertainty_;
  mip_filter_euler_attitude_uncertainty curr_filter_att_uncertainty_;
  mip_filter_status curr_filter_status_;

  // ROS
  ros::Publisher gps_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher filtered_imu_pub_;
  ros::Publisher nav_pub_;
  ros::Publisher nav_status_pub_;
  ros::Publisher bias_pub_;
  ros::Publisher device_status_pub_;
  sensor_msgs::NavSatFix gps_msg_;
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::Imu filtered_imu_msg_;
  nav_msgs::Odometry nav_msg_;
  std_msgs::Int16MultiArray nav_status_msg_;
  geometry_msgs::Vector3 bias_msg_;
  std::string gps_frame_id_;
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  std::string odom_child_frame_id_;
  microstrain_mips::status_msg device_status_msg_;
  bool publish_gps_;
  bool publish_imu_;
  bool publish_odom_;
  bool publish_bias_;
  bool publish_filtered_imu_;
  bool remove_imu_gravity_;
  bool frame_based_enu_;
  std::vector<double> imu_linear_cov_;
  std::vector<double> imu_angular_cov_;
  std::vector<double> imu_orientation_cov_;

  // Device Flags
  bool GX5_15;
  bool GX5_25;
  bool GX5_35;
  bool GX5_45;
  bool GQX_45;
  bool RQX_45;
  bool CXX_45;
  bool CVX_10;
  bool CVX_15;
  bool CVX_25;

  // Update rates
  int nav_rate_;
  int imu_rate_;
  int gps_rate_;

  clock_t start;
  float field_data[3];
  float soft_iron[9];
  float soft_iron_readback[9];
  float angles[3];
  float heading_angle;
  float readback_angles[3];
  float noise[3];
  float beta[3];
  float readback_beta[3];
  float readback_noise[3];
  float offset[3];
  float readback_offset[3];
  u8 com_mode;
  u16 duration;
  u8 reference_position_enable_command;
  u8 reference_position_enable_readback;
  double reference_position_command[3];
  double reference_position_readback[3];
  u8 enable_flag;
  u16 estimation_control;
  u16 estimation_control_readback;
  u8 dynamics_mode;
  u8 readback_dynamics_mode;
  gx4_25_basic_status_field basic_field;
  gx4_25_diagnostic_device_status_field diagnostic_field;
  gx4_45_basic_status_field basic_field_45;
  gx4_45_diagnostic_device_status_field diagnostic_field_45;
  mip_complementary_filter_settings comp_filter_command, comp_filter_readback;
  mip_filter_accel_magnitude_error_adaptive_measurement_command accel_magnitude_error_command,
      accel_magnitude_error_readback;
  mip_filter_magnetometer_magnitude_error_adaptive_measurement_command mag_magnitude_error_command,
      mag_magnitude_error_readback;
  mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command mag_dip_angle_error_command,
      mag_dip_angle_error_readback;
  mip_filter_zero_update_command zero_update_control, zero_update_readback;
};  // Microstrain class

// Define wrapper functions that call the Microstrain member functions
#ifdef __cplusplus
extern "C"
#endif
{

/**
 * Callback for KF estimate packets from sensor.
 */
void filter_packet_callback_wrapper(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type);
/**
 * Callback for AHRS packets from sensor.
 */
void ahrs_packet_callback_wrapper(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type);
/**
 * Callback for GPS packets from sensor.
 */
void gps_packet_callback_wrapper(void* user_ptr, u8* packet, u16 packet_size, u8 callback_type);

#ifdef __cplusplus
}
#endif

}  // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
