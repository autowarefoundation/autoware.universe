/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SSC_INTERFACE_H
#define SSC_INTERFACE_H

#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/Time.h>

#include <automotive_navigation_msgs/ModuleState.h>
#include <automotive_platform_msgs/BrakeFeedback.h>
#include <automotive_platform_msgs/CurvatureFeedback.h>
#include <automotive_platform_msgs/GearCommand.h>
#include <automotive_platform_msgs/GearFeedback.h>
#include <automotive_platform_msgs/SpeedMode.h>
#include <automotive_platform_msgs/SteerMode.h>
#include <automotive_platform_msgs/ThrottleFeedback.h>
#include <automotive_platform_msgs/TurnSignalCommand.h>
#include <automotive_platform_msgs/VelocityAccelCov.h>
#include <pacmod_msgs/SystemRptFloat.h>
#include <pacmod_msgs/SystemRptInt.h>
#include <pacmod_msgs/WheelSpeedRpt.h>

#include <autoware_vehicle_msgs/ControlMode.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/Steering.h>
#include <autoware_vehicle_msgs/TurnSignal.h>
#include <autoware_vehicle_msgs/VehicleCommand.h>

static const std::string BASE_FRAME_ID = "base_link";

class SSCInterface
{
public:
  SSCInterface();
  ~SSCInterface();

  void run();

private:
  typedef message_filters::sync_policies::ApproximateTime<
    automotive_platform_msgs::VelocityAccelCov, automotive_platform_msgs::CurvatureFeedback,
    automotive_platform_msgs::ThrottleFeedback, automotive_platform_msgs::BrakeFeedback,
    automotive_platform_msgs::GearFeedback, pacmod_msgs::WheelSpeedRpt, pacmod_msgs::SystemRptFloat>
    SSCFeedbacksSyncPolicy;

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // subscribers
  ros::Subscriber vehicle_cmd_sub_;
  ros::Subscriber turn_signal_cmd_sub_;
  ros::Subscriber engage_sub_;
  ros::Subscriber module_states_sub_;
  message_filters::Subscriber<automotive_platform_msgs::VelocityAccelCov> * velocity_accel_cov_sub_;
  message_filters::Subscriber<automotive_platform_msgs::CurvatureFeedback> *
    curvature_feedback_sub_;
  message_filters::Subscriber<automotive_platform_msgs::ThrottleFeedback> * throttle_feedback_sub_;
  message_filters::Subscriber<automotive_platform_msgs::BrakeFeedback> * brake_feedback_sub_;
  message_filters::Subscriber<automotive_platform_msgs::GearFeedback> * gear_feedback_sub_;
  message_filters::Subscriber<pacmod_msgs::WheelSpeedRpt> * wheel_speed_sub_;
  message_filters::Subscriber<pacmod_msgs::SystemRptFloat> * steering_wheel_sub_;
  message_filters::Synchronizer<SSCFeedbacksSyncPolicy> * ssc_feedbacks_sync_;

  ros::Subscriber pacmod_turn_sub_;  // TEMP to support turn_signal

  // publishers
  ros::Publisher steer_mode_pub_;
  ros::Publisher speed_mode_pub_;
  ros::Publisher turn_signal_pub_;
  ros::Publisher gear_pub_;
  ros::Publisher current_shift_pub_;
  ros::Publisher control_mode_pub_;
  ros::Publisher current_twist_pub_;
  ros::Publisher current_steer_pub_;
  ros::Publisher current_steer_wheel_deg_pub_;
  ros::Publisher current_velocity_pub_;
  ros::Publisher current_velocity_kmph_pub_;
  ros::Publisher current_turn_signal_pub_;

  // ros param
  int command_timeout_;        // vehicle_cmd timeout [ms]
  double loop_rate_;           // [Hz]
  double wheel_base_;          // [m]
  double acceleration_limit_;  // [m/s^2]
  double deceleration_limit_;  // [m/s^2]
  double max_curvature_rate_;  // [rad/m/s]

  bool use_rear_wheel_speed_;     // instead of 'as/velocity_accel'
  bool use_adaptive_gear_ratio_;  // for more accurate steering angle (gr = theta_sw / theta_s)
  double tire_radius_;            // [m] (NOTE: used by 'use_rear_wheel_speed' mode)
  double ssc_gear_ratio_;         // gr = const (NOTE: used by 'use_adaptive_gear_ratio' mode)
  double agr_coef_a_, agr_coef_b_, agr_coef_c_;  // gr = a + b * speed^2 + c * theta_sw
  double steering_offset_;                       // [rad] def: measured = truth + offset

  // NOTE: default parameters in SSC
  // tire radius = 0.39             [m]
  // max steering angle = 0.533     [rad]
  // max steering wheel angle = 8.3 [rad]
  // -> ssc_gear_ratio = 16.135     [-]
  // max steering wheel rotation rate = 6.28 [rad/s]

  // variables
  bool engage_;
  bool command_initialized_;
  bool shift_cmd_initialized_;
  bool turn_signal_cmd_initialized_;
  double adaptive_gear_ratio_;
  ros::Time command_time_;
  autoware_vehicle_msgs::VehicleCommand vehicle_cmd_;
  autoware_vehicle_msgs::TurnSignal turn_signal_cmd_;
  automotive_navigation_msgs::ModuleState module_states_;
  ros::Rate * rate_;

  pacmod_msgs::WheelSpeedRptConstPtr wheel_speed_rpt_ptr_;
  automotive_platform_msgs::VelocityAccelCovConstPtr vel_acc_cov_ptr_;
  automotive_platform_msgs::GearFeedbackConstPtr gear_feedback_ptr_;

  // callbacks
  void callbackFromVehicleCmd(const autoware_vehicle_msgs::VehicleCommandConstPtr & msg);
  void callbackFromTurnSignalCmd(const autoware_vehicle_msgs::TurnSignalConstPtr & msg);
  void callbackFromEngage(const std_msgs::BoolConstPtr & msg);
  void callbackFromSSCModuleStates(const automotive_navigation_msgs::ModuleStateConstPtr & msg);
  void callbackFromSSCFeedbacks(
    const automotive_platform_msgs::VelocityAccelCovConstPtr & msg_velocity,
    const automotive_platform_msgs::CurvatureFeedbackConstPtr & msg_curvature,
    const automotive_platform_msgs::ThrottleFeedbackConstPtr & msg_throttle,
    const automotive_platform_msgs::BrakeFeedbackConstPtr & msg_brake,
    const automotive_platform_msgs::GearFeedbackConstPtr & msg_gear,
    const pacmod_msgs::WheelSpeedRptConstPtr & msg_wheel_speed,
    const pacmod_msgs::SystemRptFloatConstPtr & msg_steering_wheel);
  void callbackTurnSignal(const pacmod_msgs::SystemRptInt & turn);

  // functions
  void publishCommand();
  double calculateVehicleVelocity(
    const pacmod_msgs::WheelSpeedRpt & wheel_speed_rpt,
    const automotive_platform_msgs::VelocityAccelCov & vel_acc_cov,
    const automotive_platform_msgs::GearFeedback & gear_feedback, const bool use_rear_wheel_speed);
  uint8_t toSSCShiftCmd(const autoware_vehicle_msgs::Shift & shift);
  int32_t toAutowareTurnSignal(const pacmod_msgs::SystemRptInt & turn) const;
};

#endif  // SSC_INTERFACE_H
