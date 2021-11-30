// Copyright 2017-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SSC_INTERFACE__SSC_INTERFACE_HPP_
#define SSC_INTERFACE__SSC_INTERFACE_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <automotive_navigation_msgs/msg/module_state.hpp>
#include <automotive_platform_msgs/msg/brake_feedback.hpp>
#include <automotive_platform_msgs/msg/curvature_feedback.hpp>
#include <automotive_platform_msgs/msg/gear_command.hpp>
#include <automotive_platform_msgs/msg/gear_feedback.hpp>
#include <automotive_platform_msgs/msg/speed_mode.hpp>
#include <automotive_platform_msgs/msg/steer_mode.hpp>
#include <automotive_platform_msgs/msg/throttle_feedback.hpp>
#include <automotive_platform_msgs/msg/turn_signal_command.hpp>
#include <automotive_platform_msgs/msg/velocity_accel_cov.hpp>
#include <autoware_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_vehicle_msgs/msg/control_mode.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/shift_stamped.hpp>
#include <autoware_vehicle_msgs/msg/steering.hpp>
#include <autoware_vehicle_msgs/msg/turn_signal.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>
#include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>
#include <std_msgs/msg/header.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <string>

static const char BASE_FRAME_ID[] = "base_link";

class SSCInterface : public rclcpp::Node
{
public:
  SSCInterface();
  ~SSCInterface();

private:
  typedef message_filters::sync_policies::ApproximateTime<
    automotive_platform_msgs::msg::VelocityAccelCov,
    automotive_platform_msgs::msg::CurvatureFeedback,
    automotive_platform_msgs::msg::ThrottleFeedback, automotive_platform_msgs::msg::BrakeFeedback,
    automotive_platform_msgs::msg::GearFeedback, pacmod3_msgs::msg::WheelSpeedRpt,
    pacmod3_msgs::msg::SystemRptFloat>
    SSCFeedbacksSyncPolicy;

  // subscribers
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr turn_signal_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;
  rclcpp::Subscription<automotive_navigation_msgs::msg::ModuleState>::SharedPtr module_states_sub_;

  std::unique_ptr<message_filters::Subscriber<automotive_platform_msgs::msg::VelocityAccelCov>>
    velocity_accel_cov_sub_;
  std::unique_ptr<message_filters::Subscriber<automotive_platform_msgs::msg::CurvatureFeedback>>
    curvature_feedback_sub_;
  std::unique_ptr<message_filters::Subscriber<automotive_platform_msgs::msg::ThrottleFeedback>>
    throttle_feedback_sub_;
  std::unique_ptr<message_filters::Subscriber<automotive_platform_msgs::msg::BrakeFeedback>>
    brake_feedback_sub_;
  std::unique_ptr<message_filters::Subscriber<automotive_platform_msgs::msg::GearFeedback>>
    gear_feedback_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>> wheel_speed_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>
    steering_wheel_sub_;
  std::unique_ptr<message_filters::Synchronizer<SSCFeedbacksSyncPolicy>> ssc_feedbacks_sync_;

  // TEMP to support turn_signal
  rclcpp::Subscription<pacmod3_msgs::msg::SystemRptInt>::SharedPtr pacmod_turn_sub_;

  // publishers
  rclcpp::Publisher<automotive_platform_msgs::msg::SteerMode>::SharedPtr steer_mode_pub_;
  rclcpp::Publisher<automotive_platform_msgs::msg::SpeedMode>::SharedPtr speed_mode_pub_;
  rclcpp::Publisher<automotive_platform_msgs::msg::TurnSignalCommand>::SharedPtr turn_signal_pub_;
  rclcpp::Publisher<automotive_platform_msgs::msg::GearCommand>::SharedPtr gear_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr current_shift_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr current_twist_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Steering>::SharedPtr current_steer_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr
    current_steer_wheel_deg_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr current_velocity_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr current_velocity_kmph_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr current_turn_signal_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

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
  bool engage_ = false;
  bool command_initialized_ = false;
  bool shift_cmd_initialized_ = false;
  bool turn_signal_cmd_initialized_ = false;
  double adaptive_gear_ratio_;
  rclcpp::Time command_time_;
  autoware_vehicle_msgs::msg::VehicleCommand vehicle_cmd_;
  autoware_vehicle_msgs::msg::TurnSignal turn_signal_cmd_;
  automotive_navigation_msgs::msg::ModuleState module_states_;

  pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;
  automotive_platform_msgs::msg::VelocityAccelCov::ConstSharedPtr vel_acc_cov_ptr_;
  automotive_platform_msgs::msg::GearFeedback::ConstSharedPtr gear_feedback_ptr_;

  // callbacks
  void callbackFromVehicleCmd(const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg);
  void callbackFromTurnSignalCmd(const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);
  void callbackFromEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
  void callbackFromSSCModuleStates(
    const automotive_navigation_msgs::msg::ModuleState::ConstSharedPtr msg);
  void callbackFromSSCFeedbacks(
    const automotive_platform_msgs::msg::VelocityAccelCov::ConstSharedPtr msg_velocity,
    const automotive_platform_msgs::msg::CurvatureFeedback::ConstSharedPtr msg_curvature,
    const automotive_platform_msgs::msg::ThrottleFeedback::ConstSharedPtr msg_throttle,
    const automotive_platform_msgs::msg::BrakeFeedback::ConstSharedPtr msg_brake,
    const automotive_platform_msgs::msg::GearFeedback::ConstSharedPtr msg_gear,
    const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr msg_wheel_speed,
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr msg_steering_wheel);
  void callbackTurnSignal(const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn);

  // functions
  void publishCommand();
  double calculateVehicleVelocity(
    const pacmod3_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
    const automotive_platform_msgs::msg::VelocityAccelCov & vel_acc_cov,
    const automotive_platform_msgs::msg::GearFeedback & gear_feedback,
    const bool use_rear_wheel_speed);
  uint8_t toSSCShiftCmd(const autoware_vehicle_msgs::msg::Shift & shift);
  int32_t toAutowareTurnSignal(const pacmod3_msgs::msg::SystemRptInt & turn) const;
};

#endif  // SSC_INTERFACE__SSC_INTERFACE_HPP_
