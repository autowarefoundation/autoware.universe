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

#ifndef VEHICLE_AS_PACMOD_INTERFACE_H
#define VEHICLE_AS_PACMOD_INTERFACE_H

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "pacmod_msgs/msg/global_rpt.hpp"
#include "pacmod_msgs/msg/steer_system_cmd.hpp"
#include "pacmod_msgs/msg/system_cmd_float.hpp"
#include "pacmod_msgs/msg/system_cmd_int.hpp"
#include "pacmod_msgs/msg/system_rpt_float.hpp"
#include "pacmod_msgs/msg/system_rpt_int.hpp"
#include "pacmod_msgs/msg/wheel_speed_rpt.hpp"

#include "autoware_vehicle_msgs/msg/control_mode.hpp"
#include "autoware_vehicle_msgs/msg/raw_vehicle_command.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/steering.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"

class PacmodInterface : public rclcpp::Node
{
public:
  PacmodInterface();
  ~PacmodInterface();

private:
  typedef message_filters::sync_policies::ApproximateTime<
    pacmod_msgs::msg::SystemRptFloat, pacmod_msgs::msg::WheelSpeedRpt,
    pacmod_msgs::msg::SystemRptFloat, pacmod_msgs::msg::SystemRptFloat,
    pacmod_msgs::msg::SystemRptInt, pacmod_msgs::msg::SystemRptInt, pacmod_msgs::msg::GlobalRpt>
    PacmodFeedbacksSyncPolicy;

  /* subscribers */
  // From Autoware
  rclcpp::Subscription<autoware_vehicle_msgs::msg::RawVehicleCommand>::SharedPtr
    raw_vehicle_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr turn_signal_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_cmd_sub_;

  // From Pacmod
  std::unique_ptr<message_filters::Subscriber<pacmod_msgs::msg::SystemRptFloat>>
    steer_wheel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod_msgs::msg::WheelSpeedRpt>>
    wheel_speed_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod_msgs::msg::SystemRptFloat>> accel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod_msgs::msg::SystemRptFloat>> brake_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod_msgs::msg::SystemRptInt>> shift_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod_msgs::msg::SystemRptInt>> turn_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod_msgs::msg::GlobalRpt>> global_rpt_sub_;
  std::unique_ptr<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>> pacmod_feedbacks_sync_;

  /* publishers */
  // To Pacmod
  rclcpp::Publisher<pacmod_msgs::msg::SystemCmdFloat>::SharedPtr accel_cmd_pub_;
  rclcpp::Publisher<pacmod_msgs::msg::SystemCmdFloat>::SharedPtr brake_cmd_pub_;
  rclcpp::Publisher<pacmod_msgs::msg::SteerSystemCmd>::SharedPtr steer_cmd_pub_;
  rclcpp::Publisher<pacmod_msgs::msg::SystemCmdInt>::SharedPtr shift_cmd_pub_;
  rclcpp::Publisher<pacmod_msgs::msg::SystemCmdInt>::SharedPtr turn_cmd_pub_;

  // To Autoware
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Steering>::SharedPtr steering_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr shift_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr turn_signal_status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  /* ros param */
  std::string base_frame_id_;
  int command_timeout_ms_;  // vehicle_cmd timeout [ms]
  bool is_pacmod_rpt_received_;
  bool is_pacmod_enabled_;
  bool is_clear_override_needed_;
  bool prev_override_;
  double loop_rate_;           // [Hz]
  double tire_radius_;         // [m]
  double wheel_base_;          // [m]
  double steering_offset_;     // [rad] def: measured = truth + offset
  double vgr_coef_a_;          // variable gear ratio coeffs
  double vgr_coef_b_;          // variable gear ratio coeffs
  double vgr_coef_c_;          // variable gear ratio coeffs
  double accel_pedal_offset_;  // offset of accel pedal value
  double brake_pedal_offset_;  // offset of brake pedal value

  double emergency_brake_;             // brake command when emergency [m/s^2]
  double max_throttle_;                // max throttle [0~1]
  double max_brake_;                   // max throttle [0~1]
  double max_steering_wheel_;          // max steering wheel angle [rad]
  double max_steering_wheel_rate_;     // [rad/s]
  double min_steering_wheel_rate_;     // [rad/s]
  bool enable_steering_rate_control_;  // use steering angle speed for command [rad/s]

  /* input values */
  autoware_vehicle_msgs::msg::RawVehicleCommand::ConstSharedPtr raw_vehicle_cmd_ptr_;
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr turn_signal_cmd_ptr_;

  pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt_ptr_;  // [rad]
  pacmod_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;   // [m/s]
  pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr_;
  pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr_;  // [m/s]
  pacmod_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt_ptr_;    // [m/s]
  pacmod_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr_;      // [m/s]
  bool engage_cmd_;
  bool prev_engage_cmd_;
  rclcpp::Time vehicle_command_received_time_;

  /* callbacks */
  void callbackVehicleCmd(const autoware_vehicle_msgs::msg::RawVehicleCommand::ConstSharedPtr msg);
  void callbackTurnSignalCmd(const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);
  void callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void callbackPacmodRpt(
    const pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
    const pacmod_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
    const pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
    const pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
    const pacmod_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
    const pacmod_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
    const pacmod_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt);

  /*  functions */
  void publishCommands();
  double calculateVehicleVelocity(
    const pacmod_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
    const pacmod_msgs::msg::SystemRptInt & shift_rpt);
  double calculateVariableGearRatio(const double vel, const double steer_wheel);
  double calcSteerWheelRateCmd(const double gear_ratio);
  uint16_t toPacmodShiftCmd(const autoware_vehicle_msgs::msg::Shift & shift);
  uint16_t toPacmodTurnCmd(const autoware_vehicle_msgs::msg::TurnSignal & turn);
  int32_t toAutowareShiftCmd(const pacmod_msgs::msg::SystemRptInt & shift);
  int32_t toAutowareTurnSignal(const pacmod_msgs::msg::SystemRptInt & turn);
};

#endif
