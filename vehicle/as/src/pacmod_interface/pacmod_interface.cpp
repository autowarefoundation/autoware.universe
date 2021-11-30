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

#include "pacmod_interface/pacmod_interface.h"

namespace
{
template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_WARN("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}
}  // namespace

PacmodInterface::PacmodInterface()
: nh_(),
  private_nh_("~"),
  engage_cmd_(false),
  prev_engage_cmd_(false),
  is_pacmod_rpt_received_(false),
  is_pacmod_enabled_(false),
  is_clear_override_needed_(false)
{
  /* setup parameters */
  private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  private_nh_.param<int>("command_timeout_ms", command_timeout_ms_, 1000);
  private_nh_.param<double>("loop_rate", loop_rate_, 30.0);
  private_nh_.param<bool>("show_debug_info", show_debug_info_, false);

  /* parameters for vehicle specifications */
  tire_radius_ = waitForParam<double>(private_nh_, "/vehicle_info/wheel_radius");
  wheel_base_ = waitForParam<double>(private_nh_, "/vehicle_info/wheel_base");
  private_nh_.param<double>("steering_offset", steering_offset_, 0.0);
  private_nh_.param<bool>("enable_steering_rate_control", enable_steering_rate_control_, false);

  /* parameters for emergency stop */
  private_nh_.param<double>("emergency_brake", emergency_brake_, 0.7);
  private_nh_.param<double>("max_throttle", max_throttle_, 0.2);
  private_nh_.param<double>("max_brake", max_brake_, 0.8);

  private_nh_.param<double>("vgr_coef_a", vgr_coef_a_, 15.713);
  private_nh_.param<double>("vgr_coef_b", vgr_coef_b_, 0.053);
  private_nh_.param<double>("vgr_coef_c", vgr_coef_c_, 0.042);

  /* parameters for limitter */
  private_nh_.param<double>("max_steering_wheel", max_steering_wheel_, 2.7 * M_PI);
  private_nh_.param<double>("max_steering_wheel_rate", max_steering_wheel_rate_, 6.6);
  private_nh_.param<double>("min_steering_wheel_rate", min_steering_wheel_rate_, 0.5);

  rate_ = new ros::Rate(loop_rate_);

  /* subscribers */
  // From autoware
  raw_vehicle_cmd_sub_ =
    nh_.subscribe("/vehicle/raw_vehicle_cmd", 1, &PacmodInterface::callbackVehicleCmd, this);
  turn_signal_cmd_sub_ =
    nh_.subscribe("/control/turn_signal_cmd", 1, &PacmodInterface::callbackTurnSignalCmd, this);
  engage_cmd_sub_ = nh_.subscribe("/vehicle/engage", 1, &PacmodInterface::callbackEngage, this);

  // From pacmod
  steer_wheel_rpt_sub_ = new message_filters::Subscriber<pacmod_msgs::SystemRptFloat>(
    nh_, "/pacmod/parsed_tx/steer_rpt", 1);
  wheel_speed_rpt_sub_ = new message_filters::Subscriber<pacmod_msgs::WheelSpeedRpt>(
    nh_, "/pacmod/parsed_tx/wheel_speed_rpt", 1);
  accel_rpt_sub_ = new message_filters::Subscriber<pacmod_msgs::SystemRptFloat>(
    nh_, "/pacmod/parsed_tx/accel_rpt", 1);
  brake_rpt_sub_ = new message_filters::Subscriber<pacmod_msgs::SystemRptFloat>(
    nh_, "/pacmod/parsed_tx/brake_rpt", 1);
  shift_rpt_sub_ = new message_filters::Subscriber<pacmod_msgs::SystemRptInt>(
    nh_, "/pacmod/parsed_tx/shift_rpt", 1);
  turn_rpt_sub_ = new message_filters::Subscriber<pacmod_msgs::SystemRptInt>(
    nh_, "/pacmod/parsed_tx/turn_rpt", 1);
  global_rpt_sub_ =
    new message_filters::Subscriber<pacmod_msgs::GlobalRpt>(nh_, "/pacmod/parsed_tx/global_rpt", 1);
  pacmod_feedbacks_sync_ = new message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>(
    PacmodFeedbacksSyncPolicy(10), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,
    *brake_rpt_sub_, *shift_rpt_sub_, *turn_rpt_sub_, *global_rpt_sub_);
  pacmod_feedbacks_sync_->registerCallback(
    boost::bind(&PacmodInterface::callbackPacmodRpt, this, _1, _2, _3, _4, _5, _6, _7));

  /* publisher */
  // To pacmod
  accel_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdFloat>("pacmod/as_rx/accel_cmd", 1);
  brake_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdFloat>("pacmod/as_rx/brake_cmd", 1);
  steer_cmd_pub_ = nh_.advertise<pacmod_msgs::SteerSystemCmd>("pacmod/as_rx/steer_cmd", 1);
  shift_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdInt>("pacmod/as_rx/shift_cmd", 1);
  turn_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdInt>("pacmod/as_rx/turn_cmd", 1);

  // To Autoware
  control_mode_pub_ =
    nh_.advertise<autoware_vehicle_msgs::ControlMode>("/vehicle/status/control_mode", 10);
  vehicle_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vehicle/status/twist", 1);
  steering_status_pub_ =
    nh_.advertise<autoware_vehicle_msgs::Steering>("/vehicle/status/steering", 1);
  shift_status_pub_ =
    nh_.advertise<autoware_vehicle_msgs::ShiftStamped>("/vehicle/status/shift", 1);
  turn_signal_status_pub_ =
    nh_.advertise<autoware_vehicle_msgs::TurnSignal>("/vehicle/status/turn_signal", 1);
}

PacmodInterface::~PacmodInterface() {}

void PacmodInterface::run()
{
  while (ros::ok()) {
    ros::spinOnce();
    publishCommands();
    rate_->sleep();
  }
}

void PacmodInterface::callbackVehicleCmd(
  const autoware_vehicle_msgs::RawVehicleCommand::ConstPtr & msg)
{
  vehicle_command_received_time_ = ros::Time::now();
  raw_vehicle_cmd_ptr_ = std::make_shared<autoware_vehicle_msgs::RawVehicleCommand>(*msg);
}
void PacmodInterface::callbackTurnSignalCmd(const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg)
{
  turn_signal_cmd_ptr_ = std::make_shared<autoware_vehicle_msgs::TurnSignal>(*msg);
}
void PacmodInterface::callbackEngage(const std_msgs::BoolConstPtr & msg)
{
  engage_cmd_ = msg->data;
  is_clear_override_needed_ = true;
}
void PacmodInterface::callbackPacmodRpt(
  const pacmod_msgs::SystemRptFloatConstPtr & steer_wheel_rpt,
  const pacmod_msgs::WheelSpeedRptConstPtr & wheel_speed_rpt,
  const pacmod_msgs::SystemRptFloatConstPtr & accel_rpt,
  const pacmod_msgs::SystemRptFloatConstPtr & brake_rpt,
  const pacmod_msgs::SystemRptIntConstPtr & shift_rpt,
  const pacmod_msgs::SystemRptIntConstPtr & turn_rpt,
  const pacmod_msgs::GlobalRptConstPtr & global_rpt)
{
  is_pacmod_rpt_received_ = true;
  steer_wheel_rpt_ptr_ = std::make_shared<pacmod_msgs::SystemRptFloat>(*steer_wheel_rpt);
  wheel_speed_rpt_ptr_ = std::make_shared<pacmod_msgs::WheelSpeedRpt>(*wheel_speed_rpt);
  accel_rpt_ptr_ = std::make_shared<pacmod_msgs::SystemRptFloat>(*accel_rpt);
  brake_rpt_ptr_ = std::make_shared<pacmod_msgs::SystemRptFloat>(*brake_rpt);
  shift_rpt_ptr_ = std::make_shared<pacmod_msgs::SystemRptInt>(*shift_rpt);
  global_rpt_ptr_ = std::make_shared<pacmod_msgs::GlobalRpt>(*global_rpt);

  is_pacmod_enabled_ =
    steer_wheel_rpt_ptr_->enabled && accel_rpt_ptr_->enabled && brake_rpt_ptr_->enabled;
  ROS_INFO_COND(
    show_debug_info_,
    "[Pacmod Interface] enabled: is_pacmod_enabled_ %d, steer %d, accel %d, brake %d, shift %d, "
    "global %d",
    is_pacmod_enabled_, steer_wheel_rpt_ptr_->enabled, accel_rpt_ptr_->enabled,
    brake_rpt_ptr_->enabled, shift_rpt_ptr_->enabled, global_rpt_ptr_->enabled);

  const double current_velocity = calculateVehicleVelocity(
    *wheel_speed_rpt_ptr_, *shift_rpt_ptr_);  // current vehicle speed > 0 [m/s]
  const double curr_steer_wheel =
    steer_wheel_rpt_ptr_->output;  // current vehicle steering wheel angle [rad]
  const double adaptive_gear_ratio = calculateVariableGearRatio(current_velocity, curr_steer_wheel);
  const double curr_steer = curr_steer_wheel / adaptive_gear_ratio - steering_offset_;

  std_msgs::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = ros::Time::now();

  /* publish vehicle status control_mode */
  {
    autoware_vehicle_msgs::ControlMode control_mode_msg;
    control_mode_msg.header = header;

    if (!global_rpt->enabled) {
      control_mode_msg.data = autoware_vehicle_msgs::ControlMode::MANUAL;
    } else if (is_pacmod_enabled_) {
      control_mode_msg.data = autoware_vehicle_msgs::ControlMode::AUTO;
    } else if (!steer_wheel_rpt_ptr_->enabled) {
      control_mode_msg.data = autoware_vehicle_msgs::ControlMode::AUTO_PEDAL_ONLY;
    } else {
      control_mode_msg.data = autoware_vehicle_msgs::ControlMode::AUTO_STEER_ONLY;
    }

    control_mode_pub_.publish(control_mode_msg);
  }

  /* publish vehicle status twist */
  geometry_msgs::TwistStamped twist;
  twist.header = header;
  twist.twist.linear.x = current_velocity;                                        // [m/s]
  twist.twist.angular.z = current_velocity * std::tan(curr_steer) / wheel_base_;  // [rad/s]
  vehicle_twist_pub_.publish(twist);

  /* publish current shift */
  autoware_vehicle_msgs::ShiftStamped shift_msg;
  shift_msg.header = header;
  shift_msg.shift.data = toAutowareShiftCmd(*shift_rpt_ptr_);
  shift_status_pub_.publish(shift_msg);

  /* publish current steernig angle */
  autoware_vehicle_msgs::Steering steer_msg;
  steer_msg.header = header;
  steer_msg.data = curr_steer;
  steering_status_pub_.publish(steer_msg);

  autoware_vehicle_msgs::TurnSignal turn_msg;
  turn_msg.header = header;
  turn_msg.data = toAutowareTurnSignal(*turn_rpt);
  turn_signal_status_pub_.publish(turn_msg);
}

void PacmodInterface::publishCommands()
{
  /* guard */
  if (!raw_vehicle_cmd_ptr_ || !is_pacmod_rpt_received_) {
    ROS_INFO_DELAYED_THROTTLE(
      1.0, "[pacmod interface] vehicle_cmd = %d, pacmod_msgs = %d", raw_vehicle_cmd_ptr_ != nullptr,
      is_pacmod_rpt_received_);
    return;
  }

  const ros::Time current_time = ros::Time::now();

  double desired_throttle = raw_vehicle_cmd_ptr_->control.throttle;
  double desired_brake = raw_vehicle_cmd_ptr_->control.brake;

  /* check emergency and timeout */
  const bool emergency = (raw_vehicle_cmd_ptr_->emergency == 1);
  const double vehicle_cmd_delta_time_ms =
    (ros::Time::now() - vehicle_command_received_time_).toSec() * 1000.0;
  const bool timeouted =
    (command_timeout_ms_ >= 0.0) ? (vehicle_cmd_delta_time_ms > command_timeout_ms_) : false;
  if (emergency || timeouted) {
    ROS_ERROR(
      "[pacmod interface] Emergency Stopping, emergency = %d, timeouted = %d", emergency,
      timeouted);
    desired_throttle = 0.0;
    desired_brake = emergency_brake_;
  }

  const double current_velocity = calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *shift_rpt_ptr_);
  const double curr_steer_wheel = steer_wheel_rpt_ptr_->output;

  /* calculate desired steering wheel */
  double adaptive_gear_ratio = calculateVariableGearRatio(current_velocity, curr_steer_wheel);
  double desired_steer_wheel =
    (raw_vehicle_cmd_ptr_->control.steering_angle + steering_offset_) * adaptive_gear_ratio;
  desired_steer_wheel =
    std::min(std::max(desired_steer_wheel, -max_steering_wheel_), max_steering_wheel_);

  /* check clear flag */
  bool clear_override = false;
  if (is_pacmod_enabled_ == true) {
    is_clear_override_needed_ = false;
  } else if (is_clear_override_needed_ == true) {
    clear_override = true;
  }
  ROS_INFO_COND(
    show_debug_info_,
    "[Pacmod Interface] is_pacmod_enabled_ = %d, is_clear_override_needed_ = %d, clear_override = "
    "%d",
    is_pacmod_enabled_, is_clear_override_needed_, clear_override);

  /* check shift change */
  const double brake_for_shift_trans = 0.7;
  uint16_t desired_shift = shift_rpt_ptr_->output;
  if (std::fabs(current_velocity) < 0.1) {  // velocity is low -> the shift can be changed
    if (
      toPacmodShiftCmd(raw_vehicle_cmd_ptr_->shift) !=
      shift_rpt_ptr_->output) {  // need shift change.
      desired_throttle = 0.0;
      desired_brake = brake_for_shift_trans;  // set brake to change the shift
      desired_shift = toPacmodShiftCmd(raw_vehicle_cmd_ptr_->shift);
      ROS_INFO_COND(
        show_debug_info_,
        "[Pacmod Interface] Doing shift change. current = %d, desired = %d. set brake_cmd to %f",
        shift_rpt_ptr_->output, toPacmodShiftCmd(raw_vehicle_cmd_ptr_->shift), desired_brake);
    }
  }

  /* publish accel cmd */
  pacmod_msgs::SystemCmdFloat accel_cmd;
  accel_cmd.header.frame_id = base_frame_id_;
  accel_cmd.header.stamp = current_time;
  accel_cmd.enable = engage_cmd_;
  accel_cmd.ignore_overrides = false;
  accel_cmd.clear_override = clear_override;
  accel_cmd.clear_faults = false;
  accel_cmd.command = std::min(desired_throttle, max_throttle_);
  accel_cmd_pub_.publish(accel_cmd);

  /* publish brake cmd */
  pacmod_msgs::SystemCmdFloat brake_cmd;
  brake_cmd.header.frame_id = base_frame_id_;
  brake_cmd.header.stamp = current_time;
  brake_cmd.enable = engage_cmd_;
  brake_cmd.ignore_overrides = false;
  brake_cmd.clear_override = clear_override;
  brake_cmd.clear_faults = false;
  brake_cmd.command = std::min(desired_brake, max_brake_);
  brake_cmd_pub_.publish(brake_cmd);

  /* publish steering cmd */
  pacmod_msgs::SteerSystemCmd steer_cmd;
  double desired_rotation_rate;  // [rad/s]

  steer_cmd.header.frame_id = base_frame_id_;
  steer_cmd.header.stamp = current_time;
  steer_cmd.enable = engage_cmd_;
  steer_cmd.ignore_overrides = false;
  steer_cmd.clear_override = clear_override;
  steer_cmd.clear_faults = false;
  steer_cmd.command = desired_steer_wheel;
  steer_cmd.rotation_rate = calcSteerWheelRateCmd(adaptive_gear_ratio);
  steer_cmd_pub_.publish(steer_cmd);

  /* publish shift cmd */
  pacmod_msgs::SystemCmdInt shift_cmd;
  shift_cmd.header.frame_id = base_frame_id_;
  shift_cmd.header.stamp = current_time;
  shift_cmd.enable = engage_cmd_;
  shift_cmd.ignore_overrides = false;
  shift_cmd.clear_override = clear_override;
  shift_cmd.clear_faults = false;
  shift_cmd.command = desired_shift;
  shift_cmd_pub_.publish(shift_cmd);

  if (turn_signal_cmd_ptr_) {
    /* publish shift cmd */
    pacmod_msgs::SystemCmdInt turn_cmd;
    turn_cmd.header.frame_id = base_frame_id_;
    turn_cmd.header.stamp = current_time;
    turn_cmd.enable = engage_cmd_;
    turn_cmd.ignore_overrides = false;
    turn_cmd.clear_override = clear_override;
    turn_cmd.clear_faults = false;
    turn_cmd.command = toPacmodTurnCmd(*turn_signal_cmd_ptr_);
    turn_cmd_pub_.publish(turn_cmd);
  }
}

double PacmodInterface::calcSteerWheelRateCmd(const double gear_ratio)
{
  if (!enable_steering_rate_control_) {
    return max_steering_wheel_rate_;
  }

  constexpr double margin = 1.5;
  double rate = margin * raw_vehicle_cmd_ptr_->control.steering_angle_velocity * gear_ratio;
  rate = std::min(std::max(std::fabs(rate), min_steering_wheel_rate_), max_steering_wheel_rate_);
  return rate;
}

double PacmodInterface::calculateVehicleVelocity(
  const pacmod_msgs::WheelSpeedRpt & wheel_speed_rpt, const pacmod_msgs::SystemRptInt & shift_rpt)
{
  double sign = (shift_rpt.output == pacmod_msgs::SystemRptInt::SHIFT_REVERSE) ? -1 : 1;
  double vel = (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) *
               0.5 * tire_radius_;
  return sign * vel;
}

double PacmodInterface::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

uint16_t PacmodInterface::toPacmodShiftCmd(const autoware_vehicle_msgs::Shift & shift)
{
  if (shift.data == autoware_vehicle_msgs::Shift::PARKING) {
    return pacmod_msgs::SystemCmdInt::SHIFT_PARK;
  }
  if (shift.data == autoware_vehicle_msgs::Shift::REVERSE) {
    return pacmod_msgs::SystemCmdInt::SHIFT_REVERSE;
  }
  if (shift.data == autoware_vehicle_msgs::Shift::NEUTRAL) {
    return pacmod_msgs::SystemCmdInt::SHIFT_NEUTRAL;
  }
  if (shift.data == autoware_vehicle_msgs::Shift::DRIVE) {
    return pacmod_msgs::SystemCmdInt::SHIFT_FORWARD;
  }
  if (shift.data == autoware_vehicle_msgs::Shift::LOW) {
    return pacmod_msgs::SystemCmdInt::SHIFT_LOW;
  } else {
    return pacmod_msgs::SystemCmdInt::SHIFT_NONE;
  }
}
int32_t PacmodInterface::toAutowareShiftCmd(const pacmod_msgs::SystemRptInt & shift)
{
  if (shift.output == pacmod_msgs::SystemRptInt::SHIFT_PARK) {
    return autoware_vehicle_msgs::Shift::PARKING;
  }
  if (shift.output == pacmod_msgs::SystemRptInt::SHIFT_REVERSE) {
    return autoware_vehicle_msgs::Shift::REVERSE;
  }
  if (shift.output == pacmod_msgs::SystemRptInt::SHIFT_NEUTRAL) {
    return autoware_vehicle_msgs::Shift::NEUTRAL;
  }
  if (shift.output == pacmod_msgs::SystemRptInt::SHIFT_FORWARD) {
    return autoware_vehicle_msgs::Shift::DRIVE;
  }
  if (shift.output == pacmod_msgs::SystemRptInt::SHIFT_LOW) {
    return autoware_vehicle_msgs::Shift::LOW;
  } else {
    return autoware_vehicle_msgs::Shift::NONE;
  }
}

uint16_t PacmodInterface::toPacmodTurnCmd(const autoware_vehicle_msgs::TurnSignal & turn)
{
  if (turn.data == autoware_vehicle_msgs::TurnSignal::LEFT) {
    return pacmod_msgs::SystemCmdInt::TURN_LEFT;
  } else if (turn.data == autoware_vehicle_msgs::TurnSignal::RIGHT) {
    return pacmod_msgs::SystemCmdInt::TURN_RIGHT;
  } else if (turn.data == autoware_vehicle_msgs::TurnSignal::HAZARD) {
    return pacmod_msgs::SystemCmdInt::TURN_HAZARDS;
  } else {
    return pacmod_msgs::SystemCmdInt::TURN_NONE;
  }
}

int32_t PacmodInterface::toAutowareTurnSignal(const pacmod_msgs::SystemRptInt & turn)
{
  using pacmod_msgs::SystemRptInt;
  if (turn.output == SystemRptInt::TURN_RIGHT) {
    return autoware_vehicle_msgs::TurnSignal::RIGHT;
  } else if (turn.output == SystemRptInt::TURN_LEFT) {
    return autoware_vehicle_msgs::TurnSignal::LEFT;
  } else if (turn.output == SystemRptInt::TURN_NONE) {
    return autoware_vehicle_msgs::TurnSignal::NONE;
  } else if (turn.output == SystemRptInt::TURN_HAZARDS) {
    return autoware_vehicle_msgs::TurnSignal::HAZARD;
  } else {
    return autoware_vehicle_msgs::TurnSignal::NONE;
  }
}
