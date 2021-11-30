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

#include "ssc_interface/ssc_interface.h"

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

SSCInterface::SSCInterface()
: nh_(),
  private_nh_("~"),
  engage_(false),
  command_initialized_(false),
  turn_signal_cmd_initialized_(false)
{
  // setup parameters
  private_nh_.param<bool>("use_rear_wheel_speed", use_rear_wheel_speed_, true);
  private_nh_.param<bool>("use_adaptive_gear_ratio", use_adaptive_gear_ratio_, true);
  private_nh_.param<int>("command_timeout", command_timeout_, 1000);
  private_nh_.param<double>("loop_rate", loop_rate_, 30.0);
  tire_radius_ = waitForParam<double>(private_nh_, "/vehicle_info/wheel_radius");
  wheel_base_ = waitForParam<double>(private_nh_, "/vehicle_info/wheel_base");
  private_nh_.param<double>("ssc_gear_ratio", ssc_gear_ratio_, 16.135);
  private_nh_.param<double>("acceleration_limit", acceleration_limit_, 3.0);
  private_nh_.param<double>("deceleration_limit", deceleration_limit_, 3.0);
  private_nh_.param<double>("max_curvature_rate", max_curvature_rate_, 0.15);
  private_nh_.param<double>("agr_coef_a", agr_coef_a_, 15.713);
  private_nh_.param<double>("agr_coef_b", agr_coef_b_, 0.053);
  private_nh_.param<double>("agr_coef_c", agr_coef_c_, 0.042);
  private_nh_.param<double>("steering_offset", steering_offset_, 0.0);

  rate_ = new ros::Rate(loop_rate_);

  // subscribers from autoware
  vehicle_cmd_sub_ =
    nh_.subscribe("/control/vehicle_cmd", 1, &SSCInterface::callbackFromVehicleCmd, this);
  turn_signal_cmd_sub_ =
    nh_.subscribe("/control/turn_signal_cmd", 1, &SSCInterface::callbackFromTurnSignalCmd, this);
  engage_sub_ = nh_.subscribe("vehicle/engage", 1, &SSCInterface::callbackFromEngage, this);

  // subscribers from SSC
  module_states_sub_ =
    nh_.subscribe("as/module_states", 1, &SSCInterface::callbackFromSSCModuleStates, this);
  curvature_feedback_sub_ =
    new message_filters::Subscriber<automotive_platform_msgs::CurvatureFeedback>(
      nh_, "as/curvature_feedback", 10);
  throttle_feedback_sub_ =
    new message_filters::Subscriber<automotive_platform_msgs::ThrottleFeedback>(
      nh_, "as/throttle_feedback", 10);
  brake_feedback_sub_ = new message_filters::Subscriber<automotive_platform_msgs::BrakeFeedback>(
    nh_, "as/brake_feedback", 10);
  gear_feedback_sub_ = new message_filters::Subscriber<automotive_platform_msgs::GearFeedback>(
    nh_, "as/gear_feedback", 10);
  velocity_accel_cov_sub_ =
    new message_filters::Subscriber<automotive_platform_msgs::VelocityAccelCov>(
      nh_, "as/velocity_accel_cov", 10);
  // subscribers from PACMod
  wheel_speed_sub_ = new message_filters::Subscriber<pacmod_msgs::WheelSpeedRpt>(
    nh_, "pacmod/parsed_tx/wheel_speed_rpt", 10);
  steering_wheel_sub_ = new message_filters::Subscriber<pacmod_msgs::SystemRptFloat>(
    nh_, "pacmod/parsed_tx/steer_rpt", 10);
  ssc_feedbacks_sync_ = new message_filters::Synchronizer<SSCFeedbacksSyncPolicy>(
    SSCFeedbacksSyncPolicy(10), *velocity_accel_cov_sub_, *curvature_feedback_sub_,
    *throttle_feedback_sub_, *brake_feedback_sub_, *gear_feedback_sub_, *wheel_speed_sub_,
    *steering_wheel_sub_);
  ssc_feedbacks_sync_->registerCallback(
    boost::bind(&SSCInterface::callbackFromSSCFeedbacks, this, _1, _2, _3, _4, _5, _6, _7));

  pacmod_turn_sub_ = nh_.subscribe("/pacmod/parsed_tx/turn_rpt", 1, &SSCInterface::callbackTurnSignal, this);

  // publishers to autoware
  control_mode_pub_ =
    nh_.advertise<autoware_vehicle_msgs::ControlMode>("/vehicle/status/control_mode", 10);
  current_shift_pub_ =
    nh_.advertise<autoware_vehicle_msgs::ShiftStamped>("/vehicle/status/shift", 10);
  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vehicle/status/twist", 10);
  current_steer_pub_ =
    nh_.advertise<autoware_vehicle_msgs::Steering>("/vehicle/status/steering", 10);
  current_steer_wheel_deg_pub_ =
    nh_.advertise<std_msgs::Float32>("/vehicle/status/steering_wheel_deg", 10);
  current_velocity_pub_ = nh_.advertise<std_msgs::Float32>("/vehicle/status/velocity", 10);
  current_velocity_kmph_pub_ =
    nh_.advertise<std_msgs::Float32>("/vehicle/status/velocity_kmph", 10);
  current_turn_signal_pub_ = nh_.advertise<autoware_vehicle_msgs::TurnSignal>("/vehicle/status/turn_signal", 10);

  // publishers to SSC
  steer_mode_pub_ =
    nh_.advertise<automotive_platform_msgs::SteerMode>("as/arbitrated_steering_commands", 10);
  speed_mode_pub_ =
    nh_.advertise<automotive_platform_msgs::SpeedMode>("as/arbitrated_speed_commands", 10);
  turn_signal_pub_ =
    nh_.advertise<automotive_platform_msgs::TurnSignalCommand>("as/turn_signal_command", 10);
  gear_pub_ = nh_.advertise<automotive_platform_msgs::GearCommand>("as/gear_select", 1, true);
}

SSCInterface::~SSCInterface() {}

void SSCInterface::run()
{
  while (ros::ok()) {
    ros::spinOnce();
    publishCommand();
    rate_->sleep();
  }
}

void SSCInterface::callbackFromVehicleCmd(const autoware_vehicle_msgs::VehicleCommandConstPtr & msg)
{
  command_time_ = ros::Time::now();
  vehicle_cmd_ = *msg;
  command_initialized_ = true;
}
void SSCInterface::callbackFromTurnSignalCmd(const autoware_vehicle_msgs::TurnSignalConstPtr & msg)
{
  turn_signal_cmd_ = *msg;
  turn_signal_cmd_initialized_ = true;
}
void SSCInterface::callbackFromEngage(const std_msgs::BoolConstPtr & msg) { engage_ = msg->data; }

void SSCInterface::callbackFromSSCModuleStates(
  const automotive_navigation_msgs::ModuleStateConstPtr & msg)
{
  if (msg->name.find("veh_controller") != std::string::npos) {
    module_states_ = *msg;  // *_veh_controller status is used for 'drive/steeringmode'
  }
}

void SSCInterface::callbackFromSSCFeedbacks(
  const automotive_platform_msgs::VelocityAccelCovConstPtr & msg_velocity,
  const automotive_platform_msgs::CurvatureFeedbackConstPtr & msg_curvature,
  const automotive_platform_msgs::ThrottleFeedbackConstPtr & msg_throttle,
  const automotive_platform_msgs::BrakeFeedbackConstPtr & msg_brake,
  const automotive_platform_msgs::GearFeedbackConstPtr & msg_gear,
  const pacmod_msgs::WheelSpeedRptConstPtr & msg_wheel_speed,
  const pacmod_msgs::SystemRptFloatConstPtr & msg_steering_wheel)
{
  std_msgs::Header published_msgs_header;
  published_msgs_header.frame_id = BASE_FRAME_ID;
  published_msgs_header.stamp = msg_velocity->header.stamp;

  wheel_speed_rpt_ptr_ = msg_wheel_speed;
  vel_acc_cov_ptr_ = msg_velocity;
  gear_feedback_ptr_ = msg_gear;

  // current speed
  double speed = calculateVehicleVelocity(
    *wheel_speed_rpt_ptr_, *vel_acc_cov_ptr_, *gear_feedback_ptr_, use_rear_wheel_speed_);

  // update adaptive gear ratio (avoiding zero divizion)
  adaptive_gear_ratio_ = std::max(
    1e-5, agr_coef_a_ + agr_coef_b_ * speed * speed -
            agr_coef_c_ * std::fabs(msg_steering_wheel->output));
  // current steering curvature
  double curvature =
    !use_adaptive_gear_ratio_
      ? (msg_curvature->curvature)
      : std::tan(msg_steering_wheel->output / adaptive_gear_ratio_ - steering_offset_) /
          wheel_base_;
  const double steering_angle = std::atan(curvature * wheel_base_);
  // constexpr double tread = 1.64;  // spec sheet 1.63
  // double omega =
  //   (-msg_wheel_speed->rear_right_wheel_speed + msg_wheel_speed->rear_left_wheel_speed) * tire_radius_ / tread;

  // as_current_velocity (geometry_msgs::TwistStamped)
  geometry_msgs::TwistStamped twist;
  twist.header = published_msgs_header;
  twist.twist.linear.x = speed;               // [m/s]
  twist.twist.angular.z = curvature * speed;  // [rad/s]
  current_twist_pub_.publish(twist);

  // gearshift
  autoware_vehicle_msgs::ShiftStamped shift_msg;
  shift_msg.header = published_msgs_header;
  if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::NONE) {
    shift_msg.shift.data = autoware_vehicle_msgs::Shift::NONE;
  } else if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::PARK) {
    shift_msg.shift.data = autoware_vehicle_msgs::Shift::PARKING;
  } else if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::REVERSE) {
    shift_msg.shift.data = autoware_vehicle_msgs::Shift::REVERSE;
  } else if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::NEUTRAL) {
    shift_msg.shift.data = autoware_vehicle_msgs::Shift::NEUTRAL;
  } else if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::DRIVE) {
    shift_msg.shift.data = autoware_vehicle_msgs::Shift::DRIVE;
  }
  current_shift_pub_.publish(shift_msg);

  // control mode
  autoware_vehicle_msgs::ControlMode mode;
  mode.header = published_msgs_header;
  mode.data = (module_states_.state == "active") ? autoware_vehicle_msgs::ControlMode::AUTO
                                                 : autoware_vehicle_msgs::ControlMode::MANUAL;
  control_mode_pub_.publish(mode);

  // steering
  autoware_vehicle_msgs::Steering steer;
  steer.header = published_msgs_header;
  steer.data = steering_angle - steering_offset_;
  current_steer_pub_.publish(steer);
}

void SSCInterface::publishCommand()
{
  /* guard */
  if (!command_initialized_ || !wheel_speed_rpt_ptr_ || !vel_acc_cov_ptr_ || !gear_feedback_ptr_) {
    ROS_INFO_DELAYED_THROTTLE(
      1.0, "vehicle_cmd = %d, wheel_speed_rpt = %d, vel_acc_cov = %d, gear_feedback = %d",
      command_initialized_, wheel_speed_rpt_ptr_ != nullptr, vel_acc_cov_ptr_ != nullptr,
      gear_feedback_ptr_ != nullptr);
    return;
  }

  ros::Time stamp = ros::Time::now();

  // Desired values
  // Driving mode (If autonomy mode should be active, mode = 1)
  unsigned char desired_mode = engage_ ? 1 : 0;

  // Speed for SSC speed_model
  double desired_speed = vehicle_cmd_.control.velocity;
  double deceleration_limit = deceleration_limit_;

  // Curvature for SSC steer_model
  double desired_steering_angle = !use_adaptive_gear_ratio_
                                    ? vehicle_cmd_.control.steering_angle + steering_offset_
                                    : (vehicle_cmd_.control.steering_angle + steering_offset_) *
                                        ssc_gear_ratio_ / adaptive_gear_ratio_;
  double desired_curvature = std::tan(desired_steering_angle) / wheel_base_;

  // Turn signal
  unsigned char desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;

  if (turn_signal_cmd_initialized_) {
    if (turn_signal_cmd_.data == autoware_vehicle_msgs::TurnSignal::NONE) {
      desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
    } else if (turn_signal_cmd_.data == autoware_vehicle_msgs::TurnSignal::LEFT) {
      desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::LEFT;
    } else if (turn_signal_cmd_.data == autoware_vehicle_msgs::TurnSignal::RIGHT) {
      desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::RIGHT;
    } else if (turn_signal_cmd_.data == autoware_vehicle_msgs::TurnSignal::HAZARD) {
      // NOTE: HAZARD signal cannot be used in automotive_platform_msgs::TurnSignalCommand
    }
  }

  // Override desired speed to ZERO by emergency/timeout
  bool emergency = (vehicle_cmd_.emergency == 1);
  bool timeouted = (((ros::Time::now() - command_time_).toSec() * 1000) > command_timeout_);

  if (emergency) {
    ROS_ERROR_THROTTLE(
      1.0, "Emergency Stopping, emergency = %d, timeouted = %d", emergency, timeouted);
    desired_speed = 0.0;
    deceleration_limit = 0.0;
  } else if (timeouted) {
    ROS_ERROR_THROTTLE(
      1.0, "Timeout Stopping, emergency = %d, timeouted = %d", emergency, timeouted);
    desired_speed = 0.0;
  }

  /* check shift change */
  double current_velocity = calculateVehicleVelocity(
    *wheel_speed_rpt_ptr_, *vel_acc_cov_ptr_, *gear_feedback_ptr_, use_rear_wheel_speed_);
  uint8_t desired_shift = gear_feedback_ptr_->current_gear.gear;
  if (std::abs(current_velocity) < 0.1) {  // velocity is low -> the shift can be changed
    if (toSSCShiftCmd(vehicle_cmd_.shift) != gear_feedback_ptr_->current_gear.gear) {
      desired_speed = 0.0;
      deceleration_limit = 0.0;  // set quick stop mode to change the shift
      desired_shift = toSSCShiftCmd(vehicle_cmd_.shift);
      ROS_INFO_THROTTLE(
        1.0, "Doing shift change. current = %d, desired = %d. set quick stop mode",
        gear_feedback_ptr_->current_gear.gear, toSSCShiftCmd(vehicle_cmd_.shift));
    }
  }

  // speed command
  automotive_platform_msgs::SpeedMode speed_mode;
  speed_mode.header.frame_id = BASE_FRAME_ID;
  speed_mode.header.stamp = stamp;
  speed_mode.mode = desired_mode;
  speed_mode.speed = desired_speed;
  speed_mode.acceleration_limit = acceleration_limit_;
  speed_mode.deceleration_limit = deceleration_limit;

  // steer command
  automotive_platform_msgs::SteerMode steer_mode;
  steer_mode.header.frame_id = BASE_FRAME_ID;
  steer_mode.header.stamp = stamp;
  steer_mode.mode = desired_mode;
  steer_mode.curvature = desired_curvature;
  steer_mode.max_curvature_rate = max_curvature_rate_;

  // turn signal command
  automotive_platform_msgs::TurnSignalCommand turn_signal;
  turn_signal.header.frame_id = BASE_FRAME_ID;
  turn_signal.header.stamp = stamp;
  turn_signal.mode = desired_mode;
  turn_signal.turn_signal = desired_turn_signal;

  // gear_cmd command
  automotive_platform_msgs::GearCommand gear_cmd;
  gear_cmd.header.frame_id = BASE_FRAME_ID;
  gear_cmd.header.stamp = stamp;
  gear_cmd.command.gear = desired_shift;

  // publish
  speed_mode_pub_.publish(speed_mode);
  steer_mode_pub_.publish(steer_mode);
  turn_signal_pub_.publish(turn_signal);
  gear_pub_.publish(gear_cmd);

  ROS_INFO_STREAM_THROTTLE(
    1.0, "Mode: " << (int)desired_mode << ", "
                  << "Speed: " << speed_mode.speed << ", "
                  << "Curvature: " << steer_mode.curvature << ", "
                  << "Gear: " << (int)gear_cmd.command.gear << ", "
                  << "TurnSignal: " << (int)turn_signal.turn_signal);
}

double SSCInterface::calculateVehicleVelocity(
  const pacmod_msgs::WheelSpeedRpt & wheel_speed_rpt,
  const automotive_platform_msgs::VelocityAccelCov & vel_acc_cov,
  const automotive_platform_msgs::GearFeedback & gear_feedback, const bool use_rear_wheel_speed)
{
  const auto rear_wheel_speed =
    (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) *
    tire_radius_ / 2.0;
  double speed = !use_rear_wheel_speed ? vel_acc_cov.velocity : rear_wheel_speed;
  speed = std::abs(speed);
  if (gear_feedback.current_gear.gear == automotive_platform_msgs::Gear::REVERSE) speed *= -1.0;
  return speed;
}

uint8_t SSCInterface::toSSCShiftCmd(const autoware_vehicle_msgs::Shift & shift)
{
  using automotive_platform_msgs::Gear;
  using autoware_vehicle_msgs::Shift;

  if (shift.data == Shift::PARKING) return Gear::PARK;
  if (shift.data == Shift::REVERSE) return Gear::REVERSE;
  if (shift.data == Shift::NEUTRAL) return Gear::NEUTRAL;
  if (shift.data == Shift::DRIVE) return Gear::DRIVE;
  if (shift.data == Shift::LOW) return Gear::LOW;

  return Gear::NONE;
}

void SSCInterface::callbackTurnSignal(const pacmod_msgs::SystemRptInt & turn) {
  autoware_vehicle_msgs::TurnSignal cmd;
  cmd.header.stamp = turn.header.stamp;
  cmd.header.frame_id = BASE_FRAME_ID;
  cmd.data = toAutowareTurnSignal(turn);
  current_turn_signal_pub_.publish(cmd);
}

// TEMP for pacmod turn_signal status
int32_t SSCInterface::toAutowareTurnSignal(const pacmod_msgs::SystemRptInt & turn) const
{
  using autoware_vehicle_msgs::TurnSignal;
  using pacmod_msgs::SystemRptInt;

  if (turn.output == SystemRptInt::TURN_RIGHT) return TurnSignal::RIGHT;
  if (turn.output == SystemRptInt::TURN_LEFT) return TurnSignal::LEFT;
  if (turn.output == SystemRptInt::TURN_NONE) return TurnSignal::NONE;
  if (turn.output == SystemRptInt::TURN_HAZARDS) return TurnSignal::HAZARD;

  return TurnSignal::NONE;
}
