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

#include <ssc_interface/ssc_interface.hpp>

#include <memory>
#include <string>
#include <utility>

SSCInterface::SSCInterface() : Node("ssc_interface")
{
  using std::placeholders::_1;

  // setup parameters
  use_rear_wheel_speed_ = declare_parameter("use_rear_wheel_speed", true);
  use_adaptive_gear_ratio_ = declare_parameter("use_adaptive_gear_ratio", true);
  command_timeout_ = declare_parameter("command_timeout", 1000);
  loop_rate_ = declare_parameter("loop_rate", 30.0);
  tire_radius_ = declare_parameter("vehicle_info.wheel_radius", 0.5);
  wheel_base_ = declare_parameter("vehicle_info.wheel_base", 4.0);
  ssc_gear_ratio_ = declare_parameter("ssc_gear_ratio", 16.135);
  acceleration_limit_ = declare_parameter("acceleration_limit", 3.0);
  deceleration_limit_ = declare_parameter("deceleration_limit", 3.0);
  max_curvature_rate_ = declare_parameter("max_curvature_rate", 0.15);
  agr_coef_a_ = declare_parameter("agr_coef_a", 15.713);
  agr_coef_b_ = declare_parameter("agr_coef_b", 0.053);
  agr_coef_c_ = declare_parameter("agr_coef_c", 0.042);
  steering_offset_ = declare_parameter("steering_offset", 0.0);

  // subscribers from autoware
  vehicle_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::VehicleCommand>(
    "/control/vehicle_cmd", rclcpp::QoS{1},
    std::bind(&SSCInterface::callbackFromVehicleCmd, this, _1));
  turn_signal_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "/control/turn_signal_cmd", rclcpp::QoS{1},
    std::bind(&SSCInterface::callbackFromTurnSignalCmd, this, _1));
  engage_sub_ = create_subscription<autoware_vehicle_msgs::msg::Engage>(
    "/vehicle/engage", rclcpp::QoS{1}, std::bind(&SSCInterface::callbackFromEngage, this, _1));

  // subscribers from SSC and PACMod
  velocity_accel_cov_sub_ =
    std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::VelocityAccelCov>>(
      this, "as/velocity_accel_cov");
  curvature_feedback_sub_ =
    std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::CurvatureFeedback>>(
      this, "as/curvature_feedback");
  throttle_feedback_sub_ =
    std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::ThrottleFeedback>>(
      this, "as/throttle_feedback");
  brake_feedback_sub_ =
    std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::BrakeFeedback>>(
      this, "as/brake_feedback");
  gear_feedback_sub_ =
    std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::GearFeedback>>(
      this, "as/gear_feedback");
  wheel_speed_sub_ = std::make_unique<message_filters::Subscriber<pacmod_msgs::msg::WheelSpeedRpt>>(
    this, "pacmod/parsed_tx/wheel_speed_rpt");
  steering_wheel_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod_msgs::msg::SystemRptFloat>>(
      this, "pacmod/parsed_tx/steer_rpt");

  ssc_feedbacks_sync_ = std::make_unique<message_filters::Synchronizer<SSCFeedbacksSyncPolicy>>(
    SSCFeedbacksSyncPolicy(10), *velocity_accel_cov_sub_, *curvature_feedback_sub_,
    *throttle_feedback_sub_, *brake_feedback_sub_, *gear_feedback_sub_, *wheel_speed_sub_,
    *steering_wheel_sub_);

  ssc_feedbacks_sync_->registerCallback(std::bind(
    &SSCInterface::callbackFromSSCFeedbacks, this, _1, std::placeholders::_2, std::placeholders::_3,
    std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7));

  module_states_sub_ = create_subscription<automotive_navigation_msgs::msg::ModuleState>(
    "as/module_states", rclcpp::QoS{1},
    std::bind(&SSCInterface::callbackFromSSCModuleStates, this, _1));

  // TEMP from pacmod
  pacmod_turn_sub_ = create_subscription<pacmod_msgs::msg::SystemRptInt>(
    "/pacmod/parsed_tx/turn_rpt", rclcpp::QoS{1},
    std::bind(&SSCInterface::callbackTurnSignal, this, _1));

  // publishers to autoware
  control_mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlMode>(
    "/vehicle/status/control_mode", rclcpp::QoS{10});
  current_shift_pub_ = create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>(
    "/vehicle/status/shift", rclcpp::QoS{10});
  current_twist_pub_ =
    create_publisher<geometry_msgs::msg::TwistStamped>("/vehicle/status/twist", rclcpp::QoS{10});
  current_steer_pub_ = create_publisher<autoware_vehicle_msgs::msg::Steering>(
    "/vehicle/status/steering", rclcpp::QoS{10});
  current_steer_wheel_deg_pub_ = create_publisher<autoware_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/steering_wheel_deg", rclcpp::QoS{10});
  current_velocity_pub_ = create_publisher<autoware_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/velocity", rclcpp::QoS{10});
  current_velocity_kmph_pub_ = create_publisher<autoware_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/velocity_kmph", rclcpp::QoS{10});
  current_turn_signal_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnSignal>(
    "/vehicle/status/turn_signal", rclcpp::QoS{10});

  // publishers to SSC
  steer_mode_pub_ = create_publisher<automotive_platform_msgs::msg::SteerMode>(
    "as/arbitrated_steering_commands", rclcpp::QoS{10});
  speed_mode_pub_ = create_publisher<automotive_platform_msgs::msg::SpeedMode>(
    "as/arbitrated_speed_commands", rclcpp::QoS{10});
  turn_signal_pub_ = create_publisher<automotive_platform_msgs::msg::TurnSignalCommand>(
    "as/turn_signal_command", rclcpp::QoS{10});
  rclcpp::QoS durable_qos{10};  // to latch the topic
  durable_qos.transient_local();
  gear_pub_ =
    create_publisher<automotive_platform_msgs::msg::GearCommand>("as/gear_select", durable_qos);

  // Timer
  auto timer_callback = std::bind(&SSCInterface::publishCommand, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / loop_rate_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

SSCInterface::~SSCInterface() {}

void SSCInterface::callbackFromVehicleCmd(
  const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg)
{
  command_time_ = get_clock()->now();
  vehicle_cmd_ = *msg;
  command_initialized_ = true;
}
void SSCInterface::callbackFromTurnSignalCmd(
  const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  turn_signal_cmd_ = *msg;
  turn_signal_cmd_initialized_ = true;
}
void SSCInterface::callbackFromEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  engage_ = msg->engage;
}

void SSCInterface::callbackFromSSCModuleStates(
  const automotive_navigation_msgs::msg::ModuleState::ConstSharedPtr msg)
{
  if (msg->name.find("veh_controller") != std::string::npos) {
    module_states_ = *msg;  // *_veh_controller status is used for 'drive/steeringmode'
  }
}

void SSCInterface::callbackFromSSCFeedbacks(
  const automotive_platform_msgs::msg::VelocityAccelCov::ConstSharedPtr msg_velocity,
  const automotive_platform_msgs::msg::CurvatureFeedback::ConstSharedPtr msg_curvature,
  const automotive_platform_msgs::msg::ThrottleFeedback::ConstSharedPtr msg_throttle,
  const automotive_platform_msgs::msg::BrakeFeedback::ConstSharedPtr msg_brake,
  const automotive_platform_msgs::msg::GearFeedback::ConstSharedPtr msg_gear,
  const pacmod_msgs::msg::WheelSpeedRpt::ConstSharedPtr msg_wheel_speed,
  const pacmod_msgs::msg::SystemRptFloat::ConstSharedPtr msg_steering_wheel)
{
  std_msgs::msg::Header published_msgs_header;
  published_msgs_header.frame_id = BASE_FRAME_ID;
  published_msgs_header.stamp = msg_velocity->header.stamp;

  wheel_speed_rpt_ptr_ = msg_wheel_speed;
  vel_acc_cov_ptr_ = msg_velocity;
  gear_feedback_ptr_ = msg_gear;

  // current speed
  double speed = calculateVehicleVelocity(
    *wheel_speed_rpt_ptr_, *vel_acc_cov_ptr_, *gear_feedback_ptr_, use_rear_wheel_speed_);

  // update adaptive gear ratio (avoiding zero division)
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
  //   (-msg_wheel_speed->rear_right_wheel_speed + msg_wheel_speed->rear_left_wheel_speed) *
  //   tire_radius_ / tread;

  // as_current_velocity (geometry_msgs::msg::TwistStamped)
  geometry_msgs::msg::TwistStamped twist;
  twist.header = published_msgs_header;
  twist.twist.linear.x = speed;               // [m/s]
  twist.twist.angular.z = curvature * speed;  // [rad/s]
  current_twist_pub_->publish(twist);

  // gearshift
  autoware_vehicle_msgs::msg::ShiftStamped shift_msg;
  shift_msg.header = published_msgs_header;
  if (msg_gear->current_gear.gear == automotive_platform_msgs::msg::Gear::NONE) {
    shift_msg.shift.data = autoware_vehicle_msgs::msg::Shift::NONE;
  } else if (msg_gear->current_gear.gear == automotive_platform_msgs::msg::Gear::PARK) {
    shift_msg.shift.data = autoware_vehicle_msgs::msg::Shift::PARKING;
  } else if (msg_gear->current_gear.gear == automotive_platform_msgs::msg::Gear::REVERSE) {
    shift_msg.shift.data = autoware_vehicle_msgs::msg::Shift::REVERSE;
  } else if (msg_gear->current_gear.gear == automotive_platform_msgs::msg::Gear::NEUTRAL) {
    shift_msg.shift.data = autoware_vehicle_msgs::msg::Shift::NEUTRAL;
  } else if (msg_gear->current_gear.gear == automotive_platform_msgs::msg::Gear::DRIVE) {
    shift_msg.shift.data = autoware_vehicle_msgs::msg::Shift::DRIVE;
  }
  current_shift_pub_->publish(shift_msg);

  // control mode
  autoware_vehicle_msgs::msg::ControlMode mode;
  mode.header = published_msgs_header;
  mode.data = (module_states_.state == "active") ? autoware_vehicle_msgs::msg::ControlMode::AUTO
                                                 : autoware_vehicle_msgs::msg::ControlMode::MANUAL;
  control_mode_pub_->publish(mode);

  // steering
  autoware_vehicle_msgs::msg::Steering steer;
  steer.header = published_msgs_header;
  steer.data = steering_angle - steering_offset_;
  current_steer_pub_->publish(steer);
}

void SSCInterface::publishCommand()
{
  /* guard */
  if (!command_initialized_ || !wheel_speed_rpt_ptr_ || !vel_acc_cov_ptr_ || !gear_feedback_ptr_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "vehicle_cmd = %d, wheel_speed_rpt = %d, vel_acc_cov = %d, gear_feedback = %d",
      command_initialized_, wheel_speed_rpt_ptr_ != nullptr, vel_acc_cov_ptr_ != nullptr,
      gear_feedback_ptr_ != nullptr);
    return;
  }

  rclcpp::Time stamp = get_clock()->now();

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
  unsigned char desired_turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::NONE;

  if (turn_signal_cmd_initialized_) {
    if (turn_signal_cmd_.data == autoware_vehicle_msgs::msg::TurnSignal::NONE) {
      desired_turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::NONE;
    } else if (turn_signal_cmd_.data == autoware_vehicle_msgs::msg::TurnSignal::LEFT) {
      desired_turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::LEFT;
    } else if (turn_signal_cmd_.data == autoware_vehicle_msgs::msg::TurnSignal::RIGHT) {
      desired_turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::RIGHT;
    } else if (turn_signal_cmd_.data == autoware_vehicle_msgs::msg::TurnSignal::HAZARD) {
      // NOTE: HAZARD signal cannot be used in automotive_platform_msgs::msg::TurnSignalCommand
    }
  }

  // Override desired speed to ZERO by emergency/timeout
  bool emergency = (vehicle_cmd_.emergency == 1);
  bool timed_out = (((get_clock()->now() - command_time_).seconds() * 1000) > command_timeout_);

  if (emergency) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Emergency Stopping, emergency = %d, timed_out = %d", emergency, timed_out);
    desired_speed = 0.0;
    deceleration_limit = 0.0;
  } else if (timed_out) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Timeout Stopping, emergency = %d, timed_out = %d", emergency, timed_out);
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
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
        "Doing shift change. current = %d, desired = %d. set quick stop mode",
        gear_feedback_ptr_->current_gear.gear, toSSCShiftCmd(vehicle_cmd_.shift));
    }
  }

  // speed command
  automotive_platform_msgs::msg::SpeedMode speed_mode;
  speed_mode.header.frame_id = BASE_FRAME_ID;
  speed_mode.header.stamp = stamp;
  speed_mode.mode = desired_mode;
  speed_mode.speed = desired_speed;
  speed_mode.acceleration_limit = acceleration_limit_;
  speed_mode.deceleration_limit = deceleration_limit;

  // steer command
  automotive_platform_msgs::msg::SteerMode steer_mode;
  steer_mode.header.frame_id = BASE_FRAME_ID;
  steer_mode.header.stamp = stamp;
  steer_mode.mode = desired_mode;
  steer_mode.curvature = desired_curvature;
  steer_mode.max_curvature_rate = max_curvature_rate_;

  // turn signal command
  automotive_platform_msgs::msg::TurnSignalCommand turn_signal;
  turn_signal.header.frame_id = BASE_FRAME_ID;
  turn_signal.header.stamp = stamp;
  turn_signal.mode = desired_mode;
  turn_signal.turn_signal = desired_turn_signal;

  // gear_cmd command
  automotive_platform_msgs::msg::GearCommand gear_cmd;
  gear_cmd.header.frame_id = BASE_FRAME_ID;
  gear_cmd.header.stamp = stamp;
  gear_cmd.command.gear = desired_shift;

  // publish
  speed_mode_pub_->publish(speed_mode);
  steer_mode_pub_->publish(steer_mode);
  turn_signal_pub_->publish(turn_signal);
  gear_pub_->publish(gear_cmd);
}

double SSCInterface::calculateVehicleVelocity(
  const pacmod_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
  const automotive_platform_msgs::msg::VelocityAccelCov & vel_acc_cov,
  const automotive_platform_msgs::msg::GearFeedback & gear_feedback,
  const bool use_rear_wheel_speed)
{
  const auto rear_wheel_speed =
    (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) *
    tire_radius_ / 2.0;
  double speed = !use_rear_wheel_speed ? vel_acc_cov.velocity : rear_wheel_speed;
  speed = std::abs(speed);
  if (gear_feedback.current_gear.gear == automotive_platform_msgs::msg::Gear::REVERSE) {
    speed *= -1.0;
  }
  return speed;
}

uint8_t SSCInterface::toSSCShiftCmd(const autoware_vehicle_msgs::msg::Shift & shift)
{
  using automotive_platform_msgs::msg::Gear;
  using autoware_vehicle_msgs::msg::Shift;

  if (shift.data == Shift::PARKING) {
    return Gear::PARK;
  }
  if (shift.data == Shift::REVERSE) {
    return Gear::REVERSE;
  }
  if (shift.data == Shift::NEUTRAL) {
    return Gear::NEUTRAL;
  }
  if (shift.data == Shift::DRIVE) {
    return Gear::DRIVE;
  }
  if (shift.data == Shift::LOW) {
    return Gear::LOW;
  }

  return Gear::NONE;
}

void SSCInterface::callbackTurnSignal(const pacmod_msgs::msg::SystemRptInt::ConstSharedPtr turn)
{
  autoware_vehicle_msgs::msg::TurnSignal cmd;
  cmd.header.stamp = turn->header.stamp;
  cmd.header.frame_id = BASE_FRAME_ID;
  cmd.data = toAutowareTurnSignal(*turn);
  current_turn_signal_pub_->publish(cmd);
}

// TEMP for pacmod turn_signal status
int32_t SSCInterface::toAutowareTurnSignal(const pacmod_msgs::msg::SystemRptInt & turn) const
{
  using autoware_vehicle_msgs::msg::TurnSignal;
  using pacmod_msgs::msg::SystemRptInt;

  if (turn.output == SystemRptInt::TURN_RIGHT) {
    return TurnSignal::RIGHT;
  }
  if (turn.output == SystemRptInt::TURN_LEFT) {
    return TurnSignal::LEFT;
  }
  if (turn.output == SystemRptInt::TURN_NONE) {
    return TurnSignal::NONE;
  }
  if (turn.output == SystemRptInt::TURN_HAZARDS) {
    return TurnSignal::HAZARD;
  }

  return TurnSignal::NONE;
}
