// Copyright 2015-2019 Autoware Foundation
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"

#include "autoware_api_utils/autoware_api_utils.hpp"
#include "vehicle_cmd_gate/vehicle_cmd_gate.hpp"

namespace
{
void fillFrameId(std::string * frame_id, const std::string & name)
{
  if (*frame_id == "") {
    *frame_id = name;
  }
}

const char * getGateModeName(const autoware_control_msgs::msg::GateMode::_data_type & gate_mode)
{
  using autoware_control_msgs::msg::GateMode;

  if (gate_mode == GateMode::AUTO) {
    return "AUTO";
  }
  if (gate_mode == GateMode::EXTERNAL) {
    return "EXTERNAL";
  }
  return "NOT_SUPPORTED";
}

}  // namespace

VehicleCmdGate::VehicleCmdGate(const rclcpp::NodeOptions & node_options)
: Node("vehicle_cmd_gate", node_options),
  is_engaged_(false), updater_(this)
{
  using namespace std::placeholders;

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  // Publisher
  vehicle_cmd_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VehicleCommand>(
    "output/vehicle_cmd", durable_qos);
  control_cmd_pub_ = this->create_publisher<autoware_control_msgs::msg::ControlCommandStamped>(
    "output/control_cmd", durable_qos);
  shift_cmd_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>(
    "output/shift_cmd", durable_qos);
  turn_signal_cmd_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::TurnSignal>(
    "output/turn_signal_cmd", durable_qos);
  gate_mode_pub_ = this->create_publisher<autoware_control_msgs::msg::GateMode>(
    "output/gate_mode", durable_qos);
  engage_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::Engage>(
    "output/engage", durable_qos);

  // Subscriber
  system_emergency_sub_ = this->create_subscription<autoware_control_msgs::msg::EmergencyMode>(
    "input/system_emergency", 1, std::bind(&VehicleCmdGate::onSystemEmergency, this, _1));
  external_emergency_stop_sub_ =
    this->create_subscription<autoware_control_msgs::msg::EmergencyMode>(
    "input/external_emergency_stop", 1,
    std::bind(&VehicleCmdGate::onExternalEmergencyStop, this, _1));
  gate_mode_sub_ = this->create_subscription<autoware_control_msgs::msg::GateMode>(
    "input/gate_mode", 1, std::bind(&VehicleCmdGate::onGateMode, this, _1));
  engage_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::Engage>(
    "input/engage", 1, std::bind(&VehicleCmdGate::onEngage, this, _1));
  steer_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::Steering>(
    "input/steering", 1, std::bind(&VehicleCmdGate::onSteering, this, _1));

  // Subscriber for auto
  auto_control_cmd_sub_ =
    this->create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
    "input/auto/control_cmd", 1, std::bind(&VehicleCmdGate::onAutoCtrlCmd, this, _1));
  auto_turn_signal_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "input/auto/turn_signal_cmd", 1, std::bind(&VehicleCmdGate::onAutoTurnSignalCmd, this, _1));
  auto_shift_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "input/auto/shift_cmd", 1, std::bind(&VehicleCmdGate::onAutoShiftCmd, this, _1));

  // Subscriber for external
  remote_control_cmd_sub_ =
    this->create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
    "input/external/control_cmd", 1,
    std::bind(&VehicleCmdGate::onRemoteCtrlCmd, this, _1));
  remote_turn_signal_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "input/external/turn_signal_cmd", 1,
    std::bind(&VehicleCmdGate::onRemoteTurnSignalCmd, this, _1));
  remote_shift_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "input/external/shift_cmd", 1,
    std::bind(&VehicleCmdGate::onRemoteShiftCmd, this, _1));

  // Subscriber for emergency
  emergency_control_cmd_sub_ =
    this->create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
    "input/emergency/control_cmd", 1, std::bind(&VehicleCmdGate::onEmergencyCtrlCmd, this, _1));
  emergency_turn_signal_cmd_sub_ =
    this->create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "input/emergency/turn_signal_cmd", 1,
    std::bind(&VehicleCmdGate::onEmergencyTurnSignalCmd, this, _1));
  emergency_shift_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "input/emergency/shift_cmd", 1, std::bind(&VehicleCmdGate::onEmergencyShiftCmd, this, _1));

  // Parameter
  update_period_ = 1.0 / declare_parameter("update_rate", 10.0);
  use_emergency_handling_ = declare_parameter("use_emergency_handling", false);
  use_external_emergency_stop_ = declare_parameter("use_external_emergency_stop", false);
  system_emergency_heartbeat_timeout_ =
    declare_parameter("system_emergency_heartbeat_timeout", 0.5);
  external_emergency_stop_heartbeat_timeout_ =
    declare_parameter("external_emergency_stop_heartbeat_timeout", 0.5);

  // Vehicle Parameter
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  double wheel_base = vehicle_info.wheel_base_m;
  double vel_lim = declare_parameter("vel_lim", 25.0);
  double lon_acc_lim = declare_parameter("lon_acc_lim", 5.0);
  double lon_jerk_lim = declare_parameter("lon_jerk_lim", 5.0);
  double lat_acc_lim = declare_parameter("lat_acc_lim", 5.0);
  double lat_jerk_lim = declare_parameter("lat_jerk_lim", 5.0);
  filter_.setWheelBase(wheel_base);
  filter_.setVelLim(vel_lim);
  filter_.setLonAccLim(lon_acc_lim);
  filter_.setLonJerkLim(lon_jerk_lim);
  filter_.setLatAccLim(lat_acc_lim);
  filter_.setLatJerkLim(lat_jerk_lim);

  // Set default value
  current_gate_mode_.data = autoware_control_msgs::msg::GateMode::AUTO;

  // Service
  srv_engage_ = create_service<autoware_external_api_msgs::srv::Engage>(
    "~/service/engage",
    std::bind(&VehicleCmdGate::onEngageService, this, _1, _2));
  srv_external_emergency_ = create_service<autoware_external_api_msgs::srv::SetEmergency>(
    "~/service/external_emergency",
    std::bind(&VehicleCmdGate::onExternalEmergencyStopService, this, _1, _2, _3));
  srv_external_emergency_stop_ = this->create_service<std_srvs::srv::Trigger>(
    "~/service/external_emergency_stop",
    std::bind(&VehicleCmdGate::onSetExternalEmergencyStopService, this, _1, _2, _3));
  srv_clear_external_emergency_stop_ = this->create_service<std_srvs::srv::Trigger>(
    "~/service/clear_external_emergency_stop",
    std::bind(&VehicleCmdGate::onClearExternalEmergencyStopService, this, _1, _2, _3));

  // Diagnostics Updater
  updater_.setHardwareID("vehicle_cmd_gate");
  updater_.add(
    "heartbeat", [](auto & stat) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Alive");
    });
  updater_.add("emergency_stop_operation", this, &VehicleCmdGate::checkExternalEmergencyStop);

  // Start Request
  const auto use_start_request = declare_parameter("use_start_request", false);
  start_request_ = std::make_unique<StartRequest>(this, use_start_request);

  // Timer
  auto timer_callback = std::bind(&VehicleCmdGate::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(update_period_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

bool VehicleCmdGate::isHeartbeatTimeout(
  const std::shared_ptr<rclcpp::Time> & heartbeat_received_time, const double timeout)
{
  if (timeout == 0.0) {
    return false;
  }

  if (!heartbeat_received_time) {
    return true;
  }

  const auto time_from_heartbeat = this->now() - *heartbeat_received_time;

  return time_from_heartbeat.seconds() > timeout;
}

// for auto
void VehicleCmdGate::onAutoCtrlCmd(
  autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg)
{
  auto_commands_.control = *msg;

  if (current_gate_mode_.data == autoware_control_msgs::msg::GateMode::AUTO) {
    publishControlCommands(auto_commands_);
  }
}
void VehicleCmdGate::onAutoTurnSignalCmd(autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  auto_commands_.turn_signal = *msg;
}
void VehicleCmdGate::onAutoShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  auto_commands_.shift = *msg;
}

// for remote
void VehicleCmdGate::onRemoteCtrlCmd(
  autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg)
{
  remote_commands_.control = *msg;

  if (current_gate_mode_.data == autoware_control_msgs::msg::GateMode::EXTERNAL) {
    publishControlCommands(remote_commands_);
  }
}
void VehicleCmdGate::onRemoteTurnSignalCmd(
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  remote_commands_.turn_signal = *msg;
}
void VehicleCmdGate::onRemoteShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  remote_commands_.shift = *msg;
}

// for emergency
void VehicleCmdGate::onEmergencyCtrlCmd(
  autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg)
{
  emergency_commands_.control = *msg;

  if (use_emergency_handling_ && is_system_emergency_) {
    publishControlCommands(emergency_commands_);
  }
}
void VehicleCmdGate::onEmergencyTurnSignalCmd(
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  emergency_commands_.turn_signal = *msg;
}
void VehicleCmdGate::onEmergencyShiftCmd(
  autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  emergency_commands_.shift = *msg;
}

void VehicleCmdGate::onTimer()
{
  updater_.force_update();

  // Check system emergency heartbeat
  if (use_emergency_handling_) {
    is_system_emergency_heartbeat_timeout_ = isHeartbeatTimeout(
      system_emergency_heartbeat_received_time_, system_emergency_heartbeat_timeout_);

    if (is_system_emergency_heartbeat_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000 /*ms*/, "system_emergency heartbeat is timeout.");
      publishEmergencyStopControlCommands();
      return;
    }
  }

  // Check external emergency stop heartbeat
  if (use_external_emergency_stop_) {
    is_external_emergency_stop_heartbeat_timeout_ = isHeartbeatTimeout(
      external_emergency_stop_heartbeat_received_time_, external_emergency_stop_heartbeat_timeout_);

    if (is_external_emergency_stop_heartbeat_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000 /*ms*/, "external_emergency_stop heartbeat is timeout.");
      is_external_emergency_stop_ = true;
    }
  }

  // Check external emergency stop
  if (is_external_emergency_stop_) {
    if (!is_external_emergency_stop_heartbeat_timeout_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000 /*ms*/,
        "Please call `clear_external_emergency_stop` service to clear state.");
    }

    publishEmergencyStopControlCommands();
    return;
  }

  // Select commands
  autoware_vehicle_msgs::msg::TurnSignal turn_signal;
  autoware_vehicle_msgs::msg::ShiftStamped shift;
  if (use_emergency_handling_ && is_system_emergency_) {
    turn_signal = emergency_commands_.turn_signal;
    shift = emergency_commands_.shift;
  } else {
    if (current_gate_mode_.data == autoware_control_msgs::msg::GateMode::AUTO) {
      turn_signal = auto_commands_.turn_signal;
      shift = auto_commands_.shift;

      // Don't send turn signal when autoware is not engaged
      if (!is_engaged_) {
        turn_signal.data = autoware_vehicle_msgs::msg::TurnSignal::NONE;
      }
    } else if (current_gate_mode_.data == autoware_control_msgs::msg::GateMode::EXTERNAL) {
      turn_signal = remote_commands_.turn_signal;
      shift = remote_commands_.shift;
    } else {
      throw std::runtime_error("invalid mode");
    }
  }

  // Add frame_id to prevent RViz warnings
  fillFrameId(&shift.header.frame_id, "base_link");
  fillFrameId(&turn_signal.header.frame_id, "base_link");

  // Engage
  autoware_vehicle_msgs::msg::Engage autoware_engage;
  autoware_engage.stamp = this->now();
  autoware_engage.engage = is_engaged_;

  // Publish topics
  gate_mode_pub_->publish(current_gate_mode_);
  turn_signal_cmd_pub_->publish(turn_signal);
  shift_cmd_pub_->publish(shift);
  engage_pub_->publish(autoware_engage);

  // Publish start request
  start_request_->publishStartAccepted();
}

void VehicleCmdGate::publishControlCommands(const Commands & commands)
{
  // Check system emergency
  if (use_emergency_handling_ && is_system_emergency_heartbeat_timeout_) {
    return;
  }

  // Check external emergency stop
  if (is_external_emergency_stop_) {
    return;
  }

  Commands filtered_commands;

  // Set default commands
  {
    filtered_commands.control = commands.control;
    filtered_commands.shift = commands.shift;  // tmp
  }

  // Check emergency
  if (use_emergency_handling_ && is_system_emergency_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "Emergency!");
    filtered_commands.control = emergency_commands_.control;
    filtered_commands.shift = emergency_commands_.shift;  // tmp
  }

  // Check start after applying all gates except engage
  if (is_engaged_) {
    start_request_->checkStartRequest(filtered_commands.control);
  }

  // Check engage
  if (!is_engaged_ || !start_request_->isAccepted()) {
    filtered_commands.control.control = createStopControlCmd();
  }

  // Check stopped after applying all gates
  start_request_->checkStopped(filtered_commands.control);

  // Apply limit filtering
  filtered_commands.control.control = filterControlCommand(filtered_commands.control.control);

  // tmp: Create VehicleCmd
  autoware_vehicle_msgs::msg::VehicleCommand vehicle_cmd;
  vehicle_cmd.header = filtered_commands.control.header;
  vehicle_cmd.control = filtered_commands.control.control;
  vehicle_cmd.shift = filtered_commands.shift.shift;
  vehicle_cmd.emergency = (use_emergency_handling_ && is_system_emergency_);

  // Add frame_id to prevent RViz warnings
  fillFrameId(&vehicle_cmd.header.frame_id, "base_link");
  fillFrameId(&filtered_commands.control.header.frame_id, "base_link");

  // Publish commands
  vehicle_cmd_pub_->publish(vehicle_cmd);
  control_cmd_pub_->publish(filtered_commands.control);

  // Save ControlCmd to steering angle when disengaged
  prev_control_cmd_ = filtered_commands.control.control;
}

void VehicleCmdGate::publishEmergencyStopControlCommands()
{
  const auto stamp = this->now();

  // ControlCommand
  autoware_control_msgs::msg::ControlCommandStamped control_cmd;
  control_cmd.header.stamp = stamp;
  control_cmd.header.frame_id = "base_link";
  control_cmd.control = createEmergencyStopControlCmd();

  // Check stopped after applying all gates
  start_request_->checkStopped(control_cmd);

  // Shift
  autoware_vehicle_msgs::msg::ShiftStamped shift;
  shift.header.stamp = stamp;
  shift.header.frame_id = "base_link";
  shift.shift.data = autoware_vehicle_msgs::msg::Shift::NONE;

  // TurnSignal
  autoware_vehicle_msgs::msg::TurnSignal turn_signal;
  turn_signal.header.stamp = stamp;
  turn_signal.header.frame_id = "base_link";
  turn_signal.data = autoware_vehicle_msgs::msg::TurnSignal::HAZARD;

  // VehicleCommand
  autoware_vehicle_msgs::msg::VehicleCommand vehicle_cmd;
  vehicle_cmd.header.stamp = stamp;
  vehicle_cmd.header.frame_id = "base_link";
  vehicle_cmd.control = control_cmd.control;
  vehicle_cmd.shift = shift.shift;
  vehicle_cmd.emergency = true;

  // Engage
  autoware_vehicle_msgs::msg::Engage autoware_engage;
  autoware_engage.stamp = stamp;
  autoware_engage.engage = is_engaged_;

  // Publish topics
  vehicle_cmd_pub_->publish(vehicle_cmd);
  control_cmd_pub_->publish(control_cmd);
  gate_mode_pub_->publish(current_gate_mode_);
  turn_signal_cmd_pub_->publish(turn_signal);
  shift_cmd_pub_->publish(shift);
  engage_pub_->publish(autoware_engage);

  // Publish start request
  start_request_->publishStartAccepted();
}

autoware_control_msgs::msg::ControlCommand VehicleCmdGate::filterControlCommand(
  const autoware_control_msgs::msg::ControlCommand & in)
{
  autoware_control_msgs::msg::ControlCommand out = in;
  const double dt = getDt();

  filter_.limitLongitudinalWithVel(out);
  filter_.limitLongitudinalWithAcc(dt, out);
  filter_.limitLongitudinalWithJerk(dt, out);
  filter_.limitLateralWithLatAcc(dt, out);
  filter_.limitLateralWithLatJerk(dt, out);
  filter_.setPrevCmd(out);

  return out;
}

autoware_control_msgs::msg::ControlCommand VehicleCmdGate::createStopControlCmd() const
{
  autoware_control_msgs::msg::ControlCommand cmd;

  cmd.steering_angle = current_steer_;
  cmd.steering_angle_velocity = 0.0;
  cmd.velocity = 0.0;
  cmd.acceleration = -1.5;

  return cmd;
}

autoware_control_msgs::msg::ControlCommand VehicleCmdGate::createEmergencyStopControlCmd() const
{
  autoware_control_msgs::msg::ControlCommand cmd;

  cmd.steering_angle = prev_control_cmd_.steering_angle;
  cmd.steering_angle_velocity = prev_control_cmd_.steering_angle_velocity;
  cmd.velocity = 0.0;
  cmd.acceleration = -2.5;

  return cmd;
}

void VehicleCmdGate::onSystemEmergency(
  autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr msg)
{
  is_system_emergency_ = msg->is_emergency;
  system_emergency_heartbeat_received_time_ = std::make_shared<rclcpp::Time>(this->now());
}

void VehicleCmdGate::onExternalEmergencyStop(
  autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr msg)
{
  if (msg->is_emergency) {
    is_external_emergency_stop_ = true;
  }
  external_emergency_stop_heartbeat_received_time_ = std::make_shared<rclcpp::Time>(this->now());
}

void VehicleCmdGate::onGateMode(autoware_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  const auto prev_gate_mode = current_gate_mode_;
  current_gate_mode_ = *msg;

  if (current_gate_mode_.data != prev_gate_mode.data) {
    RCLCPP_INFO(
      get_logger(), "GateMode changed: %s -> %s", getGateModeName(prev_gate_mode.data),
      getGateModeName(current_gate_mode_.data));
  }
}

void VehicleCmdGate::onEngage(autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  is_engaged_ = msg->engage;
}

void VehicleCmdGate::onEngageService(
  const autoware_external_api_msgs::srv::Engage::Request::SharedPtr request,
  const autoware_external_api_msgs::srv::Engage::Response::SharedPtr response)
{
  is_engaged_ = request->engage;
  response->status = autoware_api_utils::response_success();
}

void VehicleCmdGate::onSteering(autoware_vehicle_msgs::msg::Steering::ConstSharedPtr msg)
{
  current_steer_ = msg->data;
}

double VehicleCmdGate::getDt()
{
  if (!prev_time_) {
    prev_time_ = std::make_shared<rclcpp::Time>(this->now());
    return 0.0;
  }

  const auto current_time = this->now();
  const auto dt = (current_time - *prev_time_).seconds();
  *prev_time_ = current_time;

  return dt;
}

void VehicleCmdGate::onExternalEmergencyStopService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<autoware_external_api_msgs::srv::SetEmergency::Request> request,
  const std::shared_ptr<autoware_external_api_msgs::srv::SetEmergency::Response> response)
{
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
  if (request->emergency) {
    onSetExternalEmergencyStopService(request_header, req, res);
  } else {
    onClearExternalEmergencyStopService(request_header, req, res);
  }

  if (res->success) {
    response->status = autoware_api_utils::response_success(res->message);
  } else {
    response->status = autoware_api_utils::response_error(res->message);
  }
}

bool VehicleCmdGate::onSetExternalEmergencyStopService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> req_header,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  is_external_emergency_stop_ = true;
  res->success = true;
  res->message = "external_emergency_stop requested was accepted.";

  return true;
}

bool VehicleCmdGate::onClearExternalEmergencyStopService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> req_header,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (is_external_emergency_stop_) {
    if (!is_external_emergency_stop_heartbeat_timeout_) {
      is_external_emergency_stop_ = false;
      res->success = true;
      res->message = "external_emergency_stop state was cleared.";
    } else {
      res->success = false;
      res->message = "Couldn't clear external_emergency_stop state because heartbeat is timeout.";
    }
  } else {
    res->success = false;
    res->message = "Not in external_emergency_stop state.";
  }

  return true;
}

void VehicleCmdGate::checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  DiagnosticStatus status;
  if (is_external_emergency_stop_heartbeat_timeout_) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "external_emergency_stop heartbeat is timeout.";
  } else if (is_external_emergency_stop_) {
    status.level = DiagnosticStatus::ERROR;
    status.message =
      "external_emergency_stop is required. Please call `clear_external_emergency_stop` service to "
      "clear state.";
  } else {
    status.level = DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}


VehicleCmdGate::StartRequest::StartRequest(rclcpp::Node * node, bool use_start_request)
{
  using namespace std::placeholders;

  node_ = node;
  use_start_request_ = use_start_request;
  is_start_requesting_ = false;
  is_start_accepted_ = false;
  is_start_cancelled_ = false;

  if (!use_start_request_) {
    return;
  }

  request_start_cli_ = node_->create_client<std_srvs::srv::Trigger>(
    "/api/autoware/set/start_request");
  request_start_pub_ = node_->create_publisher<autoware_debug_msgs::msg::BoolStamped>(
    "/api/autoware/get/start_accepted", rclcpp::QoS(1));
  current_twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/localization/twist", rclcpp::QoS(1),
    std::bind(&VehicleCmdGate::StartRequest::onCurrentTwist, this, _1));
}

void VehicleCmdGate::StartRequest::onCurrentTwist(
  geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  current_twist_ = *msg;
}

bool VehicleCmdGate::StartRequest::isAccepted()
{
  return !use_start_request_ || is_start_accepted_;
}

void VehicleCmdGate::StartRequest::publishStartAccepted()
{
  if (!use_start_request_) {
    return;
  }

  autoware_debug_msgs::msg::BoolStamped start_accepted;
  start_accepted.stamp = node_->now();
  start_accepted.data = is_start_accepted_;
  request_start_pub_->publish(start_accepted);
}

void VehicleCmdGate::StartRequest::checkStopped(const ControlCommandStamped & control)
{
  if (!use_start_request_) {
    return;
  }

  if (is_start_accepted_) {
    const auto control_velocity = std::abs(control.control.velocity);
    const auto current_velocity = std::abs(current_twist_.twist.linear.x);
    if (control_velocity < eps && current_velocity < eps) {
      is_start_accepted_ = false;
      is_start_cancelled_ = true;
      RCLCPP_INFO(node_->get_logger(), "clear start request");
    }
  }
}

void VehicleCmdGate::StartRequest::checkStartRequest(const ControlCommandStamped & control)
{
  if (!use_start_request_) {
    return;
  }

  if (!is_start_accepted_ && !is_start_requesting_) {
    const auto control_velocity = std::abs(control.control.velocity);
    if (eps < control_velocity) {
      is_start_requesting_ = true;
      is_start_cancelled_ = false;
      request_start_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
          const auto response = future.get();
          is_start_requesting_ = false;
          if (!is_start_cancelled_) {
            is_start_accepted_ = response->success;
            RCLCPP_INFO(node_->get_logger(), "start request is updated");
          } else {
            RCLCPP_INFO(node_->get_logger(), "start request is cancelled");
          }
        });
      RCLCPP_INFO(node_->get_logger(), "call start request");
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(VehicleCmdGate)
