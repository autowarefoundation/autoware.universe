// Copyright 2020 Tier IV, Inc.
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

#include "emergency_handler/emergency_handler_core.hpp"

#include <memory>
#include <string>
#include <utility>

EmergencyHandler::EmergencyHandler() : Node("emergency_handler")
{
  // Parameter
  param_.update_rate = declare_parameter<int>("update_rate", 10);
  param_.timeout_hazard_status = declare_parameter<double>("timeout_hazard_status", 0.5);
  param_.timeout_takeover_request = declare_parameter<double>("timeout_takeover_request", 10.0);
  param_.use_takeover_request = declare_parameter<bool>("use_takeover_request", false);
  param_.use_parking_after_stopped = declare_parameter<bool>("use_parking_after_stopped", false);
  param_.turning_hazard_on.emergency = declare_parameter<bool>("turning_hazard_on.emergency", true);

  using std::placeholders::_1;

  // Subscriber
  sub_hazard_status_stamped_ =
    create_subscription<autoware_auto_system_msgs::msg::HazardStatusStamped>(
      "~/input/hazard_status", rclcpp::QoS{1},
      std::bind(&EmergencyHandler::onHazardStatusStamped, this, _1));
  sub_prev_control_command_ =
    create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "~/input/prev_control_command", rclcpp::QoS{1},
      std::bind(&EmergencyHandler::onPrevControlCommand, this, _1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&EmergencyHandler::onOdometry, this, _1));
  // subscribe control mode
  sub_control_mode_ = create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "~/input/control_mode", rclcpp::QoS{1}, std::bind(&EmergencyHandler::onControlMode, this, _1));
  sub_mrm_comfortable_stop_status_ = create_subscription<autoware_adapi_v1_msgs::msg::MRMBehaviorStatus>(
    "~/input/mrm/comfortable_stop/status", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onMRMComfortableStopStatus, this, _1));
  sub_mrm_emergency_stop_status_ = create_subscription<autoware_adapi_v1_msgs::msg::MRMBehaviorStatus>(
    "~/input/mrm/emergency_stop/status", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onMRMEmergencyStopStatus, this, _1));

  // Heartbeat
  heartbeat_hazard_status_ = std::make_shared<
    HeaderlessHeartbeatChecker<autoware_auto_system_msgs::msg::HazardStatusStamped>>(
    *this, "~/input/hazard_status", param_.timeout_hazard_status);

  // Publisher
  pub_control_command_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "~/output/control_command", rclcpp::QoS{1});
  pub_hazard_cmd_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
    "~/output/hazard", rclcpp::QoS{1});
  pub_gear_cmd_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("~/output/gear", rclcpp::QoS{1});
  pub_emergency_state_ = create_publisher<autoware_auto_system_msgs::msg::EmergencyState>(
    "~/output/emergency_state", rclcpp::QoS{1});
  pub_mrm_state_ = create_publisher<autoware_adapi_v1_msgs::msg::MRMState>(
    "~/output/mrm/state", rclcpp::QoS{1});

  // Clients
  client_mrm_comfortable_stop_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_mrm_comfortable_stop_ =
    create_client<autoware_adapi_v1_msgs::srv::OperateMRM>(
      "~/output/mrm/comfortable_stop/operate", rmw_qos_profile_services_default, client_mrm_comfortable_stop_group_);
  client_mrm_emergency_stop_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_mrm_emergency_stop_ =
    create_client<autoware_adapi_v1_msgs::srv::OperateMRM>(
      "~/output/mrm/emergency_stop/operate", rmw_qos_profile_services_default, client_mrm_emergency_stop_group_);

  // Initialize
  odom_ = std::make_shared<const nav_msgs::msg::Odometry>();
  control_mode_ = std::make_shared<const autoware_auto_vehicle_msgs::msg::ControlModeReport>();
  prev_control_command_ = autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr(
    new autoware_auto_control_msgs::msg::AckermannControlCommand);
  mrm_comfortable_stop_status_ = std::make_shared<const autoware_adapi_v1_msgs::msg::MRMBehaviorStatus>();
  mrm_emergency_stop_status_ = std::make_shared<const autoware_adapi_v1_msgs::msg::MRMBehaviorStatus>();
  mrm_state_.stamp = this->now();
  mrm_state_.state = autoware_adapi_v1_msgs::msg::MRMState::NORMAL;
  mrm_state_.behavior = autoware_adapi_v1_msgs::msg::MRMState::NONE;

  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&EmergencyHandler::onTimer, this));
}

void EmergencyHandler::onHazardStatusStamped(
  const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  hazard_status_stamped_ = msg;
}

void EmergencyHandler::onPrevControlCommand(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  auto control_command = new autoware_auto_control_msgs::msg::AckermannControlCommand(*msg);
  control_command->stamp = msg->stamp;
  prev_control_command_ =
    autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr(control_command);
}

void EmergencyHandler::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void EmergencyHandler::onControlMode(
  const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg)
{
  control_mode_ = msg;
}

void EmergencyHandler::onMRMComfortableStopStatus(
  const autoware_adapi_v1_msgs::msg::MRMBehaviorStatus::ConstSharedPtr msg)
{
  mrm_comfortable_stop_status_ = msg;
}

void EmergencyHandler::onMRMEmergencyStopStatus(
  const autoware_adapi_v1_msgs::msg::MRMBehaviorStatus::ConstSharedPtr msg)
{
  mrm_emergency_stop_status_ = msg;
}

autoware_auto_vehicle_msgs::msg::HazardLightsCommand EmergencyHandler::createHazardCmdMsg()
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
  HazardLightsCommand msg;

  // Check emergency
  const bool is_emergency = isEmergency(hazard_status_stamped_->status);

  if (hazard_status_stamped_->status.emergency_holding) {
    // turn hazard on during emergency holding
    msg.command = HazardLightsCommand::ENABLE;
  } else if (is_emergency && param_.turning_hazard_on.emergency) {
    // turn hazard on if vehicle is in emergency state and
    // turning hazard on if emergency flag is true
    msg.command = HazardLightsCommand::ENABLE;

  } else {
    msg.command = HazardLightsCommand::NO_COMMAND;
  }
  return msg;
}

void EmergencyHandler::publishControlCommands()
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;

  // Create timestamp
  const auto stamp = this->now();

  // Publish ControlCommand
  /* {
    autoware_auto_control_msgs::msg::AckermannControlCommand msg;
    msg = selectAlternativeControlCommand();
    msg.stamp = stamp;
    msg.lateral.stamp = stamp;
    msg.longitudinal.stamp = stamp;
    pub_control_command_->publish(msg);
  } */

  // Publish hazard command
  pub_hazard_cmd_->publish(createHazardCmdMsg());

  // Publish gear
  {
    GearCommand msg;
    msg.stamp = stamp;
    if (param_.use_parking_after_stopped && isStopped()) {
      msg.command = GearCommand::PARK;
    } else {
      msg.command = GearCommand::DRIVE;
    }
    pub_gear_cmd_->publish(msg);
  }

  // Publish Emergency State
  {
    autoware_auto_system_msgs::msg::EmergencyState emergency_state;
    emergency_state.stamp = stamp;
    emergency_state.state = emergency_state_;
    pub_emergency_state_->publish(emergency_state);
  }
}

void EmergencyHandler::publishMRMState()
{
  mrm_state_.stamp = this->now();
  pub_mrm_state_->publish(mrm_state_);
}

void EmergencyHandler::operateMRM()
{
  using autoware_auto_system_msgs::msg::EmergencyState;
  using autoware_adapi_v1_msgs::msg::MRMState;

  if (emergency_state_ == EmergencyState::NORMAL) {
    // Cancel MRM behavior when returning to NORMAL state
    const auto current_mrm_behavior = MRMState::NONE;
    if (current_mrm_behavior != mrm_behavior_) {
      cancelMRMBehavior(mrm_behavior_);
      mrm_behavior_ = current_mrm_behavior;
      mrm_state_.behavior = current_mrm_behavior;
    }
    return;
  }
  if (emergency_state_ == EmergencyState::OVERRIDE_REQUESTING) {
    // Do nothing
    return;
  }
  if (emergency_state_ == EmergencyState::MRM_OPERATING) {
    const auto current_mrm_behavior = updateMRMBehavior();
    if (current_mrm_behavior != mrm_behavior_) {
      cancelMRMBehavior(mrm_behavior_);
      callMRMBehavior(current_mrm_behavior);
      mrm_behavior_ = current_mrm_behavior;
      mrm_state_.behavior = current_mrm_behavior;
    }
    return;
  }
  if (emergency_state_ == EmergencyState::MRM_SUCCEEDED) {
    // TODO(mkuri): operate MRC behavior
    // Do nothing
    return;
  }
  if (emergency_state_ == EmergencyState::MRM_FAILED) {
    // Do nothing
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM state: %d", emergency_state_);
}

void EmergencyHandler::callMRMBehavior(
    const autoware_adapi_v1_msgs::msg::MRMState::_behavior_type & mrm_behavior) const
{
  using autoware_adapi_v1_msgs::msg::MRMState;

  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::OperateMRM::Request>();
  request->operate = true;

  if (mrm_behavior == MRMState::COMFORTABLE_STOP) {
    auto result = client_mrm_comfortable_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Comfortable stop is operated");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Comfortable stop is failed to operate");
    }
    return;
  }
  if (mrm_behavior == MRMState::EMERGENCY_STOP) {
    auto result = client_mrm_emergency_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop is operated");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Emergency stop is failed to operate");
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
}

void EmergencyHandler::cancelMRMBehavior(
  const autoware_adapi_v1_msgs::msg::MRMState::_behavior_type & mrm_behavior) const
{
  using autoware_adapi_v1_msgs::msg::MRMState;

  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::OperateMRM::Request>();
  request->operate = false;

  if (mrm_behavior == MRMState::COMFORTABLE_STOP) {
    auto result = client_mrm_comfortable_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Comfortable stop is canceled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Comfortable stop is failed to cancel");
    }
    return;
  }
  if (mrm_behavior == MRMState::EMERGENCY_STOP) {
    auto result = client_mrm_emergency_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop is canceled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Emergency stop is failed to cancel");
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
}

bool EmergencyHandler::isDataReady()
{
  if (!hazard_status_stamped_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for hazard_status_stamped msg...");
    return false;
  }

  return true;
}

void EmergencyHandler::onTimer()
{
  if (!isDataReady()) {
    return;
  }
  if (heartbeat_hazard_status_->isTimeout()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "heartbeat_hazard_status is timeout");
    emergency_state_ = autoware_auto_system_msgs::msg::EmergencyState::MRM_OPERATING;
    publishControlCommands();
    return;
  }

  // Update Emergency State
  updateEmergencyState();

  // Publish control commands
  publishControlCommands();
  operateMRM();
  publishMRMState();
}

void EmergencyHandler::transitionTo(const int new_state)
{
  using autoware_auto_system_msgs::msg::EmergencyState;

  const auto state2string = [](const int state) {
    if (state == EmergencyState::NORMAL) {
      return "NORMAL";
    }
    if (state == EmergencyState::OVERRIDE_REQUESTING) {
      return "OVERRIDE_REQUESTING";
    }
    if (state == EmergencyState::MRM_OPERATING) {
      return "MRM_OPERATING";
    }
    if (state == EmergencyState::MRM_SUCCEEDED) {
      return "MRM_SUCCEEDED";
    }
    if (state == EmergencyState::MRM_FAILED) {
      return "MRM_FAILED";
    }

    const auto msg = "invalid state: " + std::to_string(state);
    throw std::runtime_error(msg);
  };

  RCLCPP_INFO(
    this->get_logger(), "EmergencyState changed: %s -> %s", state2string(emergency_state_),
    state2string(new_state));

  emergency_state_ = new_state;
  // TODO(Makoto Kurihara): Use MRM state
  mrm_state_.state = new_state;
}

void EmergencyHandler::updateEmergencyState()
{
  using autoware_auto_system_msgs::msg::EmergencyState;
  using autoware_auto_vehicle_msgs::msg::ControlModeReport;

  // Check emergency
  const bool is_emergency = isEmergency(hazard_status_stamped_->status);

  // Get mode
  const bool is_auto_mode = control_mode_->mode == ControlModeReport::AUTONOMOUS;
  const bool is_takeover_done = control_mode_->mode == ControlModeReport::MANUAL;

  // State Machine
  if (emergency_state_ == EmergencyState::NORMAL) {
    // NORMAL
    if (is_auto_mode && is_emergency) {
      if (param_.use_takeover_request) {
        takeover_requested_time_ = this->get_clock()->now();
        transitionTo(EmergencyState::OVERRIDE_REQUESTING);
        return;
      } else {
        transitionTo(EmergencyState::MRM_OPERATING);
        return;
      }
    }
  } else {
    // Emergency
    // Send recovery events if "not emergency" or "takeover done"
    if (!is_emergency) {
      transitionTo(EmergencyState::NORMAL);
      return;
    }
    // TODO(Kenji Miyake): Check if human can safely override, for example using DSM
    if (is_takeover_done) {
      transitionTo(EmergencyState::NORMAL);
      return;
    }

    if (emergency_state_ == EmergencyState::OVERRIDE_REQUESTING) {
      const auto time_from_takeover_request = this->get_clock()->now() - takeover_requested_time_;
      if (time_from_takeover_request.seconds() > param_.timeout_takeover_request) {
        transitionTo(EmergencyState::MRM_OPERATING);
        return;
      }
    } else if (emergency_state_ == EmergencyState::MRM_OPERATING) {
      // TODO(Kenji Miyake): Check MRC is accomplished
      if (isStopped()) {
        transitionTo(EmergencyState::MRM_SUCCEEDED);
        return;
      }
    } else if (emergency_state_ == EmergencyState::MRM_SUCCEEDED) {
      // Do nothing(only checking common recovery events)
    } else if (emergency_state_ == EmergencyState::MRM_FAILED) {
      // Do nothing(only checking common recovery events)
    } else {
      const auto msg = "invalid state: " + std::to_string(emergency_state_);
      throw std::runtime_error(msg);
    }
  }
}

autoware_adapi_v1_msgs::msg::MRMState::_behavior_type EmergencyHandler::updateMRMBehavior()
{
  using autoware_auto_system_msgs::msg::HazardStatus;
  using autoware_adapi_v1_msgs::msg::MRMState;

  // Get hazard level
  const auto level = hazard_status_stamped_->status.level;

  // State machine
  if (mrm_behavior_ == MRMState::NONE) {
    if (level == HazardStatus::LATENT_FAULT) {
      return MRMState::COMFORTABLE_STOP;
    }
    if (level == HazardStatus::SINGLE_POINT_FAULT) {
      return MRMState::EMERGENCY_STOP;
    }
  }
  if (mrm_behavior_ == MRMState::COMFORTABLE_STOP) {
    if (level == HazardStatus::SINGLE_POINT_FAULT) {
      return MRMState::EMERGENCY_STOP;
    }
  }

  return mrm_behavior_;
}

bool EmergencyHandler::isEmergency(
  const autoware_auto_system_msgs::msg::HazardStatus & hazard_status)
{
  return hazard_status.emergency || hazard_status.emergency_holding;
}

bool EmergencyHandler::isStopped()
{
  constexpr auto th_stopped_velocity = 0.001;
  if (odom_->twist.twist.linear.x < th_stopped_velocity) {
    return true;
  }

  return false;
}

autoware_auto_control_msgs::msg::AckermannControlCommand
EmergencyHandler::selectAlternativeControlCommand()
{
  // TODO(jilaada): Add safe_stop planner

  // Emergency Stop
  {
    autoware_auto_control_msgs::msg::AckermannControlCommand emergency_stop_cmd;
    emergency_stop_cmd.lateral = prev_control_command_->lateral;
    emergency_stop_cmd.longitudinal.speed = 0.0;
    emergency_stop_cmd.longitudinal.acceleration = -2.5;

    return emergency_stop_cmd;
  }
}
