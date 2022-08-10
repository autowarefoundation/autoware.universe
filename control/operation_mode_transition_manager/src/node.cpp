// Copyright 2022 Autoware Foundation
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

#include "node.hpp"

#include <component_interface_utils/rclcpp/exceptions.hpp>

namespace operation_mode_transition_manager
{

OperationModeTransitionManager::OperationModeTransitionManager(const rclcpp::NodeOptions & options)
: Node("operation_mode_transition_manager", options)
{
  sub_control_mode_report_ = create_subscription<ControlModeReport>(
    "control_mode_report", 1,
    [this](const ControlModeReport::SharedPtr msg) { control_mode_report_ = *msg; });

  sub_gate_operation_mode_ = create_subscription<OperationModeState>(
    "gate_operation_mode", 1,
    [this](const OperationModeState::SharedPtr msg) { gate_operation_mode_ = *msg; });

  cli_control_mode_ = create_client<ControlModeRequest>("control_mode_request");
  pub_debug_info_ = create_publisher<ModeChangeBase::DebugInfo>("~/debug_info", 1);

  // component interface
  {
    const auto node = component_interface_utils::NodeAdaptor(this);
    node.init_srv(
      srv_autoware_control, this, &OperationModeTransitionManager::onChangeAutowareControl);
    node.init_srv(srv_operation_mode, this, &OperationModeTransitionManager::onChangeOperationMode);
    node.init_pub(pub_operation_mode_);
  }

  // timer
  {
    const auto period_ns = rclcpp::Rate(declare_parameter<double>("frequency_hz")).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&OperationModeTransitionManager::onTimer, this));
  }

  // initialize state
  transition_timeout_ = declare_parameter<double>("transition_timeout");
  current_mode_ = OperationMode::STOP;
  transition_ = nullptr;
  gate_operation_mode_.operation.mode = OperationModeValue::UNKNOWN;
  gate_operation_mode_.is_in_transition = false;
  control_mode_report_.mode = ControlModeReport::NO_COMMAND;

  // modes
  modes_[OperationMode::STOP] = std::make_unique<StopMode>();
  modes_[OperationMode::AUTONOMOUS] = std::make_unique<AutonomousMode>(this);
  modes_[OperationMode::LOCAL] = std::make_unique<LocalMode>();
  modes_[OperationMode::REMOTE] = std::make_unique<RemoteMode>();
}

void OperationModeTransitionManager::onChangeAutowareControl(
  const ChangeAutowareControlAPI::Service::Request::SharedPtr request,
  const ChangeAutowareControlAPI::Service::Response::SharedPtr response)
{
  if (request->autoware_control) {
    // Treat as a transition to the current operation mode.
    changeOperationMode(std::nullopt);
  } else {
    // Allow mode transition to complete without canceling.
    transition_.reset();
    changeControlMode(ControlModeRequestType::MANUAL);
  }
  response->status.success = true;
}

void OperationModeTransitionManager::onChangeOperationMode(
  const ChangeOperationModeAPI::Service::Request::SharedPtr request,
  const ChangeOperationModeAPI::Service::Response::SharedPtr response)
{
  const auto mode = toEnum(*request);
  if (!mode) {
    throw component_interface_utils::ParameterError("The operation mode is invalid.");
  }
  changeOperationMode(mode.value());
  response->status.success = true;
}

void OperationModeTransitionManager::changeControlMode(ControlModeRequestType::_data_type mode)
{
  const auto callback = [this](rclcpp::Client<ControlModeRequest>::SharedFuture future) {
    if (!future.get()->success) {
      RCLCPP_WARN(get_logger(), "Autonomous mode change was rejected.");
      if (transition_) {
        transition_->is_engage_failed = true;
      }
    }
  };

  const auto request = std::make_shared<ControlModeRequest::Request>();
  request->mode.header.stamp = now();
  request->mode.data = mode;
  cli_control_mode_->async_send_request(request, callback);
}

void OperationModeTransitionManager::changeOperationMode(std::optional<OperationMode> request_mode)
{
  const bool current_control = control_mode_report_.mode == ControlModeReport::AUTONOMOUS;
  const bool request_control = request_mode ? false : true;

  if (current_mode_ == request_mode) {
    throw component_interface_utils::NoEffectWarning("The mode is the same as the current.");
  }

  if (current_control && request_control) {
    throw component_interface_utils::NoEffectWarning("Autoware already controls the vehicle.");
  }

  // TODO(Takagi, Isamu): overwrite transition between operation modes
  if (transition_) {
    throw component_interface_utils::ServiceException(
      ServiceResponse::ERROR_IN_TRANSITION, "The mode transition is in progress.");
  }

  if (!modes_.at(request_mode.value_or(current_mode_))->isModeChangeAvailable()) {
    throw component_interface_utils::ServiceException(
      ServiceResponse::ERROR_NOT_AVAILABLE, "The mode change condition is not satisfied.");
  }

  // Enter transition mode if the vehicle is being or will be controlled by Autoware.
  if (current_control || request_control) {
    if (request_control) {
      transition_ = std::make_unique<Transition>(now(), request_control, std::nullopt);
    } else {
      transition_ = std::make_unique<Transition>(now(), request_control, current_mode_);
    }
  }
  current_mode_ = request_mode.value_or(current_mode_);
}

void OperationModeTransitionManager::cancelTransition()
{
  const auto & previous = transition_->previous;
  if (previous) {
    current_mode_ = previous.value();
  } else {
    changeControlMode(ControlModeRequestType::MANUAL);
  }
  transition_.reset();
}

void OperationModeTransitionManager::processTransition()
{
  const bool current_control = control_mode_report_.mode == ControlModeReport::AUTONOMOUS;

  // Check timeout.
  if (transition_timeout_ < (now() - transition_->time).seconds()) {
    return cancelTransition();
  }

  // Check engage failure.
  if (transition_->is_engage_failed) {
    return cancelTransition();
  }

  // Check override.
  if (current_control) {
    transition_->is_engage_completed = true;
  } else {
    if (transition_->is_engage_completed) {
      return cancelTransition();
    }
  }

  // Check completion when engaged, otherwise engage after the gate reflects transition.
  if (current_control) {
    if (modes_.at(current_mode_)->isModeChangeCompleted()) {
      return transition_.reset();
    }
  } else {
    if (transition_->is_engage_requested && gate_operation_mode_.is_in_transition) {
      transition_->is_engage_requested = false;
      return changeControlMode(ControlModeRequestType::AUTO);
    }
  }
}

void OperationModeTransitionManager::onTimer()
{
  for (const auto & [type, mode] : modes_) {
    mode->update(current_mode_ == type && transition_);
  }

  if (transition_) {
    processTransition();
  }

  publishData();
}

void OperationModeTransitionManager::publishData()
{
  const bool current_control = control_mode_report_.mode == ControlModeReport::AUTONOMOUS;
  const auto time = now();

  OperationModeStateAPI::Message state;
  state.operation = toMsg(current_mode_);
  state.is_autoware_control_enabled = current_control;
  state.is_in_transition = transition_ ? true : false;
  state.change_to_stop = modes_.at(OperationMode::STOP)->isModeChangeAvailable();
  state.change_to_autonomous = modes_.at(OperationMode::AUTONOMOUS)->isModeChangeAvailable();
  state.change_to_local = modes_.at(OperationMode::LOCAL)->isModeChangeAvailable();
  state.change_to_remote = modes_.at(OperationMode::REMOTE)->isModeChangeAvailable();

  if (prev_state_ != state) {
    prev_state_ = state;
    state.stamp = time;
    pub_operation_mode_->publish(state);
  }

  ModeChangeBase::DebugInfo debug_info = modes_.at(OperationMode::AUTONOMOUS)->getDebugInfo();
  debug_info.stamp = time;
  debug_info.current_state = toString(current_mode_);
  debug_info.previous_state = transition_ ? toString(transition_->previous) : "NONE";
  pub_debug_info_->publish(debug_info);
}

}  // namespace operation_mode_transition_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(operation_mode_transition_manager::OperationModeTransitionManager)
