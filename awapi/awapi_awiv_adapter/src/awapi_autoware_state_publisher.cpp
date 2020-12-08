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

#include "awapi_awiv_adapter/awapi_autoware_state_publisher.hpp"
#include <regex>

namespace autoware_api
{
AutowareIvAutowareStatePublisher::AutowareIvAutowareStatePublisher(rclcpp::Node & node)
: logger_(node.get_logger().get_child("awapi_awiv_autoware_state_publisher")),
  clock_(node.get_clock()),
  arrived_goal_(false)
{
  // publisher
  pub_state_ = node.create_publisher<autoware_api_msgs::msg::AwapiAutowareStatus>(
    "output/autoware_status", 1);
}

void AutowareIvAutowareStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  autoware_api_msgs::msg::AwapiAutowareStatus status;

  //input header
  status.header.frame_id = "base_link";
  status.header.stamp = clock_->now();

  // get all info
  getAutowareStateInfo(aw_info.autoware_state_ptr, &status);
  getControlModeInfo(aw_info.control_mode_ptr, &status);
  getGateModeInfo(aw_info.gate_mode_ptr, &status);
  getIsEmergencyInfo(aw_info.is_emergency_ptr, &status);
  getStopReasonInfo(aw_info.stop_reason_ptr, &status);
  getDiagInfo(aw_info.diagnostic_ptr, &status);
  getGlobalRptInfo(aw_info.global_rpt_ptr, &status);

  // publish info
  pub_state_->publish(status);
}

void AutowareIvAutowareStatePublisher::getAutowareStateInfo(
  const autoware_system_msgs::msg::AutowareState::ConstSharedPtr & autoware_state_ptr,
  autoware_api_msgs::msg::AwapiAutowareStatus * status)
{
  if (!autoware_state_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      logger_, *clock_, 5000 /* ms */, "autoware_state is nullptr");
    return;
  }

  // get autoware_state
  status->autoware_state = autoware_state_ptr->state;
  status->arrived_goal = isGoal(autoware_state_ptr);
}

void AutowareIvAutowareStatePublisher::getControlModeInfo(
  const autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr & control_mode_ptr,
  autoware_api_msgs::msg::AwapiAutowareStatus * status)
{
  if (!control_mode_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "control mode is nullptr");
    return;
  }

  // get control mode
  status->control_mode = control_mode_ptr->data;
}

void AutowareIvAutowareStatePublisher::getGateModeInfo(
  const autoware_control_msgs::msg::GateMode::ConstSharedPtr & gate_mode_ptr,
  autoware_api_msgs::msg::AwapiAutowareStatus * status)
{
  if (!gate_mode_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "gate mode is nullptr");
    return;
  }

  // get control mode
  status->gate_mode = gate_mode_ptr->data;
}

void AutowareIvAutowareStatePublisher::getIsEmergencyInfo(
  const autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr & is_emergency_ptr,
  autoware_api_msgs::msg::AwapiAutowareStatus * status)
{
  if (!is_emergency_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "is_emergency is nullptr");
    return;
  }

  // get emergency
  status->emergency_stopped = is_emergency_ptr->is_emergency;
}

void AutowareIvAutowareStatePublisher::getStopReasonInfo(
  const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & stop_reason_ptr,
  autoware_api_msgs::msg::AwapiAutowareStatus * status)
{
  if (!stop_reason_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "stop reason is nullptr");
    return;
  }

  status->stop_reason = *stop_reason_ptr;
}

void AutowareIvAutowareStatePublisher::getDiagInfo(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & diag_ptr,
  autoware_api_msgs::msg::AwapiAutowareStatus * status)
{
  if (!diag_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "diagnostics is nullptr");
    return;
  }

  // get diag
  status->diagnostics = extractLeafDiag(diag_ptr->status);
}

void AutowareIvAutowareStatePublisher::getGlobalRptInfo(
  const pacmod_msgs::msg::GlobalRpt::ConstSharedPtr & global_rpt_ptr,
  autoware_api_msgs::msg::AwapiAutowareStatus * status)
{
  if (!global_rpt_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "global_rpt is nullptr");
    return;
  }

  // get global_rpt
  status->autonomous_overriden = global_rpt_ptr->override_active;
}

bool AutowareIvAutowareStatePublisher::isGoal(
  const autoware_system_msgs::msg::AutowareState::ConstSharedPtr & autoware_state)
{
  //rename
  const auto & aw_state = autoware_state->state;

  if (aw_state == autoware_system_msgs::msg::AutowareState::ARRIVAL_GOAL) {
    arrived_goal_ = true;
  } else if (
    prev_state_ == autoware_system_msgs::msg::AutowareState::DRIVING &&
    aw_state == autoware_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE)
  {
    arrived_goal_ = true;
  }

  if (
    aw_state == autoware_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE ||
    aw_state == autoware_system_msgs::msg::AutowareState::DRIVING)
  {
    //cancel goal state
    arrived_goal_ = false;
  }

  prev_state_ = aw_state;

  return arrived_goal_;
}

std::vector<diagnostic_msgs::msg::DiagnosticStatus>
AutowareIvAutowareStatePublisher::extractLeafDiag(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diag_vec)
{
  updateDiagNameSet(diag_vec);

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> leaf_diag_info;
  for (const auto diag : diag_vec) {
    if (isLeaf(diag)) {
      leaf_diag_info.emplace_back(diag);
    }
  }
  return leaf_diag_info;
}

std::string AutowareIvAutowareStatePublisher::splitStringByLastSlash(const std::string & str)
{
  const auto last_slash = str.find_last_of("/");

  if (last_slash == std::string::npos) {
    // if not find slash
    return str;
  }

  return str.substr(0, last_slash);
}

void AutowareIvAutowareStatePublisher::updateDiagNameSet(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diag_vec)
{
  // set diag name to diag_name_set_
  for (const auto & diag : diag_vec) {
    diag_name_set_.insert(splitStringByLastSlash(diag.name));
  }
}

bool AutowareIvAutowareStatePublisher::isLeaf(const diagnostic_msgs::msg::DiagnosticStatus & diag)
{
  //if not find diag.name in diag set, diag is leaf.
  return diag_name_set_.find(diag.name) == diag_name_set_.end();
}

}  // namespace autoware_api
