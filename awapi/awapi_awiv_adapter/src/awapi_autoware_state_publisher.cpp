/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <awapi_awiv_adapter/awapi_autoware_state_publisher.h>

namespace autoware_api
{
AutowareIvAutowareStatePublisher::AutowareIvAutowareStatePublisher() : nh_(), pnh_("~")
{
  // publisher
  pub_state_ = pnh_.advertise<autoware_api_msgs::AwapiAutowareStatus>("output/autoware_status", 1);
}

void AutowareIvAutowareStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  autoware_api_msgs::AwapiAutowareStatus status;

  //input header
  status.header.frame_id = "base_link";
  status.header.stamp = ros::Time::now();

  // get all info
  getAutowareStateInfo(aw_info.autoware_state_ptr, &status);
  getControlModeInfo(aw_info.control_mode_ptr, &status);
  getGateModeInfo(aw_info.gate_mode_ptr, &status);
  getIsEmergencyInfo(aw_info.is_emergency_ptr, &status);
  getStopReasonInfo(aw_info.stop_reason_ptr, &status);
  getDiagInfo(aw_info.diagnostic_ptr, &status);
  getGlobalRptInfo(aw_info.global_rpt_ptr, &status);

  // publish info
  pub_state_.publish(status);
}

void AutowareIvAutowareStatePublisher::getAutowareStateInfo(
  const autoware_system_msgs::AutowareState::ConstPtr & autoware_state_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!autoware_state_ptr) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] autoware_state is nullptr");
    return;
  }

  // get autoware_state
  status->autoware_state = autoware_state_ptr->state;
}

void AutowareIvAutowareStatePublisher::getControlModeInfo(
  const autoware_vehicle_msgs::ControlMode::ConstPtr & control_mode_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!control_mode_ptr) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] control mode is nullptr");
    return;
  }

  // get control mode
  status->control_mode = control_mode_ptr->data;
}

void AutowareIvAutowareStatePublisher::getGateModeInfo(
  const autoware_control_msgs::GateMode::ConstPtr & gate_mode_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!gate_mode_ptr) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] gate mode is nullptr");
    return;
  }

  // get control mode
  status->gate_mode = gate_mode_ptr->data;
}

void AutowareIvAutowareStatePublisher::getIsEmergencyInfo(
  const std_msgs::Bool::ConstPtr & is_emergency_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!is_emergency_ptr) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] is_emergency is nullptr");
    return;
  }

  // get emergency
  status->emergency_stopped = is_emergency_ptr->data;
}

void AutowareIvAutowareStatePublisher::getStopReasonInfo(
  const autoware_planning_msgs::StopReasonArray::ConstPtr & stop_reason_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!stop_reason_ptr) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] stop reason is nullptr");
    return;
  }

  status->stop_reason = *stop_reason_ptr;
}

void AutowareIvAutowareStatePublisher::getDiagInfo(
  const diagnostic_msgs::DiagnosticArray::ConstPtr & diag_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!diag_ptr) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] diagnostics is nullptr");
    return;
  }

  // get diag
  status->diagnostics = diag_ptr->status;
}

void AutowareIvAutowareStatePublisher::getGlobalRptInfo(
  const pacmod_msgs::GlobalRpt::ConstPtr & global_rpt_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!global_rpt_ptr) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] global_rpt is nullptr");
    return;
  }

  // get global_rpt
  status->autonomous_overriden = global_rpt_ptr->override_active;
}

}  // namespace autoware_api
