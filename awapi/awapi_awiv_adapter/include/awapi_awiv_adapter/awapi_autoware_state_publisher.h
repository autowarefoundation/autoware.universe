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

#include <autoware_api_msgs/AwapiAutowareStatus.h>
#include <awapi_awiv_adapter/awapi_autoware_util.h>

namespace autoware_api
{
class AutowareIvAutowareStatePublisher
{
public:
  AutowareIvAutowareStatePublisher();
  void statePublisher(const AutowareInfo & aw_info);

private:
  // node handle
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // publisher
  ros::Publisher pub_state_;

  // parameter

  /* parameter for judging goal now */
  bool arrived_goal_;
  autoware_system_msgs::AutowareState::_state_type prev_state_;

  void getAutowareStateInfo(
    const autoware_system_msgs::AutowareState::ConstPtr & autoware_state_ptr,
    autoware_api_msgs::AwapiAutowareStatus * status);
  void getControlModeInfo(
    const autoware_vehicle_msgs::ControlMode::ConstPtr & control_mode_ptr,
    autoware_api_msgs::AwapiAutowareStatus * status);
  void getGateModeInfo(
    const autoware_control_msgs::GateMode::ConstPtr & gate_mode_ptr,
    autoware_api_msgs::AwapiAutowareStatus * status);
  void getIsEmergencyInfo(
    const std_msgs::Bool::ConstPtr & is_emergency_ptr,
    autoware_api_msgs::AwapiAutowareStatus * status);
  void getStopReasonInfo(
    const autoware_planning_msgs::StopReasonArray::ConstPtr & stop_reason_ptr,
    autoware_api_msgs::AwapiAutowareStatus * status);
  void getDiagInfo(
    const diagnostic_msgs::DiagnosticArray::ConstPtr & diag_ptr,
    autoware_api_msgs::AwapiAutowareStatus * status);

  void getGlobalRptInfo(
    const pacmod_msgs::GlobalRpt::ConstPtr & global_rpt_ptr,
    autoware_api_msgs::AwapiAutowareStatus * status);

  bool isGoal(const autoware_system_msgs::AutowareState::ConstPtr & autoware_state);
};

}  // namespace autoware_api
