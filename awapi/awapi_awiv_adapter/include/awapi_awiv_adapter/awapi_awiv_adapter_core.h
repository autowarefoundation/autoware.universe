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
#include <awapi_awiv_adapter/awapi_autoware_util.h>
#include <awapi_awiv_adapter/awapi_lane_change_state_publisher.h>
#include <awapi_awiv_adapter/awapi_obstacle_avoidance_state_publisher.h>
#include <awapi_awiv_adapter/awapi_stop_reason_aggregator.h>
#include <awapi_awiv_adapter/awapi_vehicle_state_publisher.h>

namespace autoware_api
{
class AutowareIvAdapter
{
public:
  AutowareIvAdapter();

private:
  // node handle
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // subscriber
  ros::Subscriber sub_steer_;
  ros::Subscriber sub_vehicle_cmd_;
  ros::Subscriber sub_turn_signal_cmd_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_gear_;
  ros::Subscriber sub_battery_;
  ros::Subscriber sub_nav_sat_;
  ros::Subscriber sub_autoware_state_;
  ros::Subscriber sub_control_mode_;
  ros::Subscriber sub_gate_mode_;
  ros::Subscriber sub_emergency_;
  ros::Subscriber sub_stop_reason_;
  ros::Subscriber sub_diagnostics_;
  ros::Subscriber sub_global_rpt_;
  ros::Subscriber sub_lane_change_available_;
  ros::Subscriber sub_lane_change_ready_;
  ros::Subscriber sub_lane_change_candidate_;
  ros::Subscriber sub_obstacle_avoid_ready_;
  ros::Subscriber sub_obstacle_avoid_candidate_;
  // timer
  ros::Timer timer_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // callback function
  void callbackSteer(const autoware_vehicle_msgs::Steering::ConstPtr & msg_ptr);
  void callbackVehicleCmd(const autoware_vehicle_msgs::VehicleCommand::ConstPtr & msg_ptr);
  void callbackTurnSignal(const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg_ptr);
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr & msg_ptr);
  void callbackGear(const autoware_vehicle_msgs::ShiftStamped::ConstPtr & msg_ptr);
  void callbackBattery(const std_msgs::Float32::ConstPtr & msg_ptr);
  void callbackNavSat(const sensor_msgs::NavSatFix::ConstPtr & msg_ptr);
  void callbackAutowareState(const autoware_system_msgs::AutowareState::ConstPtr & msg_ptr);
  void callbackControlMode(const autoware_vehicle_msgs::ControlMode::ConstPtr & msg_ptr);
  void callbackGateMode(const autoware_control_msgs::GateMode::ConstPtr & msg_ptr);
  void callbackIsEmergency(const std_msgs::Bool::ConstPtr & msg_ptr);
  void callbackStopReason(const autoware_planning_msgs::StopReasonArray::ConstPtr & msg_ptr);
  void callbackDiagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr & msg_ptr);
  void callbackGlobalRpt(const pacmod_msgs::GlobalRpt::ConstPtr & msg_ptr);
  void callbackLaneChangeAvailable(const std_msgs::Bool::ConstPtr & msg_ptr);
  void callbackLaneChangeReady(const std_msgs::Bool::ConstPtr & msg_ptr);
  void callbackLaneChangeCandidatePath(const autoware_planning_msgs::Path::ConstPtr & msg_ptr);
  void callbackLaneObstacleAvoidReady(const std_msgs::Bool::ConstPtr & msg_ptr);
  void callbackLaneObstacleAvoidCandidatePath(
    const autoware_planning_msgs::Trajectory::ConstPtr & msg_ptr);

  // timer function
  void timerCallback(const ros::TimerEvent & e);

  void emergencyParamCheck(const bool emergency_handling_param);
  void getCurrentPose();

  // parameter
  AutowareInfo aw_info_;
  std::unique_ptr<AutowareIvVehicleStatePublisher> vehicle_state_publisher_;
  std::unique_ptr<AutowareIvAutowareStatePublisher> autoware_state_publisher_;
  std::unique_ptr<AutowareIvStopReasonAggregator> stop_reason_aggreagator_;
  std::unique_ptr<AutowareIvLaneChangeStatePublisher> lane_change_state_publisher_;
  std::unique_ptr<AutowareIvObstacleAvoidanceStatePublisher> obstacle_avoidance_state_publisher_;
  double status_pub_hz_;
  double stop_reason_timeout_;
};

}  // namespace autoware_api
