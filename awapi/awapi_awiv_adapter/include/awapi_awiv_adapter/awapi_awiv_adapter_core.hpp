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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_AWIV_ADAPTER_CORE_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_AWIV_ADAPTER_CORE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/msg/emergency_mode.hpp"
#include "autoware_control_msgs/msg/gate_mode.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/stop_reason_array.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_vehicle_msgs/msg/control_mode.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/steering.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pacmod_msgs/msg/global_rpt.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "awapi_awiv_adapter/awapi_autoware_state_publisher.hpp"
#include "awapi_awiv_adapter/awapi_autoware_util.hpp"
#include "awapi_awiv_adapter/awapi_lane_change_state_publisher.hpp"
#include "awapi_awiv_adapter/awapi_max_velocity_publisher.hpp"
#include "awapi_awiv_adapter/awapi_obstacle_avoidance_state_publisher.hpp"
#include "awapi_awiv_adapter/awapi_pacmod_util.hpp"
#include "awapi_awiv_adapter/awapi_stop_reason_aggregator.hpp"
#include "awapi_awiv_adapter/awapi_vehicle_state_publisher.hpp"

namespace autoware_api
{
class AutowareIvAdapter : public rclcpp::Node
{
public:
  AutowareIvAdapter();

private:
  // subscriber
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Steering>::SharedPtr sub_steer_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr sub_vehicle_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr sub_turn_signal_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr sub_gear_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_battery_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_nav_sat_;
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr sub_autoware_state_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlMode>::SharedPtr sub_control_mode_;
  rclcpp::Subscription<autoware_control_msgs::msg::GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emergency_;
  rclcpp::Subscription<autoware_system_msgs::msg::HazardStatusStamped>::SharedPtr
    sub_hazard_status_;
  rclcpp::Subscription<autoware_planning_msgs::msg::StopReasonArray>::SharedPtr sub_stop_reason_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diagnostics_;
  rclcpp::Subscription<pacmod_msgs::msg::GlobalRpt>::SharedPtr sub_global_rpt_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_lane_change_available_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_lane_change_ready_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr sub_lane_change_candidate_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_obstacle_avoid_ready_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_obstacle_avoid_candidate_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_max_velocity_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_temporary_stop_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_autoware_traj_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_door_control_;
  rclcpp::Subscription<pacmod_msgs::msg::SystemRptInt>::SharedPtr sub_door_status_;

  // publisher
  rclcpp::Publisher<pacmod_msgs::msg::SystemCmdInt>::SharedPtr pub_door_control_;
  rclcpp::Publisher<autoware_api_msgs::msg::DoorStatus>::SharedPtr pub_door_status_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // callback function
  void callbackSteer(const autoware_vehicle_msgs::msg::Steering::ConstSharedPtr msg_ptr);
  void callbackVehicleCmd(const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg_ptr);
  void callbackTurnSignal(const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg_ptr);
  void callbackTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg_ptr);
  void callbackGear(const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg_ptr);
  void callbackBattery(const std_msgs::msg::Float32::ConstSharedPtr msg_ptr);
  void callbackNavSat(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg_ptr);
  void callbackAutowareState(
    const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg_ptr);
  void callbackControlMode(const autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr msg_ptr);
  void callbackGateMode(const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg_ptr);
  void callbackIsEmergency(const std_msgs::msg::Bool::ConstSharedPtr msg_ptr);
  void callbackHazardStatus(
    const autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg_ptr);
  void callbackStopReason(
    const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr msg_ptr);
  void callbackDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg_ptr);
  void callbackGlobalRpt(const pacmod_msgs::msg::GlobalRpt::ConstSharedPtr msg_ptr);
  void callbackLaneChangeAvailable(const std_msgs::msg::Bool::ConstSharedPtr msg_ptr);
  void callbackLaneChangeReady(const std_msgs::msg::Bool::ConstSharedPtr msg_ptr);
  void callbackLaneChangeCandidatePath(
    const autoware_planning_msgs::msg::Path::ConstSharedPtr msg_ptr);
  void callbackLaneObstacleAvoidReady(const std_msgs::msg::Bool::ConstSharedPtr msg_ptr);
  void callbackLaneObstacleAvoidCandidatePath(
    const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr);
  void callbackMaxVelocity(const std_msgs::msg::Float32::ConstSharedPtr msg_ptr);
  void callbackTemporaryStop(const std_msgs::msg::Bool::ConstSharedPtr msg_ptr);
  void callbackAutowareTrajectory(
    const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr);
  void callbackDoorControl(const std_msgs::msg::Bool::ConstSharedPtr msg_ptr);
  void callbackDoorStatus(const pacmod_msgs::msg::SystemRptInt::ConstSharedPtr msg_ptr);

  // timer function
  void timerCallback();

  void emergencyParamCheck(const bool emergency_stop_param);
  void getCurrentPose();

  // parameter
  AutowareInfo aw_info_;
  std::unique_ptr<AutowareIvVehicleStatePublisher> vehicle_state_publisher_;
  std::unique_ptr<AutowareIvAutowareStatePublisher> autoware_state_publisher_;
  std::unique_ptr<AutowareIvStopReasonAggregator> stop_reason_aggregator_;
  std::unique_ptr<AutowareIvLaneChangeStatePublisher> lane_change_state_publisher_;
  std::unique_ptr<AutowareIvObstacleAvoidanceStatePublisher>
  obstacle_avoidance_state_publisher_;
  std::unique_ptr<AutowareIvMaxVelocityPublisher> max_velocity_publisher_;
  double status_pub_hz_;
  double stop_reason_timeout_;
  double default_max_velocity;
  double stop_reason_thresh_dist_;
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_AWIV_ADAPTER_CORE_HPP_
