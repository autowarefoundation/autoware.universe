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
#include "awapi_awiv_adapter/awapi_obstacle_avoidance_state_publisher.hpp"
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
  rclcpp::Subscription<autoware_control_msgs::msg::EmergencyMode>::SharedPtr sub_emergency_;
  rclcpp::Subscription<autoware_planning_msgs::msg::StopReasonArray>::SharedPtr sub_stop_reason_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diagnostics_;
  rclcpp::Subscription<pacmod_msgs::msg::GlobalRpt>::SharedPtr sub_global_rpt_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_lane_change_available_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_lane_change_ready_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr sub_lane_change_candidate_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_obstacle_avoid_ready_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_obstacle_avoid_candidate_;
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
  void callbackIsEmergency(const autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr msg_ptr);
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

  // timer function
  void timerCallback();

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
