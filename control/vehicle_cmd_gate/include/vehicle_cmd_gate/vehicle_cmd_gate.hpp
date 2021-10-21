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

#ifndef VEHICLE_CMD_GATE__VEHICLE_CMD_GATE_HPP_
#define VEHICLE_CMD_GATE__VEHICLE_CMD_GATE_HPP_

#include <memory>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/msg/control_command_stamped.hpp"
#include "autoware_system_msgs/msg/emergency_state_stamped.hpp"
#include "autoware_control_msgs/msg/gate_mode.hpp"
#include "autoware_debug_msgs/msg/bool_stamped.hpp"
#include "autoware_external_api_msgs/msg/heartbeat.hpp"
#include "autoware_external_api_msgs/msg/emergency.hpp"
#include "autoware_external_api_msgs/srv/engage.hpp"
#include "autoware_external_api_msgs/srv/set_emergency.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/steering.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "vehicle_cmd_gate/vehicle_cmd_filter.hpp"

struct Commands
{
  autoware_control_msgs::msg::ControlCommandStamped control;
  autoware_vehicle_msgs::msg::TurnSignal turn_signal;
  autoware_vehicle_msgs::msg::ShiftStamped shift;
};

class VehicleCmdGate : public rclcpp::Node
{
public:
  explicit VehicleCmdGate(const rclcpp::NodeOptions & node_options);

private:
  // Publisher
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr shift_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr turn_signal_cmd_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::GateMode>::SharedPtr gate_mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_pub_;

  // Subscription
  rclcpp::Subscription<autoware_system_msgs::msg::EmergencyStateStamped>::SharedPtr
    emergency_state_sub_;
  rclcpp::Subscription<autoware_external_api_msgs::msg::Heartbeat>::SharedPtr
    external_emergency_stop_heartbeat_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::GateMode>::SharedPtr gate_mode_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Steering>::SharedPtr steer_sub_;

  void onGateMode(autoware_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onEmergencyState(autoware_system_msgs::msg::EmergencyStateStamped::ConstSharedPtr msg);
  void onExternalEmergencyStopHeartbeat(
    autoware_external_api_msgs::msg::Heartbeat::ConstSharedPtr msg);
  void onSteering(autoware_vehicle_msgs::msg::Steering::ConstSharedPtr msg);

  bool is_engaged_;
  bool is_system_emergency_ = false;
  bool is_external_emergency_stop_ = false;
  double current_steer_ = 0;
  autoware_control_msgs::msg::GateMode current_gate_mode_;

  // Heartbeat
  std::shared_ptr<rclcpp::Time> emergency_state_heartbeat_received_time_;
  bool is_emergency_state_heartbeat_timeout_ = false;
  std::shared_ptr<rclcpp::Time> external_emergency_stop_heartbeat_received_time_;
  bool is_external_emergency_stop_heartbeat_timeout_ = false;
  bool isHeartbeatTimeout(
    const std::shared_ptr<rclcpp::Time> & heartbeat_received_time, const double timeout);

  // Subscriber for auto
  Commands auto_commands_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    auto_control_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr auto_turn_signal_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr auto_shift_cmd_sub_;
  void onAutoCtrlCmd(autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg);
  void onAutoTurnSignalCmd(autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);
  void onAutoShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);

  // Subscription for external
  Commands remote_commands_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    remote_control_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr
    remote_turn_signal_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr remote_shift_cmd_sub_;
  void onRemoteCtrlCmd(autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg);
  void onRemoteTurnSignalCmd(autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);
  void onRemoteShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);

  // Subscription for emergency
  Commands emergency_commands_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    emergency_control_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr
    emergency_turn_signal_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr
    emergency_shift_cmd_sub_;
  void onEmergencyCtrlCmd(autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg);
  void onEmergencyTurnSignalCmd(autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);
  void onEmergencyShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);

  // Parameter
  double update_period_;
  bool use_emergency_handling_;
  bool use_external_emergency_stop_;
  double system_emergency_heartbeat_timeout_;
  double external_emergency_stop_heartbeat_timeout_;

  // Service
  rclcpp::Service<autoware_external_api_msgs::srv::Engage>::SharedPtr srv_engage_;
  rclcpp::Service<autoware_external_api_msgs::srv::SetEmergency>::SharedPtr srv_external_emergency_;
  rclcpp::Publisher<autoware_external_api_msgs::msg::Emergency>::SharedPtr pub_external_emergency_;
  void onEngageService(
    const autoware_external_api_msgs::srv::Engage::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::Engage::Response::SharedPtr response);
  void onExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<autoware_external_api_msgs::srv::SetEmergency::Request> request,
    const std::shared_ptr<autoware_external_api_msgs::srv::SetEmergency::Response> response);

  // TODO(Takagi, Isamu): deprecated
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_external_emergency_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_external_emergency_stop_;
  void onEngage(autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
  bool onSetExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  bool onClearExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Timer / Event
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();
  void publishControlCommands(const Commands & input_msg);
  void publishEmergencyStopControlCommands();

  // Diagnostics Updater
  diagnostic_updater::Updater updater_;

  void checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Algorithm
  autoware_control_msgs::msg::ControlCommand prev_control_cmd_;
  autoware_control_msgs::msg::ControlCommand createStopControlCmd() const;
  autoware_control_msgs::msg::ControlCommand createEmergencyStopControlCmd() const;

  std::shared_ptr<rclcpp::Time> prev_time_;
  double getDt();

  VehicleCmdFilter filter_;
  autoware_control_msgs::msg::ControlCommand filterControlCommand(
    const autoware_control_msgs::msg::ControlCommand & msg);

  // Start request service
  struct StartRequest
  {
private:
    static constexpr double eps = 1e-3;
    using ControlCommandStamped = autoware_control_msgs::msg::ControlCommandStamped;

public:
    StartRequest(rclcpp::Node * node, bool use_start_request);
    bool isAccepted();
    void publishStartAccepted();
    void checkStopped(const ControlCommandStamped & control);
    void checkStartRequest(const ControlCommandStamped & control);

private:
    bool use_start_request_;
    bool is_start_requesting_;
    bool is_start_accepted_;
    bool is_start_cancelled_;
    geometry_msgs::msg::TwistStamped current_twist_;

    rclcpp::Node * node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr request_start_cli_;
    rclcpp::Publisher<autoware_debug_msgs::msg::BoolStamped>::SharedPtr request_start_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_twist_sub_;
    void onCurrentTwist(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  };

  std::unique_ptr<StartRequest> start_request_;
};

#endif  // VEHICLE_CMD_GATE__VEHICLE_CMD_GATE_HPP_
