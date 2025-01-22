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

#ifndef VEHICLE_CMD_GATE_HPP_
#define VEHICLE_CMD_GATE_HPP_

#include "adapi_pause_interface.hpp"
#include "autoware/universe_utils/ros/logger_level_configure.hpp"
#include "moderate_stop_interface.hpp"
#include "vehicle_cmd_filter.hpp"

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware_vehicle_cmd_gate/msg/is_filter_activated.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_debug_msgs/msg/bool_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/msg/heartbeat.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_emergency.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::vehicle_cmd_gate
{

using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_control_msgs::msg::Control;
using autoware_control_msgs::msg::Longitudinal;
using autoware_vehicle_cmd_gate::msg::IsFilterActivated;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using std_srvs::srv::Trigger;
using tier4_control_msgs::msg::GateMode;
using tier4_debug_msgs::msg::BoolStamped;
using tier4_external_api_msgs::msg::Emergency;
using tier4_external_api_msgs::msg::Heartbeat;
using tier4_external_api_msgs::srv::SetEmergency;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_vehicle_msgs::msg::VehicleEmergencyStamped;
using visualization_msgs::msg::MarkerArray;

using diagnostic_msgs::msg::DiagnosticStatus;
using nav_msgs::msg::Odometry;

using EngageMsg = autoware_vehicle_msgs::msg::Engage;
using EngageSrv = tier4_external_api_msgs::srv::Engage;

using autoware::motion_utils::VehicleStopChecker;
struct Commands
{
  Control control;
  TurnIndicatorsCommand turn_indicator;
  HazardLightsCommand hazard_light;
  GearCommand gear;
  explicit Commands(const uint8_t & default_gear = GearCommand::PARK)
  {
    gear.command = default_gear;
  }
};

class VehicleCmdGate : public rclcpp::Node
{
public:
  explicit VehicleCmdGate(const rclcpp::NodeOptions & node_options);

private:
  // Publisher
  rclcpp::Publisher<VehicleEmergencyStamped>::SharedPtr vehicle_cmd_emergency_pub_;
  rclcpp::Publisher<Control>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_cmd_pub_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_indicator_cmd_pub_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_light_cmd_pub_;
  rclcpp::Publisher<GateMode>::SharedPtr gate_mode_pub_;
  rclcpp::Publisher<EngageMsg>::SharedPtr engage_pub_;
  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_pub_;
  rclcpp::Publisher<IsFilterActivated>::SharedPtr is_filter_activated_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr filter_activated_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr filter_activated_marker_raw_pub_;
  rclcpp::Publisher<BoolStamped>::SharedPtr filter_activated_flag_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr processing_time_pub_;
  // Parameter callback
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  // Subscription
  rclcpp::Subscription<Heartbeat>::SharedPtr external_emergency_stop_heartbeat_sub_;
  rclcpp::Subscription<GateMode>::SharedPtr gate_mode_sub_;
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_sub_;
  rclcpp::Subscription<MrmState>::SharedPtr mrm_state_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr kinematics_sub_;             // for filter
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr acc_sub_;  // for filter
  rclcpp::Subscription<SteeringReport>::SharedPtr steer_sub_;            // for filter

  void onGateMode(GateMode::ConstSharedPtr msg);
  void onExternalEmergencyStopHeartbeat(Heartbeat::ConstSharedPtr msg);
  void onMrmState(MrmState::ConstSharedPtr msg);

  bool is_engaged_;
  bool is_system_emergency_ = false;
  bool is_external_emergency_stop_ = false;
  double current_steer_ = 0;
  GateMode current_gate_mode_;
  MrmState current_mrm_state_;
  Odometry current_kinematics_;
  double current_acceleration_ = 0.0;
  int filter_activated_count_ = 0;

  // Heartbeat
  std::shared_ptr<rclcpp::Time> emergency_state_heartbeat_received_time_;
  bool is_emergency_state_heartbeat_timeout_ = false;
  std::shared_ptr<rclcpp::Time> external_emergency_stop_heartbeat_received_time_;
  bool is_external_emergency_stop_heartbeat_timeout_ = false;
  bool isHeartbeatTimeout(
    const std::shared_ptr<rclcpp::Time> & heartbeat_received_time, const double timeout);

  // Check initialization
  bool isDataReady();

  // Subscriber for auto
  Commands auto_commands_;
  rclcpp::Subscription<Control>::SharedPtr auto_control_cmd_sub_;
  autoware::universe_utils::InterProcessPollingSubscriber<TurnIndicatorsCommand>
    auto_turn_indicator_cmd_sub_{this, "input/auto/turn_indicators_cmd"};
  autoware::universe_utils::InterProcessPollingSubscriber<HazardLightsCommand>
    auto_hazard_light_cmd_sub_{this, "input/auto/hazard_lights_cmd"};
  autoware::universe_utils::InterProcessPollingSubscriber<GearCommand> auto_gear_cmd_sub_{
    this, "input/auto/gear_cmd"};
  void onAutoCtrlCmd(Control::ConstSharedPtr msg);

  // Subscription for external
  Commands remote_commands_;
  rclcpp::Subscription<Control>::SharedPtr remote_control_cmd_sub_;
  autoware::universe_utils::InterProcessPollingSubscriber<TurnIndicatorsCommand>
    remote_turn_indicator_cmd_sub_{this, "input/external/turn_indicators_cmd"};
  autoware::universe_utils::InterProcessPollingSubscriber<HazardLightsCommand>
    remote_hazard_light_cmd_sub_{this, "input/external/hazard_lights_cmd"};
  autoware::universe_utils::InterProcessPollingSubscriber<GearCommand> remote_gear_cmd_sub_{
    this, "input/external/gear_cmd"};
  void onRemoteCtrlCmd(Control::ConstSharedPtr msg);

  // Subscription for emergency
  Commands emergency_commands_;
  rclcpp::Subscription<Control>::SharedPtr emergency_control_cmd_sub_;
  autoware::universe_utils::InterProcessPollingSubscriber<HazardLightsCommand>
    emergency_hazard_light_cmd_sub_{this, "input/emergency/hazard_lights_cmd"};
  autoware::universe_utils::InterProcessPollingSubscriber<GearCommand> emergency_gear_cmd_sub_{
    this, "input/emergency/gear_cmd"};
  void onEmergencyCtrlCmd(Control::ConstSharedPtr msg);

  // Previous Turn Indicators, Hazard Lights and Gear
  TurnIndicatorsCommand::SharedPtr prev_turn_indicator_;
  HazardLightsCommand::SharedPtr prev_hazard_light_;
  GearCommand::SharedPtr prev_gear_;

  // Parameter
  bool use_emergency_handling_;
  bool check_external_emergency_heartbeat_;
  double system_emergency_heartbeat_timeout_;
  double external_emergency_stop_heartbeat_timeout_;
  double stop_hold_acceleration_;
  double emergency_acceleration_;
  double moderate_stop_service_acceleration_;
  bool enable_cmd_limit_filter_;
  int filter_activated_count_threshold_;
  double filter_activated_velocity_threshold_;

  // Service
  rclcpp::Service<EngageSrv>::SharedPtr srv_engage_;
  rclcpp::Service<SetEmergency>::SharedPtr srv_external_emergency_;
  rclcpp::Publisher<Emergency>::SharedPtr pub_external_emergency_;
  void onEngageService(
    const EngageSrv::Request::SharedPtr request, const EngageSrv::Response::SharedPtr response);
  void onExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const SetEmergency::Request::SharedPtr request,
    const SetEmergency::Response::SharedPtr response);

  // TODO(Takagi, Isamu): deprecated
  rclcpp::Subscription<EngageMsg>::SharedPtr engage_sub_;
  rclcpp::Service<Trigger>::SharedPtr srv_external_emergency_stop_;
  rclcpp::Service<Trigger>::SharedPtr srv_clear_external_emergency_stop_;
  void onEngage(EngageMsg::ConstSharedPtr msg);
  bool onSetExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> req_header, const Trigger::Request::SharedPtr req,
    const Trigger::Response::SharedPtr res);
  bool onClearExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> req_header, const Trigger::Request::SharedPtr req,
    const Trigger::Response::SharedPtr res);

  // Timer / Event
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_pub_status_;

  void onTimer();
  void publishControlCommands(const Commands & commands);
  void publishEmergencyStopControlCommands();
  void publishStatus();

  // Diagnostics Updater
  diagnostic_updater::Updater updater_;

  void checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat);

  template <typename T>
  T getContinuousTopic(
    const std::shared_ptr<T> & prev_topic, const T & current_topic,
    const std::string & topic_name = "");

  // Algorithm
  Control prev_control_cmd_;
  Control createStopControlCmd() const;
  Longitudinal createLongitudinalStopControlCmd() const;
  Control createEmergencyStopControlCmd() const;

  std::shared_ptr<rclcpp::Time> prev_time_;
  double getDt();
  Control getActualStatusAsCommand();

  VehicleCmdFilter filter_;
  Control filterControlCommand(const Control & in);

  // filtering on transition
  OperationModeState current_operation_mode_;
  VehicleCmdFilter filter_on_transition_;

  // Pause interface for API
  std::unique_ptr<AdapiPauseInterface> adapi_pause_;
  std::unique_ptr<ModerateStopInterface> moderate_stop_interface_;

  // stop checker
  std::unique_ptr<VehicleStopChecker> vehicle_stop_checker_;
  double stop_check_duration_;

  // debug
  MarkerArray createMarkerArray(const IsFilterActivated & filter_activated);
  void publishMarkers(const IsFilterActivated & filter_activated);

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;
};

}  // namespace autoware::vehicle_cmd_gate
#endif  // VEHICLE_CMD_GATE_HPP_
