// Copyright 2023 Tier IV, Inc.
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

#include "remapper/remapper_core.hpp"

Remapper::Remapper()
: Node("remapper")
{
  using std::placeholders::_1;

  //Subscriber
  sub_operation_mode_availability_ = create_subscription<tier4_system_msgs::msg::OperationModeAvailability>(
      "~/input/system/operation_mode/availability", rclcpp::QoS{1}, std::bind(&Remapper::onOperationModeAvailability, this, _1));
  sub_to_can_bus_ = create_subscription<can_msgs::msg::Frame>(
      "~/input/j6/to_can_bus", rclcpp::QoS{1}, std::bind(&Remapper::onToCanBus, this, _1));
  sub_control_cmd_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "~/input/control/command/control_cmd", rclcpp::QoS{1}, std::bind(&Remapper::onControlCmd, this, _1));
  sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "~/input/tf", rclcpp::QoS{1}, std::bind(&Remapper::onTransform, this, _1));
  sub_operation_mode_state_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "~/input/api/operation_mode/state", rclcpp::QoS{1}, std::bind(&Remapper::onOperationModeState, this, _1));
  sub_initialization_state_ = create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
      "~/input/api/localization/initialization_state", rclcpp::QoS{1}, std::bind(&Remapper::onLocalizationInitializationState, this, _1));
  sub_pose_with_covariance_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/input/localization/pose_with_covariance", rclcpp::QoS{1}, std::bind(&Remapper::onPoseWithCovarianceStamped, this, _1));
  sub_routing_state_ = create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
      "~/input/api/routing/state", rclcpp::QoS{1}, std::bind(&Remapper::onRouteState, this, _1));
  sub_routing_route_ = create_subscription<autoware_adapi_v1_msgs::msg::Route>(
      "~/input/api/routing/route", rclcpp::QoS{1}, std::bind(&Remapper::onRoute, this, _1));
  sub_autoware_state_ = create_subscription<autoware_auto_system_msgs::msg::AutowareState>(
      "~/input/autoware/state", rclcpp::QoS{1}, std::bind(&Remapper::onAutowareState, this, _1));
  sub_controle_mode_report_ = create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      "~/input/vehicle/status/control_mode", rclcpp::QoS{1}, std::bind(&Remapper::onControlModeReport, this, _1));
  sub_get_emergency_ = create_subscription<tier4_external_api_msgs::msg::Emergency>(
      "~/input/api/external/get/emergency", rclcpp::QoS{1}, std::bind(&Remapper::onEmergency, this, _1));
  sub_mrm_state_ = create_subscription<autoware_adapi_v1_msgs::msg::MrmState>(
      "~/input/api/fail_safe/mrm_state", rclcpp::QoS{1}, std::bind(&Remapper::onMrmState, this, _1));
  sub_diagnostics_graph_ = create_subscription<tier4_system_msgs::msg::DiagnosticGraph>(
      "~/input/diagnostics_graph", rclcpp::QoS{1}, std::bind(&Remapper::onDiagnosticGraph, this, _1));
  sub_diagnostics_graph_supervisor_ = create_subscription<tier4_system_msgs::msg::DiagnosticGraph>(
      "~/input/diagnostics_graph/supervisor", rclcpp::QoS{1}, std::bind(&Remapper::onDiagnosticGraphSupervisor, this, _1));

  //Publisher
  pub_operation_mode_availability_ = create_publisher<tier4_system_msgs::msg::OperationModeAvailability>("~/output/system/operation_mode/availability", rclcpp::QoS{1});
  pub_to_can_bus_ = create_publisher<can_msgs::msg::Frame>("~/output/j6/to_can_bus", rclcpp::QoS{1});
  pub_control_cmd_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("~/output/control/command/control_cmd", rclcpp::QoS{1});
  pub_tf_ = create_publisher<tf2_msgs::msg::TFMessage>("~/output/tf", rclcpp::QoS{1});
  pub_operation_mode_state_ = create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>("~/output/api/operation_mode/state", rclcpp::QoS{1});
  pub_initialization_state_ = create_publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>("~/output/api/localization/initialization_state", rclcpp::QoS{1});
  pub_pose_with_covariance_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/localization/pose_with_covariance", rclcpp::QoS{1});
  pub_routing_state_ = create_publisher<autoware_adapi_v1_msgs::msg::RouteState>("~/output/api/routing/state", rclcpp::QoS{1});
  pub_routing_route_ = create_publisher<autoware_adapi_v1_msgs::msg::Route>("~/output/api/routing/route", rclcpp::QoS{1});
  pub_autoware_state_ = create_publisher<autoware_auto_system_msgs::msg::AutowareState>("~/output/autoware/state", rclcpp::QoS{1});
  pub_controle_mode_report_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("~/output/vehicle/status/control_mode", rclcpp::QoS{1});
  pub_get_emergency_ = create_publisher<tier4_external_api_msgs::msg::Emergency>("~/output/api/external/get/emergency", rclcpp::QoS{1});
  pub_mrm_state_ = create_publisher<autoware_adapi_v1_msgs::msg::MrmState>("~/output/api/fail_safe/mrm_state", rclcpp::QoS{1});
  pub_diagnostics_graph_ = create_publisher<tier4_system_msgs::msg::DiagnosticGraph>("~/output/diagnostics_graph", rclcpp::QoS{1});
  pub_diagnostics_graph_supervisor_ = create_publisher<tier4_system_msgs::msg::DiagnosticGraph>("~/output/diagnostics_graph/supervisor", rclcpp::QoS{1});
}

void Remapper::onOperationModeAvailability(const tier4_system_msgs::msg::OperationModeAvailability::ConstSharedPtr msg) {
  pub_operation_mode_availability_->publish(*msg);
}

void Remapper::onToCanBus(const can_msgs::msg::Frame::ConstSharedPtr msg) {
  pub_to_can_bus_->publish(*msg);
}

void Remapper::onControlCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg) {
  pub_control_cmd_->publish(*msg);
}

void Remapper::onTransform(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
  pub_tf_->publish(*msg);
}

void Remapper::onOperationModeState(const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg) {
  pub_operation_mode_state_->publish(*msg);
}

void Remapper::onLocalizationInitializationState(const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg) {
  pub_initialization_state_->publish(*msg);
}

void Remapper::onPoseWithCovarianceStamped(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
  pub_pose_with_covariance_->publish(*msg);
}

void Remapper::onRouteState(const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg) {
  pub_routing_state_->publish(*msg);
}

void Remapper::onRoute(const autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr msg) {
  pub_routing_route_->publish(*msg);
}

void Remapper::onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg) {
  pub_autoware_state_->publish(*msg);
}

void Remapper::onControlModeReport(const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg) {
  pub_controle_mode_report_->publish(*msg);
}

void Remapper::onEmergency(const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg) {
  pub_get_emergency_->publish(*msg);
}

void Remapper::onMrmState(const autoware_adapi_v1_msgs::msg::MrmState::ConstSharedPtr msg) {
  pub_mrm_state_->publish(*msg);
}

void Remapper::onDiagnosticGraph(const tier4_system_msgs::msg::DiagnosticGraph::ConstSharedPtr msg) {
  pub_diagnostics_graph_->publish(*msg);
}

void Remapper::onDiagnosticGraphSupervisor(const tier4_system_msgs::msg::DiagnosticGraph::ConstSharedPtr msg) {
  pub_diagnostics_graph_supervisor_->publish(*msg);
}


