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

#ifndef REMAPPER__REMAPPER_CORE_HPP_
#define REMAPPER__REMAPPER_CORE_HPP_

# include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include <watchdog_system_msgs/msg/switch_status.hpp>
#include <tier4_system_msgs/msg/operation_mode_availability.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route.hpp>

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <tier4_system_msgs/msg/diagnostic_graph.hpp>



class Remapper : public rclcpp::Node
{
public:
  explicit Remapper();

private:
  // Subscriber
  rclcpp::Subscription<tier4_system_msgs::msg::OperationModeAvailability>::SharedPtr sub_operation_mode_availability_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_to_can_bus_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;

  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_operation_mode_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr sub_initialization_state_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_covariance_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_routing_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr sub_routing_route_;

  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr sub_autoware_state_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr sub_controle_mode_report_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::Emergency>::SharedPtr sub_get_emergency_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MrmState>::SharedPtr sub_mrm_state_;
  rclcpp::Subscription<tier4_system_msgs::msg::DiagnosticGraph>::SharedPtr sub_diagnostics_graph_;
  rclcpp::Subscription<tier4_system_msgs::msg::DiagnosticGraph>::SharedPtr sub_diagnostics_graph_supervisor_;

  // Publisher
  rclcpp::Publisher<tier4_system_msgs::msg::OperationModeAvailability>::SharedPtr pub_operation_mode_availability_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_to_can_bus_;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr pub_control_cmd_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;

  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr pub_operation_mode_state_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr pub_initialization_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_with_covariance_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr pub_routing_state_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::Route>::SharedPtr pub_routing_route_;
  
  rclcpp::Publisher<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr pub_autoware_state_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr pub_controle_mode_report_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Emergency>::SharedPtr pub_get_emergency_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::MrmState>::SharedPtr pub_mrm_state_;
  rclcpp::Publisher<tier4_system_msgs::msg::DiagnosticGraph>::SharedPtr pub_diagnostics_graph_;
  rclcpp::Publisher<tier4_system_msgs::msg::DiagnosticGraph>::SharedPtr pub_diagnostics_graph_supervisor_;

  void onOperationModeAvailability(const tier4_system_msgs::msg::OperationModeAvailability::ConstSharedPtr msg);
  void onToCanBus(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void onControlCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onTransform(const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);

  void onOperationModeState(const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg);
  void onLocalizationInitializationState(const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg);
  void onPoseWithCovarianceStamped(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void onRouteState(const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg);
  void onRoute(const autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr msg);

  void onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  void onControlModeReport(const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg);
  void onEmergency(const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg);
  void onMrmState(const autoware_adapi_v1_msgs::msg::MrmState::ConstSharedPtr msg);
  void onDiagnosticGraph(const tier4_system_msgs::msg::DiagnosticGraph::ConstSharedPtr msg);
  void onDiagnosticGraphSupervisor(const tier4_system_msgs::msg::DiagnosticGraph::ConstSharedPtr msg);

};

#endif  // REMAPPER__REMAPPER_CORE_HPP_