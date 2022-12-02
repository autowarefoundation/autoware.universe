//
//  Copyright 2020 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef AUTOWARE_STATE_PANEL_HPP_
#define AUTOWARE_STATE_PANEL_HPP_

#include <QGroupBox>
#include <QLabel>
#include <QLayout>
#include <QPushButton>
#include <QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/srv/set_emergency.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

namespace rviz_plugins
{
class AutowareStatePanel : public rviz_common::Panel
{
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;

  Q_OBJECT

public:
  explicit AutowareStatePanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onClickAutonomous();
  void onClickStop();
  void onClickLocal();
  void onClickRemote();
  void onClickAutowareControl();
  void onClickDirectControl();
  void onClickVelocityLimit();
  void onClickEmergencyButton();

protected:
  // Layout
  QGroupBox * makeOperationModeGroup();
  QGroupBox * makeControlModeGroup();

  void onShift(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg);
  void onEmergencyStatus(const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr sub_gear_;

  rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedPtr client_emergency_stop_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::Emergency>::SharedPtr sub_emergency_;

  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;

  // Operation Mode
  //// Gate Mode
  QLabel * operation_mode_label_ptr_{nullptr};
  QPushButton * stop_button_ptr_{nullptr};
  QPushButton * auto_button_ptr_{nullptr};
  QPushButton * local_button_ptr_{nullptr};
  QPushButton * remote_button_ptr_{nullptr};

  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_autonomous_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_stop_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_local_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_remote_;

  //// Control Mode
  QLabel * control_mode_label_ptr_{nullptr};
  QPushButton * enable_button_ptr_{nullptr};
  QPushButton * disable_button_ptr_{nullptr};
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_enable_autoware_control_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_enable_direct_control_;

  //// Functions
  void onOperationMode(const OperationModeState::ConstSharedPtr msg);
  void changeOperationMode(const rclcpp::Client<ChangeOperationMode>::SharedPtr client);

  QPushButton * velocity_limit_button_ptr_;
  QLabel * gear_label_ptr_;

  QSpinBox * pub_velocity_limit_input_;
  QPushButton * emergency_button_ptr_;

  bool current_engage_{false};
  bool current_emergency_{false};
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_STATE_PANEL_HPP_
