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

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <QLabel>
#include <memory>

#include "autoware_control_msgs/msg/gate_mode.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace rviz_plugins
{

class AutowareStatePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AutowareStatePanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected:
  void onGateMode(const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onAutowareState(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg);

  rclcpp::Subscription<autoware_control_msgs::msg::GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr sub_autoware_state_;
  QLabel * gate_mode_label_ptr_;
  QLabel * autoware_state_label_ptr_;
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_STATE_PANEL_HPP_
