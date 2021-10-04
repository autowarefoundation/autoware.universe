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

#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include "autoware_state_panel.hpp"

using std::placeholders::_1;

namespace rviz_plugins
{
AutowareStatePanel::AutowareStatePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  gate_mode_label_ptr_ = new QLabel("INIT");
  gate_mode_label_ptr_->setAlignment(Qt::AlignCenter);

  autoware_state_label_ptr_ = new QLabel("INIT");
  autoware_state_label_ptr_->setAlignment(Qt::AlignCenter);

  auto * layout = new QVBoxLayout;
  layout->addWidget(gate_mode_label_ptr_);
  layout->addWidget(autoware_state_label_ptr_);
  setLayout(layout);
}

void AutowareStatePanel::onInitialize()
{
  rclcpp::Node::SharedPtr raw_node =
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_gate_mode_ = raw_node->create_subscription<autoware_control_msgs::msg::GateMode>(
    "/control/current_gate_mode", 10,
    std::bind(&AutowareStatePanel::onGateMode, this, _1));

  sub_autoware_state_ = raw_node->create_subscription<autoware_system_msgs::msg::AutowareState>(
    "/autoware/state", 10,
    std::bind(&AutowareStatePanel::onAutowareState, this, _1));
}

void AutowareStatePanel::onGateMode(
  const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  switch (msg->data) {
    case autoware_control_msgs::msg::GateMode::AUTO:
      gate_mode_label_ptr_->setText("AUTO");
      gate_mode_label_ptr_->setStyleSheet("background-color: #00FF00;");
      break;

    case autoware_control_msgs::msg::GateMode::EXTERNAL:
      gate_mode_label_ptr_->setText("EXTERNAL");
      gate_mode_label_ptr_->setStyleSheet("background-color: #FFFF00;");
      break;

    default:
      gate_mode_label_ptr_->setText("UNKNOWN");
      gate_mode_label_ptr_->setStyleSheet("background-color: #FF0000;");
      break;
  }
}

void AutowareStatePanel::onAutowareState(
  const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg)
{
  if (msg->state == autoware_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE) {
    autoware_state_label_ptr_->setText(msg->state.c_str());
    autoware_state_label_ptr_->setStyleSheet("background-color: #00FFFF;");
  } else if (msg->state == autoware_system_msgs::msg::AutowareState::DRIVING) {
    autoware_state_label_ptr_->setText(msg->state.c_str());
    autoware_state_label_ptr_->setStyleSheet("background-color: #00FF00;");
  } else if (msg->state == autoware_system_msgs::msg::AutowareState::ARRIVAL_GOAL) {
    autoware_state_label_ptr_->setText(msg->state.c_str());
    autoware_state_label_ptr_->setStyleSheet("background-color: #FF00FF;");
  } else if (msg->state == autoware_system_msgs::msg::AutowareState::EMERGENCY) {
    autoware_state_label_ptr_->setText("Stop");
    autoware_state_label_ptr_->setStyleSheet("background-color: #0000FF;");
  } else {
    autoware_state_label_ptr_->setText(msg->state.c_str());
    autoware_state_label_ptr_->setStyleSheet("background-color: #FFFF00;");
  }
}
}  // namespace rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareStatePanel, rviz_common::Panel)
