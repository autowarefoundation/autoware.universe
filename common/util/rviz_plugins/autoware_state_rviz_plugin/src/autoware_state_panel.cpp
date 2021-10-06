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
  // Gate Mode
  auto * gate_prefix_label_ptr = new QLabel("GATE: ");
  gate_prefix_label_ptr->setAlignment(Qt::AlignCenter);
  gate_mode_label_ptr_ = new QLabel("INIT");
  gate_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * gate_layout = new QHBoxLayout;
  gate_layout->addWidget(gate_prefix_label_ptr);
  gate_layout->addWidget(gate_mode_label_ptr_);

  // State
  auto * state_prefix_label_ptr = new QLabel("STATE: ");
  state_prefix_label_ptr->setAlignment(Qt::AlignCenter);
  autoware_state_label_ptr_ = new QLabel("INIT");
  autoware_state_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * state_layout = new QHBoxLayout;
  state_layout->addWidget(state_prefix_label_ptr);
  state_layout->addWidget(autoware_state_label_ptr_);

  // Gear
  auto * gear_prefix_label_ptr = new QLabel("GEAR: ");
  gear_prefix_label_ptr->setAlignment(Qt::AlignCenter);
  gear_label_ptr_ = new QLabel("INIT");
  gear_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * gear_layout = new QHBoxLayout;
  gear_layout->addWidget(gear_prefix_label_ptr);
  gear_layout->addWidget(gear_label_ptr_);

  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(gate_layout);
  v_layout->addLayout(state_layout);
  v_layout->addLayout(gear_layout);
  setLayout(v_layout);
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

  sub_gear_ = raw_node->create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "/vehicle/status/shift", 10,
    std::bind(&AutowareStatePanel::onShift, this, _1));
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

void AutowareStatePanel::onShift(
  const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  switch (msg->shift.data) {
    case autoware_vehicle_msgs::msg::Shift::NONE:
      gear_label_ptr_->setText("NONE");
      break;
    case autoware_vehicle_msgs::msg::Shift::PARKING:
      gear_label_ptr_->setText("PARKING");
      break;
    case autoware_vehicle_msgs::msg::Shift::REVERSE:
      gear_label_ptr_->setText("REVERSE");
      break;
    case autoware_vehicle_msgs::msg::Shift::NEUTRAL:
      gear_label_ptr_->setText("NEUTRAL");
      break;
    case autoware_vehicle_msgs::msg::Shift::DRIVE:
      gear_label_ptr_->setText("DRIVE");
      break;
    case autoware_vehicle_msgs::msg::Shift::LOW:
      gear_label_ptr_->setText("LOW");
      break;
  }
}
}  // namespace rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareStatePanel, rviz_common::Panel)
