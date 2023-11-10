//  Copyright 2023 TIER IV, Inc. All rights reserved.
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

#include "side_shift_controller_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#undef signals
namespace rviz_plugins
{
SideShiftControllerPanel::SideShiftControllerPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Publish Rate
  publishing_rate_input_ = new QSpinBox();
  publishing_rate_input_->setRange(1, 100);
  publishing_rate_input_->setSingleStep(1);
  publishing_rate_input_->setValue(10);
  publishing_rate_input_->setSuffix("Hz");

  // Lateral Offset
  offset_input_ = new QDoubleSpinBox();
  offset_input_->setRange(-3.0, 3.0);
  offset_input_->setSingleStep(0.1);
  offset_input_->setValue(0.0);

  offset_value_ = new QLineEdit;
  offset_value_->setReadOnly(true);

  // Set Lateral Offset Button
  set_button_ = new QPushButton("SET");

  // Reset Lateral Offset Button
  reset_button_ = new QPushButton("RESET");

  // Stop publishing Button
  stop_button_ = new QPushButton("STOP PUBLISH");

  // Publishing Button
  publish_button_ = new QPushButton("PUBLISH");

  connect(publishing_rate_input_, SIGNAL(valueChanged(int)), this, SLOT(onRateChanged(int)));
  connect(set_button_, SIGNAL(clicked()), SLOT(onSetLateralOffset()));
  connect(reset_button_, SIGNAL(clicked()), SLOT(onResetLateralOffset()));
  connect(stop_button_, SIGNAL(clicked()), SLOT(onStopPublishing()));
  connect(publish_button_, SIGNAL(clicked()), SLOT(onStartPublishing()));

  auto * h_layout_1 = new QHBoxLayout;
  h_layout_1->addWidget(new QLabel("Rate: "));
  h_layout_1->addWidget(publishing_rate_input_);
  h_layout_1->addWidget(new QLabel("Target Offset: "));
  h_layout_1->addWidget(offset_input_);
  h_layout_1->addWidget(new QLabel("Current Offset: "));
  h_layout_1->addWidget(offset_value_);

  auto * h_layout_2 = new QHBoxLayout;
  h_layout_2->addWidget(set_button_);
  h_layout_2->addWidget(reset_button_);
  h_layout_2->addWidget(publish_button_);
  h_layout_2->addWidget(stop_button_);

  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(h_layout_1);
  v_layout->addLayout(h_layout_2);

  setLayout(v_layout);
}

void SideShiftControllerPanel::onSetLateralOffset()
{
  offset_ = offset_input_->value();
}

void SideShiftControllerPanel::onResetLateralOffset()
{
  offset_ = 0.0;
}

void SideShiftControllerPanel::onStopPublishing()
{
  publish_ = false;

  publish_button_->setText("PUBLISH");
  publish_button_->setStyleSheet("background-color: #FFFFFF");
}

void SideShiftControllerPanel::onStartPublishing()
{
  publish_ = true;

  publish_button_->setText("PUBLISHING...");
  publish_button_->setStyleSheet("background-color: #FFBF00");
}

void SideShiftControllerPanel::onInitialize()
{
  node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pub_offset_ = node_->create_publisher<LateralOffset>(
    "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/"
    "lateral_offset",
    rclcpp::QoS(1));

  createWallTimer();

  publish_ = false;
}

void SideShiftControllerPanel::onRateChanged(int new_rate)
{
  (void)new_rate;
  timer_->cancel();
  createWallTimer();
}

void SideShiftControllerPanel::createWallTimer()
{
  // convert rate from Hz to milliseconds
  const auto period =
    std::chrono::milliseconds(static_cast<int64_t>(1e3 / publishing_rate_input_->value()));
  timer_ = node_->create_wall_timer(period, [&]() { onTimer(); });
}

void SideShiftControllerPanel::onTimer()
{
  offset_value_->setText(QString::number(offset_));

  if (!publish_) {
    return;
  }

  LateralOffset msg;
  msg.stamp = rclcpp::Clock().now();
  msg.lateral_offset = offset_;
  pub_offset_->publish(msg);
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::SideShiftControllerPanel, rviz_common::Panel)
