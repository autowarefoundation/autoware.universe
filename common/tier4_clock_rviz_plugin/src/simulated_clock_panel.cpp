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

#include "simulated_clock_panel.hpp"

#include <QString>
#include <QLineEdit>
#include <QHBoxLayout>
#include <rclcpp/duration.hpp>
#include <rviz_common/display_context.hpp>
#include <qspinbox.h>
#include <qt5/QtWidgets/qboxlayout.h>

#include <chrono>
#include <string>


namespace rviz_plugins
{
SimulatedClockPanel::SimulatedClockPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  pause_button_ = new QPushButton("Pause");
  pause_button_->setToolTip("Freeze ROS time.");
  pause_button_->setCheckable(true);

  publishing_rate_box_ = new QDoubleSpinBox();
  publishing_rate_box_->setRange(0.001, 1.0);
  publishing_rate_box_->setSingleStep(0.01);
  publishing_rate_box_->setValue(0.01);
  publishing_rate_box_->setSuffix("s");

  clock_speed_box_ = new QDoubleSpinBox();
  clock_speed_box_->setRange(0.0, 10.0);
  clock_speed_box_->setSingleStep(0.1);
  clock_speed_box_->setValue(1.0);
  clock_speed_box_->setSuffix(" X real time");

  auto * layout = new QHBoxLayout(this);
  layout->addWidget(pause_button_);
  layout->addWidget(new QLabel("Clock Speed:"));
  layout->addWidget(clock_speed_box_);
  layout->addWidget(new QLabel("Clock Publishing Rate:"));
  layout->addWidget(publishing_rate_box_);
  layout->setContentsMargins(0, 0, 20, 0);
  layout->addStretch();
  /*
  layout->setContentsMargins(11, 5, 11, 5);
  */
  prev_published_time_ = std::chrono::system_clock::now();

  connect(publishing_rate_box_, SIGNAL(valueChanged(double)), this, SLOT(onRateChanged(double)));
}

void SimulatedClockPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  clock_pub_ = raw_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1));
  createWallTimer();
}

void SimulatedClockPanel::onRateChanged(double /*new_rate*/) {
  pub_timer_->cancel();
  createWallTimer();
}

void SimulatedClockPanel::createWallTimer() {
  // convert rate from seconds to milliseconds
  const auto period = std::chrono::milliseconds(static_cast<int64_t>(publishing_rate_box_->value() * 1e3));
  pub_timer_ = raw_node_->create_wall_timer(period, [&]() { onTimer(); });
}

void SimulatedClockPanel::onTimer() {
  constexpr auto one_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(1)).count();
  const auto duration_since_prev_clock = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - prev_published_time_).count();
  if(!pause_button_->isChecked()) {
    clock_msg_.clock.nanosec += static_cast<uint32_t>(static_cast<double>(duration_since_prev_clock) * clock_speed_box_->value());
    if(clock_msg_.clock.nanosec >= one_sec) {
      clock_msg_.clock.sec += static_cast<int32_t>(clock_msg_.clock.nanosec / one_sec);
      clock_msg_.clock.nanosec = clock_msg_.clock.nanosec % one_sec;
    }
  }
  clock_pub_->publish(clock_msg_);
  prev_published_time_ = std::chrono::system_clock::now();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::SimulatedClockPanel, rviz_common::Panel)
