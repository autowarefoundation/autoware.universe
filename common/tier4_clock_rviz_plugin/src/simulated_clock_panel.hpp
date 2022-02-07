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

#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rviz_common/panel.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <memory>

namespace rviz_plugins
{
class SimulatedClockPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SimulatedClockPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected Q_SLOTS:
  void onRateChanged(int new_rate);
  void onStepClicked();

protected:
  void onTimer();
  void createWallTimer();
  void addTimeToClock(const uint32_t ns);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rosgraph_msgs::msg::Clock clock_msg_;

  QPushButton * pause_button_;
  QPushButton * step_button_;
  QSpinBox * publishing_rate_input_;
  QDoubleSpinBox * clock_speed_input_;
  QSpinBox * step_time_input_;
  QComboBox * step_unit_combo_;

  std::chrono::time_point<std::chrono::system_clock> prev_published_time_;
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_STATE_PANEL_HPP_
