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

#ifndef SIDE_SHIFT_CONTROLLER_PANEL_HPP_
#define SIDE_SHIFT_CONTROLLER_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rviz_common/panel.hpp>
#endif

#include <tier4_planning_msgs/msg/lateral_offset.hpp>

#include <set>

namespace rviz_plugins
{

using tier4_planning_msgs::msg::LateralOffset;

class SideShiftControllerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SideShiftControllerPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:
  void onRateChanged(int new_rate);
  void onSetLateralOffset();
  void onResetLateralOffset();
  void onStartPublishing();
  void onStopPublishing();

protected:
  void onTimer();
  void createWallTimer();

  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<LateralOffset>::SharedPtr pub_offset_;

  QLineEdit * offset_value_;
  QPushButton * set_button_;
  QPushButton * reset_button_;
  QPushButton * publish_button_;
  QPushButton * stop_button_;
  QDoubleSpinBox * offset_input_;
  QSpinBox * publishing_rate_input_;

  bool publish_{false};
  float offset_{0.0};
};

}  // namespace rviz_plugins

#endif  // SIDE_SHIFT_CONTROLLER_PANEL_HPP_
