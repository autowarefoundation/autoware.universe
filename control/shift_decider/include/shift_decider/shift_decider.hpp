// Copyright 2020 Tier IV, Inc.
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

#ifndef CONTROL_SHIFT_DECIDER_INCLUDE_SHIFT_DECIDER_NODE_HPP_
#define CONTROL_SHIFT_DECIDER_INCLUDE_SHIFT_DECIDER_NODE_HPP_

#include "autoware_control_msgs/msg/control_command_stamped.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>

class ShiftDecider : public rclcpp::Node
{
public:
  ShiftDecider();

private:
  void onTimer();
  void onControlCmd(autoware_control_msgs::msg::ControlCommandStamped::SharedPtr msg);
  void updateCurrentShiftCmd();
  void initTimer(double period_s);

  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr pub_shift_cmd_;
  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    sub_control_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  autoware_control_msgs::msg::ControlCommandStamped::SharedPtr control_cmd_;
  autoware_vehicle_msgs::msg::ShiftStamped shift_cmd_;
};

#endif  // CONTROL_SHIFT_DECIDER_INCLUDE_SHIFT_DECIDER_NODE_HPP_
