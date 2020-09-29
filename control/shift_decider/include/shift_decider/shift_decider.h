/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CONTROL_SHIFT_DECIDER_INCLUDE_SHIFT_DECIDER_NODE_HPP_
#define CONTROL_SHIFT_DECIDER_INCLUDE_SHIFT_DECIDER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>

class ShiftDecider
{
public:
  ShiftDecider();

private:
  void onTimer(const ros::TimerEvent & e);
  void onControlCmd(const autoware_control_msgs::ControlCommandStamped::ConstPtr msg);
  void updateCurrentShiftCmd();

  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};
  ros::Publisher pub_shift_cmd_;
  ros::Subscriber sub_control_cmd_;
  ros::Timer timer_;

  autoware_control_msgs::ControlCommandStamped::ConstPtr control_cmd_;
  autoware_vehicle_msgs::ShiftStamped shift_cmd_;
};

#endif  // CONTROL_SHIFT_DECIDER_INCLUDE_SHIFT_DECIDER_NODE_HPP_
