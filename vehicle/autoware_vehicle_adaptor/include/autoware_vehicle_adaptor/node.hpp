//  Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef AUTOWARE_VEHICLE_ADAPTOR__NODE_HPP_
#define AUTOWARE_VEHICLE_ADAPTOR__NODE_HPP_

#include "autoware/universe_utils/ros/logger_level_configure.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>

namespace autoware::vehicle_adaptor
{
using Control = autoware_control_msgs::msg::Control;

class VehicleAdaptorNode : public rclcpp::Node
{
public:
  explicit VehicleAdaptorNode(const rclcpp::NodeOptions & node_options);

  rclcpp::Subscription<Control>::SharedPtr sub_control_cmd_;
  rclcpp::Publisher<Control>::SharedPtr pub_control_cmd_;

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

private:
  void onControlCmd(const Control::ConstSharedPtr msg);
};
}  // namespace autoware::vehicle_adaptor

#endif  // AUTOWARE_VEHICLE_ADAPTOR__NODE_HPP_
