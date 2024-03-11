// Copyright 2024 TIER IV, Inc.
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

#ifndef OPERATORS__EMERGENCY_STOP_HPP_
#define OPERATORS__EMERGENCY_STOP_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

namespace emergency_handler
{

namespace emergency_stop_operator
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;

struct Param
{
  double target_acceleration;
  double target_jerk;
};

class EmergencyStopOperator
{
public:
  explicit EmergencyStopOperator(rclcpp::Node * node);
  bool operate();
  bool cancel();
  void onTimer();

private:
  // Parameters
  Param params_;

  // Subscriber
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;
  void onControlCommand(AckermannControlCommand::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_cmd_;

  // Alrogithm
  bool is_prev_control_cmd_subscribed_;
  AckermannControlCommand prev_control_cmd_;
  AckermannControlCommand calcNextControlCmd();

  rclcpp::Node * node_;
};

}  // namespace emergency_stop_operator

}  // namespace emergency_handler

#endif  // OPERATORS__EMERGENCY_STOP_HPP_
