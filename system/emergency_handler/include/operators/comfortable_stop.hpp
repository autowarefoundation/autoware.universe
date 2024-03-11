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

#ifndef COMFORTABLE_STOP_HPP_
#define COMFORTABLE_STOP_HPP_

#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <rclcpp/rclcpp.hpp>

namespace emergency_handler
{

namespace comfortable_stop_operator
{

struct Param
{
  double min_acceleration;
  double max_jerk;
  double min_jerk;
};

class ComfortableStopOperator
{
public:
  explicit ComfortableStopOperator(rclcpp::Node * node);
  bool operate();
  bool cancel();
  void onTimer();

private:
  // Parameters
  Param params_;

  // Publisher
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>::SharedPtr
    pub_velocity_limit_clear_command_;

  // Algorithm
  void publishVelocityLimit();
  void publishVelocityLimitClearCommand();

  rclcpp::Node * node_;
};

} // namespace comfortable_stop_operator

} // namespace emergency_handler

#endif // COMFORTABLE_STOP_HPP_
