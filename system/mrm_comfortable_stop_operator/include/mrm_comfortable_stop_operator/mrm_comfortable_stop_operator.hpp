// Copyright 2022 Tier IV, Inc.
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

#ifndef MRM_COMFORTABLE_STOP_OPERATOR_HPP_
#define MRM_COMFORTABLE_STOP_OPERATOR_HPP_

// Core
#include <memory>

// Autoware
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

namespace mrm_comfortable_stop_operator
{

struct Parameters
{
  double min_acceleration;  // [m/s^2]
  double max_jerk;          // [m/s^3]
  double min_jerk;          // [m/s^3]
};

class MrmComfortableStopOperator : public rclcpp::Node
{
public:
  explicit MrmComfortableStopOperator();
  bool operate();
  bool cancel();

private:
  // Parameters
  Parameters params_;

  // Publisher
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>::SharedPtr
    pub_velocity_limit_clear_command_;

  void publishVelocityLimit() const;
  void publishVelocityLimitClearCommand() const;
};

}  // namespace mrm_comfortable_stop_operator

#endif  // MRM_COMFORTABLE_STOP_OPERATOR_HPP_
