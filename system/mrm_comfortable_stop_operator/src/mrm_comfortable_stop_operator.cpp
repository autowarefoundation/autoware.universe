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

#include "mrm_comfortable_stop_operator/mrm_comfortable_stop_operator.hpp"

namespace mrm_comfortable_stop_operator
{

MrmComfortableStopOperator::MrmComfortableStopOperator() : Node("mrm_comfortable_stop_operator")
{
  // Parameter
  params_.min_acceleration = declare_parameter<double>("min_acceleration");
  params_.max_jerk = declare_parameter<double>("max_jerk");
  params_.min_jerk = declare_parameter<double>("min_jerk");

  // Publisher
  pub_velocity_limit_ = create_publisher<tier4_planning_msgs::msg::VelocityLimit>(
    "~/output/velocity_limit", rclcpp::QoS{1}.transient_local());
  pub_velocity_limit_clear_command_ =
    create_publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>(
      "~/output/velocity_limit/clear", rclcpp::QoS{1}.transient_local());
}

bool MrmComfortableStopOperator::operate()
{
  publishVelocityLimit();

  // Currently, ComfortableStopOperator does not return false
  return true;
}

bool MrmComfortableStopOperator::cancel()
{
  publishVelocityLimitClearCommand();

  // Currently, ComfortableStopOperator does not return false
  return true;
}

void MrmComfortableStopOperator::publishVelocityLimit() const
{
  auto velocity_limit = tier4_planning_msgs::msg::VelocityLimit();
  velocity_limit.stamp = this->now();
  velocity_limit.max_velocity = 0;
  velocity_limit.use_constraints = true;
  velocity_limit.constraints.min_acceleration = static_cast<float>(params_.min_acceleration);
  velocity_limit.constraints.max_jerk = static_cast<float>(params_.max_jerk);
  velocity_limit.constraints.min_jerk = static_cast<float>(params_.min_jerk);
  velocity_limit.sender = "mrm_comfortable_stop_operator";

  pub_velocity_limit_->publish(velocity_limit);
}

void MrmComfortableStopOperator::publishVelocityLimitClearCommand() const
{
  auto velocity_limit_clear_command = tier4_planning_msgs::msg::VelocityLimitClearCommand();
  velocity_limit_clear_command.stamp = this->now();
  velocity_limit_clear_command.command = true;
  velocity_limit_clear_command.sender = "mrm_comfortable_stop_operator";

  pub_velocity_limit_clear_command_->publish(velocity_limit_clear_command);
}

}  // namespace mrm_comfortable_stop_operator
