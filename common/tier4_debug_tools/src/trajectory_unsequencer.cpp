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

#include "tier4_debug_tools/trajectory_unsequencer.hpp"

#include "tier4_planning_msgs/msg/trajectory_point.hpp"

TrajectoryUnsequencer::TrajectoryUnsequencer(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("trajectory_unsequencer", node_options)
{
}

void TrajectoryUnsequencer::onTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr input)
{
  tier4_planning_msgs::msg::Trajectory output;
  output.header = input->header;
  output.points.reserve(input->points.size());
  tier4_planning_msgs::msg::TrajectoryPoint new_point;
  for (const auto & point : input->points) {
    new_point.pose = point.pose;
    new_point.accel.linear.x = point.acceleration_mps2;
    new_point.twist.linear.x = point.longitudinal_velocity_mps;
    new_point.twist.linear.y = point.lateral_velocity_mps;
    output.points.push_back(new_point);
  }
  pub_trajectory_->publish(output);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryUnsequencer)
