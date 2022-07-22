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

#ifndef TIER4_DEBUG_TOOLS__TRAJECTORY_UNSEQUENCER_HPP_
#define TIER4_DEBUG_TOOLS__TRAJECTORY_UNSEQUENCER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <tier4_planning_msgs/msg/trajectory.hpp>

class TrajectoryUnsequencer : public rclcpp::Node
{
public:
  explicit TrajectoryUnsequencer(const rclcpp::NodeOptions & node_options);

private:
  /* Publishers and Subscribers */
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_ =
    create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/input", rclcpp::QoS{1},
      std::bind(&TrajectoryUnsequencer::onTrajectory, this, std::placeholders::_1));
  rclcpp::Publisher<tier4_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_ =
    create_publisher<tier4_planning_msgs::msg::Trajectory>("~/output", 1);

  /**
   * @brief Republish the trajectory in a format without sequence
   */
  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr input);
};

#endif  // TIER4_DEBUG_TOOLS__TRAJECTORY_UNSEQUENCER_HPP_
