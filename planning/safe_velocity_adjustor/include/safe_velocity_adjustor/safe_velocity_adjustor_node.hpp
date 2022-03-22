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

#ifndef SAFE_VELOCITY_ADJUSTOR__SAFE_VELOCITY_ADJUSTOR_NODE_HPP_
#define SAFE_VELOCITY_ADJUSTOR__SAFE_VELOCITY_ADJUSTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace safe_velocity_adjustor
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class SafeVelocityAdjustorNode : public rclcpp::Node
{
public:
  explicit SafeVelocityAdjustorNode(const rclcpp::NodeOptions & node_options)
  : rclcpp::Node("safe_velocity_adjustor", node_options)
  {
    // TODO(Maxime CLEMENT): declare/get ROS parameters
    time_safety_buffer_ = 0.5;
    dist_safety_buffer_ = 1.5;
  }

private:
  rclcpp::Publisher<Trajectory>::SharedPtr
    pub_trajectory_;  //!< @brief publisher for output trajectory
  rclcpp::Subscription<Trajectory>::SharedPtr
    sub_trajectory_;  //!< @brief subscriber for reference trajectory

  // parameters
  double time_safety_buffer_;
  double dist_safety_buffer_;

  // parameter update
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // topic callback
  void onTrajectory(const Trajectory::ConstSharedPtr msg)
  {
    Trajectory safe_trajectory = *msg;
    for (auto & trajectory_point : safe_trajectory.points) {
      const auto closest_collision_point = distanceToClosestCollision(trajectory_point);
      if (closest_collision_point)
        trajectory_point.longitudinal_velocity_mps =
          calculateSafeVelocity(trajectory_point, *closest_collision_point);
    }
    safe_trajectory.header.stamp = now();
    pub_trajectory_->publish(safe_trajectory);
    publishDebug(*msg, safe_trajectory);
  }

  // publish methods
  std::optional<double> distanceToClosestCollision(const TrajectoryPoint & trajectory_point)
  {
    (void)trajectory_point;
    return {};
  }

  double calculateSafeVelocity(
    const TrajectoryPoint & trajectory_point, const double & dist_to_collision)
  {
    return std::min(
      trajectory_point.longitudinal_velocity_mps,
      static_cast<decltype(trajectory_point.longitudinal_velocity_mps)>(
        dist_to_collision / time_safety_buffer_));
  }

  void publishDebug(
    const Trajectory & original_trajectory, const Trajectory & safe_trajectory) const
  {
    (void)original_trajectory;
    (void)safe_trajectory;
  }

  // debug
};
}  // namespace safe_velocity_adjustor

#endif  // SAFE_VELOCITY_ADJUSTOR__SAFE_VELOCITY_ADJUSTOR_NODE_HPP_
