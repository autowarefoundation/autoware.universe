// Copyright 2021 Tier IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_PLANNER_RESAMPLE_HPP_
#define OBSTACLE_VELOCITY_PLANNER_RESAMPLE_HPP_

#include "autoware_utils/autoware_utils.hpp"

#include "autoware_perception_msgs/msg/predicted_path.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include "boost/optional.hpp"

#include <vector>

namespace resampling
{
autoware_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_perception_msgs::msg::PredictedPath & input_path, const std::vector<double> & time_vec,
  const rclcpp::Time & start_time, const double duration);

geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const double t);

bool lerpByTimeStamp(
  const autoware_perception_msgs::msg::PredictedPath & path, const rclcpp::Time & t,
  geometry_msgs::msg::Pose * lerped_pt);

autoware_planning_msgs::msg::Trajectory applyLinearInterpolation(
  const std::vector<double> & base_index, const autoware_planning_msgs::msg::Trajectory & base_trajectory,
  const std::vector<double> & out_index, const bool use_spline_for_pose = false);
}  // namespace resampling

#endif  // OBSTACLE_VELOCITY_PLANNER_RESAMPLE_HPP_
