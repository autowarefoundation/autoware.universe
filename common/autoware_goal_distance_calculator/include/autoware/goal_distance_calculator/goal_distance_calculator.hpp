// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_HPP_
#define AUTOWARE__GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_HPP_

#include <autoware/universe_utils/geometry/pose_deviation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <boost/optional.hpp>

namespace autoware::goal_distance_calculator
{
using autoware::universe_utils::PoseDeviation;

struct Param
{
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route;
};

struct Output
{
  PoseDeviation goal_deviation;
};

class GoalDistanceCalculator
{
public:
  static Output update(const Input & input);

  void setParam(const Param & param) { param_ = param; }

private:
  Param param_;
};
}  // namespace autoware::goal_distance_calculator

#endif  // AUTOWARE__GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_HPP_
