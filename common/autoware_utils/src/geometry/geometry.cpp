// Copyright 2020 Tier IV, Inc.
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

#include "autoware_utils/geometry/geometry.hpp"

namespace autoware_utils
{
template<>
geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Point & p) {return p;}

template<>
geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Pose & p) {return p.position;}

template<>
geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseStamped & p)
{
  return p.pose.position;
}

template<>
geometry_msgs::msg::Point getPoint(const autoware_planning_msgs::msg::PathPoint & p)
{
  return p.pose.position;
}

template<>
geometry_msgs::msg::Point getPoint(const autoware_planning_msgs::msg::TrajectoryPoint & p)
{
  return p.pose.position;
}

template<>
geometry_msgs::msg::Pose getPose(const geometry_msgs::msg::Pose & p) {return p;}

template<>
geometry_msgs::msg::Pose getPose(const geometry_msgs::msg::PoseStamped & p)
{
  return p.pose;
}

template<>
geometry_msgs::msg::Pose getPose(const autoware_planning_msgs::msg::PathPoint & p)
{
  return p.pose;
}

template<>
geometry_msgs::msg::Pose getPose(const autoware_planning_msgs::msg::TrajectoryPoint & p)
{
  return p.pose;
}
}  // namespace autoware_utils
