// Copyright 2024 Tier IV, Inc.
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

#ifndef MOTION_UTILS__TRAJECTORY_V2__DETAIL__TYPES_HPP_
#define MOTION_UTILS__TRAJECTORY_V2__DETAIL__TYPES_HPP_

#include <Eigen/Dense>

#include "autoware_planning_msgs/msg/path_point.hpp"
#include "tier4_planning_msgs/msg/path_point_with_lane_id.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace motion_utils::trajectory_v2::detail
{

geometry_msgs::msg::Point to_point(const geometry_msgs::msg::Point & p)
{
  return p;
}
geometry_msgs::msg::Point to_point(const geometry_msgs::msg::Pose & p)
{
  return p.position;
}
geometry_msgs::msg::Point to_point(const Eigen::Ref<const Eigen::Vector2d> & p)
{
  geometry_msgs::msg::Point point;
  point.x = p(0);
  point.y = p(1);
  return point;
}
geometry_msgs::msg::Point to_point(const autoware_planning_msgs::msg::PathPoint & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.pose.position.x;
  point.y = p.pose.position.y;
  return point;
}
geometry_msgs::msg::Point to_point(const tier4_planning_msgs::msg::PathPointWithLaneId & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.point.pose.position.x;
  point.y = p.point.pose.position.y;
  return point;
}
}  // namespace motion_utils::trajectory_v2::detail

#endif  // MOTION_UTILS__TRAJECTORY_V2__DETAIL__TYPES_HPP_
