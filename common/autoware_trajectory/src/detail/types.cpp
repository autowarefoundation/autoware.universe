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

#include "autoware/trajectory/detail/types.hpp"

namespace autoware::trajectory::detail
{
MutablePoint3d to_point(geometry_msgs::msg::Point & p)
{
  return {p.x, p.y, p.z};
}

MutablePoint3d to_point(geometry_msgs::msg::Pose & p)
{
  return {p.position.x, p.position.y, p.position.z};
}

MutablePoint3d to_point(autoware_planning_msgs::msg::PathPoint & p)
{
  return {p.pose.position.x, p.pose.position.y, p.pose.position.z};
}

MutablePoint3d to_point(tier4_planning_msgs::msg::PathPointWithLaneId & p)
{
  return {p.point.pose.position.x, p.point.pose.position.y, p.point.pose.position.z};
}

MutablePoint2d to_point(lanelet::BasicPoint2d & p)
{
  return {p.x(), p.y()};
}

ImmutablePoint3d to_point(const geometry_msgs::msg::Point & p)
{
  return {p.x, p.y, p.z};
}

ImmutablePoint3d to_point(const geometry_msgs::msg::Pose & p)
{
  return {p.position.x, p.position.y, p.position.z};
}

ImmutablePoint3d to_point(const autoware_planning_msgs::msg::PathPoint & p)
{
  return {p.pose.position.x, p.pose.position.y, p.pose.position.z};
}

ImmutablePoint3d to_point(const tier4_planning_msgs::msg::PathPointWithLaneId & p)
{
  return {p.point.pose.position.x, p.point.pose.position.y, p.point.pose.position.z};
}

ImmutablePoint2d to_point(const lanelet::BasicPoint2d & p)
{
  return {p.x(), p.y()};
}
}  // namespace autoware::trajectory::detail
