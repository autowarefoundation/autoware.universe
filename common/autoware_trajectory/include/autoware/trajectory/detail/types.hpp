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

#ifndef AUTOWARE__TRAJECTORY__DETAIL__TYPES_HPP_
#define AUTOWARE__TRAJECTORY__DETAIL__TYPES_HPP_

#include "lanelet2_core/primitives/Point.h"

#include <Eigen/Core>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

namespace autoware::trajectory::detail
{

struct MutablePoint2d
{
  MutablePoint2d(double & x, double & y) : x(x), y(y) {}
  double & x;
  double & y;
};
struct MutablePoint3d : MutablePoint2d
{
  MutablePoint3d(double & x, double & y, double & z) : MutablePoint2d{x, y}, z(z) {}
  double & z;
};

struct ImmutablePoint2d
{
  ImmutablePoint2d(const double & x, const double & y) : x(x), y(y) {}
  const double x;
  const double y;
};

struct ImmutablePoint3d : ImmutablePoint2d
{
  ImmutablePoint3d(const double & x, const double & y, const double & z)
  : ImmutablePoint2d{x, y}, z(z)
  {
  }
  const double z;
};

/**
 * @brief Convert various point types to geometry_msgs::msg::Point.
 * @param p The input point to be converted.
 * @return geometry_msgs::msg::Point The converted point.
 */
MutablePoint3d to_point(geometry_msgs::msg::Point & p);
MutablePoint3d to_point(geometry_msgs::msg::Pose & p);
MutablePoint3d to_point(autoware_planning_msgs::msg::PathPoint & p);
MutablePoint3d to_point(tier4_planning_msgs::msg::PathPointWithLaneId & p);
MutablePoint2d to_point(lanelet::BasicPoint2d & p);

ImmutablePoint3d to_point(const geometry_msgs::msg::Point & p);
ImmutablePoint3d to_point(const geometry_msgs::msg::Pose & p);
ImmutablePoint3d to_point(const autoware_planning_msgs::msg::PathPoint & p);
ImmutablePoint3d to_point(const tier4_planning_msgs::msg::PathPointWithLaneId & p);
ImmutablePoint2d to_point(const lanelet::BasicPoint2d & p);

}  // namespace autoware::trajectory::detail

#endif  // AUTOWARE__TRAJECTORY__DETAIL__TYPES_HPP_
