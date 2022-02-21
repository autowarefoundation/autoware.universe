/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <bezier_sampler/bezier.hpp>
#include <eigen3/Eigen/Core>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace motion_planning::bezier_sampler
{
//@brief convert the given Bezier curve into a vector of autoware_auto_planning_msgs::msg::PathPoint
// with the given number of equaly spaced points
inline std::vector<autoware_auto_planning_msgs::msg::PathPoint> toPathPoints(
  const Bezier & b, int nb_points)
{
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> points;
  points.reserve(nb_points);
  for (const Eigen::Vector3d & p : b.cartesianWithHeading(nb_points)) {
    autoware_auto_planning_msgs::msg::PathPoint point;
    point.pose.position.x = p.x();
    point.pose.position.y = p.y();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p.z());
    point.pose.orientation.w = q.w();
    point.pose.orientation.x = q.x();
    point.pose.orientation.y = q.y();
    point.pose.orientation.z = q.z();
    points.push_back(point);
  }
  return points;
}
//@brief convert the given sequence of Bezier curves into a vector of
// autoware_auto_planning_msgs::msg::PathPoints with the given number of equaly spaced points
inline std::vector<autoware_auto_planning_msgs::msg::PathPoint> toPathPoints(
  const std::vector<Bezier> & bs, int nb_points)
{
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> points;
  points.reserve(bs.size() * nb_points);
  for (const Bezier & b : bs) {
    std::vector<autoware_auto_planning_msgs::msg::PathPoint> b_points = toPathPoints(b, nb_points);
    points.insert(points.end(), b_points.begin(), b_points.end());
  }
  return points;
}
//@brief convert the given Bezier curve into a vector of autoware_auto_planning_msgs::msg::PathPoint
// with the given distance between points
inline std::vector<autoware_auto_planning_msgs::msg::PathPoint> toPathPoints(
  const Bezier & b, double resolution)
{
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> points;
  for (const Eigen::Vector2d & p : b.cartesian(resolution)) {
    autoware_auto_planning_msgs::msg::PathPoint point;
    point.pose.position.x = p.x();
    point.pose.position.y = p.y();
    points.push_back(point);
  }
  return points;
}
}  // namespace motion_planning::bezier_sampler
