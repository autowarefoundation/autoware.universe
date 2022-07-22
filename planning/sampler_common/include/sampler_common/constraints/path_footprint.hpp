/*
 * Copyright 2022 Tier IV, Inc. All rights reserved.
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

#ifndef SAMPLER_COMMON__CONSTRAINTS__PATH_FOOTPRINT_HPP_
#define SAMPLER_COMMON__CONSTRAINTS__PATH_FOOTPRINT_HPP_

#include "sampler_common/structures.hpp"

#include <eigen3/Eigen/Core>

#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace sampler_common::constraints
{

Polygon buildFootprintPolygon(const Path & path, const Constraints & constraints)
{
  const auto & offsets = constraints.vehicle_offsets;
  // Using the method from Section IV.A of A. Artuñedoet al.: Real-Time Motion Planning Approach for
  // Automated Driving in Urban Environments
  Polygon footprint;
  // TODO(Maxime CLEMENT): special case when = 1
  if (path.points.size() < 2) return footprint;
  // sample points
  // we store the right bound as it needs to be added to the polygon after the left bound
  std::vector<Eigen::Vector2d> right_bound;
  std::vector<Eigen::Vector3d> points;
  points.reserve(path.points.size());
  for (size_t i = 0; i < std::min(path.points.size(), path.yaws.size()); ++i) {
    points.emplace_back(path.points[i].x(), path.points[i].y(), path.yaws[i]);
  }
  // first point: use the left and right point on the rear
  {
    const Eigen::Vector2d first_point = points.front().head<2>();
    double heading = points.front().z();
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading), std::sin(heading), std::cos(heading);
    const Eigen::Vector2d left_rear_point = first_point + rotation * offsets.left_rear;
    const Eigen::Vector2d right_rear_point = first_point + rotation * offsets.right_rear;
    boost::geometry::append(footprint, Point(left_rear_point.x(), left_rear_point.y()));
    right_bound.push_back(right_rear_point);
  }
  // For each points (except 1st and last)
  for (auto it = std::next(points.begin()); it != std::prev(points.end()); ++it) {
    const Eigen::Vector2d point = it->head<2>();
    const double prev_heading = std::prev(it)->z();
    const double heading = it->z();
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading), std::sin(heading), std::cos(heading);
    // We use the change in the heading (restricted in [-pi;pi]) to determine if the path is turning
    // left or right
    const bool turning_right = (std::fmod(heading - prev_heading + M_PI, 2 * M_PI) - M_PI) < 0;
    if (turning_right) {
      const Eigen::Vector2d left_front_point = point + rotation * offsets.left_front;
      boost::geometry::append(footprint, Point(left_front_point.x(), left_front_point.y()));
      right_bound.push_back(point + rotation * offsets.right_rear);
    } else {
      const Eigen::Vector2d left_rear_point = point + rotation * offsets.left_rear;
      boost::geometry::append(footprint, Point(left_rear_point.x(), left_rear_point.y()));
      right_bound.push_back(point + rotation * offsets.right_front);
    }
  }
  // last point: use the left and right point on the front
  {
    Eigen::Vector2d last_point = points.back().head<2>();
    double heading = points.back().z();
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading), std::sin(heading), std::cos(heading);
    Eigen::Vector2d left_front_point = last_point + rotation * offsets.left_front;
    boost::geometry::append(footprint, Point(left_front_point.x(), left_front_point.y()));
    right_bound.push_back(last_point + rotation * offsets.right_front);
  }
  for (auto it = right_bound.rbegin(); it != right_bound.rend(); ++it)
    boost::geometry::append(footprint, Point(it->x(), it->y()));
  boost::geometry::correct(footprint);
  return footprint;
}
}  // namespace sampler_common::constraints

#endif  // SAMPLER_COMMON__CONSTRAINTS__PATH_FOOTPRINT_HPP_
