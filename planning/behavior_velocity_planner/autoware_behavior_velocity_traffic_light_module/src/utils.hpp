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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace autoware::behavior_velocity_planner
{

bool getBackwardPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point);

std::optional<Point2d> findNearestCollisionPoint(
  const LineString2d & line1, const LineString2d & line2, const Point2d & origin);

bool createTargetPoint(
  const tier4_planning_msgs::msg::PathWithLaneId & input, const LineString2d & stop_line,
  const double offset, size_t & target_point_idx, Eigen::Vector2d & target_point);

bool calcStopPointAndInsertIndex(
  const tier4_planning_msgs::msg::PathWithLaneId & input_path,
  const lanelet::ConstLineString3d & lanelet_stop_lines, const double & offset,
  const double & stop_line_extend_length, Eigen::Vector2d & stop_line_point,
  size_t & stop_line_point_idx);

}  // namespace autoware::behavior_velocity_planner

#endif  // UTILS_HPP_
