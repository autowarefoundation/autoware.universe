// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__CONTROL_EVALUATOR__METRICS__METRICS_UTILS_HPP_
#define AUTOWARE__CONTROL_EVALUATOR__METRICS__METRICS_UTILS_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

namespace control_diagnostics
{
namespace metrics
{
namespace utils
{
using autoware::route_handler::RouteHandler;
using autoware::universe_utils::Point2d;
using geometry_msgs::msg::Pose;

/**
 * @brief Get the closest lanelets to the ego vehicle, considering shoulder lanelets.
 * @param [in] route_handler route handler
 * @param [in] ego_pose ego vehicle pose
 * @return closest lanelets to the ego vehicle
 **/
lanelet::ConstLanelets get_current_lanes(const RouteHandler & route_handler, const Pose & ego_pose);

/**
 * @brief Get the most side boundary of the lanelet.
 * @param [in] route_handler route handler
 * @param [in] lanelet lanelet to get the most side boundary
 * @param [in] left_side true to get left side boundary, false to get right side boundary
 * @param [in] enable_same_root true if the same root lanelets are considered, false otherwise
 * @param [in] get_shoulder_lane true if shoulder lanelets are considered, false otherwise
 * @return most side boundary of the lanelet
 **/
lanelet::ConstLineString3d get_most_side_boundary(
  const RouteHandler & route_handler, const lanelet::ConstLanelet & lanelet, const bool left_side,
  const bool enable_same_root = false, const bool get_shoulder_lane = true);

/**
 * @brief Calculate the Euler distance between the vehicle and the lanelet.
 * @param [in] vehicle_footprint vehicle footprint
 * @param [in] line lanelet line
 * @return distance between the vehicle footprint and the lanelet, 0.0 if the vehicle intersects
 *with the line
 **/
double calc_distance_to_line(
  const autoware::universe_utils::LinearRing2d & vehicle_footprint,
  const lanelet::ConstLineString3d & line);

/**
 * @brief Calculate the yaw deviation between the ego's orientation and the vector from the ego
 * position to the closest point on the line.
 * @param [in] ego_pose The pose of the ego vehicle.
 * @param [in] line The line to which the yaw deviation is calculated.
 * @return The yaw deviation in radians, normalized to the range [-π, π].
 */
double calc_yaw_to_line(const Pose & ego_pose, const lanelet::ConstLineString3d & line);

}  // namespace utils
}  // namespace metrics
}  // namespace control_diagnostics
#endif  // AUTOWARE__CONTROL_EVALUATOR__METRICS__METRICS_UTILS_HPP_
