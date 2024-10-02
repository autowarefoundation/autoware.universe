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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <optional>
#include <utility>

namespace autoware::behavior_velocity_planner::no_stopping_area
{
using PathIndexWithPose = std::pair<size_t, geometry_msgs::msg::Pose>;    // front index, pose
using PathIndexWithPoint2d = std::pair<size_t, universe_utils::Point2d>;  // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;                    // front index, offset

/**
 * @brief check if the object has a target type for stuck check
 * @param object target object
 * @return true if the object has a target type
 */
bool is_target_stuck_vehicle_type(const autoware_perception_msgs::msg::PredictedObject & object);

/**
 * @brief insert stop point on ego path
 * @param path          original path
 * @param stop_point    stop line point on the lane
 */
void insert_stop_point(
  tier4_planning_msgs::msg::PathWithLaneId & path, const PathIndexWithPose & stop_point);

/**
 * @brief auto gen no stopping area stop line from area polygon if stop line is not set
 *        ---------------
 * ------col-------------|--> ego path
 *        |     Area     |
 *        ---------------
 **/
std::optional<universe_utils::LineString2d> generate_stop_line(
  const tier4_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstPolygons3d & no_stopping_areas, const double ego_width,
  const double stop_line_margin);

}  // namespace autoware::behavior_velocity_planner::no_stopping_area

#endif  // UTILS_HPP_
