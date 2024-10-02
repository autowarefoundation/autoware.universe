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

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <utility>

namespace autoware::behavior_velocity_planner::no_stopping_area
{
using PathIndexWithPose = std::pair<size_t, geometry_msgs::msg::Pose>;    // front index, pose
using PathIndexWithPoint2d = std::pair<size_t, universe_utils::Point2d>;  // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;                    // front index, offset

bool isTargetStuckVehicleType(const autoware_perception_msgs::msg::PredictedObject & object);

void insertStopPoint(
  tier4_planning_msgs::msg::PathWithLaneId & path, const PathIndexWithPose & stop_point);
}  // namespace autoware::behavior_velocity_planner::no_stopping_area

#endif  // UTILS_HPP_
