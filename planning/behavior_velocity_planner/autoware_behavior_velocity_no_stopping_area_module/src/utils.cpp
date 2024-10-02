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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>

namespace autoware::behavior_velocity_planner::no_stopping_area
{

bool isTargetStuckVehicleType(const autoware_perception_msgs::msg::PredictedObject & object)
{
  return object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::CAR ||
         object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::BUS ||
         object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::TRUCK ||
         object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::TRAILER ||
         object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
}

void insertStopPoint(
  tier4_planning_msgs::msg::PathWithLaneId & path, const PathIndexWithPose & stop_point)
{
  auto insert_idx = static_cast<size_t>(stop_point.first + 1);
  const auto stop_pose = stop_point.second;

  // To PathPointWithLaneId
  tier4_planning_msgs::msg::PathPointWithLaneId stop_point_with_lane_id;
  stop_point_with_lane_id = path.points.at(insert_idx);
  stop_point_with_lane_id.point.pose = stop_pose;
  stop_point_with_lane_id.point.longitudinal_velocity_mps = 0.0;

  // Insert stop point or replace with zero velocity
  planning_utils::insertVelocity(path, stop_point_with_lane_id, 0.0, insert_idx);
}
}  // namespace autoware::behavior_velocity_planner::no_stopping_area
