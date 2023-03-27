// Copyright 2023 Tier IV, Inc.
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

#ifndef SCENE_MODULE__OUT_OF_LANE__OUT_OF_LANE_UTILS_HPP_
#define SCENE_MODULE__OUT_OF_LANE__OUT_OF_LANE_UTILS_HPP_

#include "scene_module/out_of_lane/overlapping_range.hpp"
#include "scene_module/out_of_lane/types.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <route_handler/route_handler.hpp>
#include <utilization/util.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::out_of_lane_utils
{
struct DebugData
{
  std::vector<lanelet::BasicPolygon2d> footprints;
  std::vector<SlowdownToInsert> slowdowns;
  Pose ego_pose;
  OverlapRanges ranges;
  std::vector<std::vector<std::pair<double, double>>> npc_times;
  std::vector<std::pair<double, double>> ego_times;
  lanelet::BasicPolygon2d current_footprint;
  lanelet::ConstLanelets current_overlapped_lanelets;
  lanelet::ConstLanelets path_lanelets;
  lanelet::ConstLanelets ignored_lanelets;
  lanelet::ConstLanelets other_lanelets;
  void reset_data()
  {
    footprints.clear();
    slowdowns.clear();
    ranges.clear();
    npc_times.clear();
    ego_times.clear();
    current_overlapped_lanelets.clear();
  }
};
}  // namespace behavior_velocity_planner::out_of_lane_utils

#endif  // SCENE_MODULE__OUT_OF_LANE__OUT_OF_LANE_UTILS_HPP_
