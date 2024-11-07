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

#ifndef AUTOWARE__PATH_GENERATOR__UTILS_HPP_
#define AUTOWARE__PATH_GENERATOR__UTILS_HPP_

#include "autoware/path_generator/planner_data.hpp"

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <optional>

namespace autoware::path_generator
{
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

namespace utils
{
std::optional<PathWithLaneId> generateCenterLinePath(const PlannerData & planner_data);

std::optional<PathWithLaneId> getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const PlannerData & planner_data);

std::optional<PathWithLaneId> getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  const PlannerData & planner_data);

std::optional<lanelet::ConstLanelets> getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

std::optional<lanelet::ConstLanelets> getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

std::optional<lanelet::ConstLanelets> getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

std::optional<lanelet::ConstLanelet> getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

std::optional<lanelet::ConstLanelets> getNextLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

std::optional<lanelet::ConstLanelets> getPreviousLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> getWaypointGroups(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::LaneletMap & lanelet_map);

void removeOverlappingPoints(PathWithLaneId & path);
}  // namespace utils
}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__UTILS_HPP_
