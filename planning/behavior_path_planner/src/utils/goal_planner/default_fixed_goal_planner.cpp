// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/utils/goal_planner/default_fixed_goal_planner.hpp"

#include "behavior_path_planner/utils/goal_planner/util.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "lanelet2_core/LaneletMap.h"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <memory>

namespace behavior_path_planner
{
using Point2d = tier4_autoware_utils::Point2d;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
BehaviorModuleOutput DefaultFixedGoalPlanner::plan(
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  BehaviorModuleOutput output =
    // use planner previous module reference path
    getPreviousModuleOutput();
  const PathWithLaneId smoothed_path =
    modifyPathForSmoothGoalConnection(*(output.path), planner_data);
  output.path = std::make_shared<PathWithLaneId>(smoothed_path);
  output.reference_path = getPreviousModuleOutput().reference_path;
  return output;
}

lanelet::ConstLanelets DefaultFixedGoalPlanner::extractLaneletsFromPath(
  const PathWithLaneId & refined_path,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto & rh = planner_data->route_handler;
  lanelet::ConstLanelets refined_path_lanelets;
  for (size_t i = 0; i < refined_path.points.size(); ++i) {
    const PathPointWithLaneId & path_point = refined_path.points[i];
    int64_t lane_id = path_point.lane_ids[0];
    lanelet::ConstLanelet lanelet = rh->getLaneletsFromId(lane_id);
    bool is_unique = true;
    for (const lanelet::ConstLanelet & existing_lanelet : refined_path_lanelets) {
      if (lanelet == existing_lanelet) {
        is_unique = false;
        break;
      }
    }
    if (is_unique) {
      refined_path_lanelets.push_back(lanelet);
    }
  }
  return refined_path_lanelets;
}

bool DefaultFixedGoalPlanner::isPathValid(
  const PathWithLaneId & refined_path,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const lanelet::ConstLanelets lanelets = extractLaneletsFromPath(refined_path, planner_data);
  for (size_t i = 0; i < refined_path.points.size(); ++i) {
    const PathPointWithLaneId & path_point = refined_path.points[i];
    if (!utils::isInLanelets(path_point.point.pose, lanelets)) {
      return false;  // at least one path_point falls outside any lanelet
    }
  }
  return true;
}

PathWithLaneId DefaultFixedGoalPlanner::modifyPathForSmoothGoalConnection(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto goal = planner_data->route_handler->getGoalPose();
  const auto goal_lane_id = planner_data->route_handler->getGoalLaneId();
  Pose refined_goal{};
  {
    lanelet::ConstLanelet goal_lanelet;
    if (planner_data->route_handler->getGoalLanelet(&goal_lanelet)) {
      refined_goal = utils::refineGoal(goal, goal_lanelet);
    } else {
      refined_goal = goal;
    }
  }
  double goal_search_radius{planner_data->parameters.refine_goal_search_radius_range};
  const double range_reduce_by{
    1};  // set a reasonable value, 10% - 20% of the refine_goal_search_radius_range is recommended
  int path_is_valid{0};
  autoware_auto_planning_msgs::msg::PathWithLaneId refined_path;
  while (goal_search_radius >= 0 && !path_is_valid) {
    refined_path =
      utils::refinePathForGoal(goal_search_radius, M_PI * 0.5, path, refined_goal, goal_lane_id);
    if (isPathValid(refined_path, planner_data)) {
      path_is_valid = 1;
    }
    goal_search_radius -= range_reduce_by;
  }
  return refined_path;
}

}  // namespace behavior_path_planner
