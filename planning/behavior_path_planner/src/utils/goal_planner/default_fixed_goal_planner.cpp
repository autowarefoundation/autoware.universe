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

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>

#include <boost/geometry.hpp>
namespace behavior_path_planner
{

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




// use the isAllPointsInAnyLane (new test)
// copied isInAnyLane()

#include "lanelet2_core/LaneletMap.h"

bool isInAnyLane(const lanelet::ConstLanelets & candidate_lanelets, const Point2d & point)
{
  for (const auto & ll : candidate_lanelets) {
    if (boost::geometry::within(point, ll.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}

bool isAllPointsInAnyLane(const PathPointWithLaneId &refined_path,
                          const lanelet::ConstLanelets &candidate_lanelets) {
  for (const auto &path_point : refined_path) {
    bool is_point_in_any_lanelet = isInAnyLane(candidate_lanelets, path_point.point.pose.position);
    if (!is_point_in_any_lanelet) {
      return false;  // at least one path_point falls outside any lanelet
    }
  }
  return true;
}

// end of the isAllPointsInAnyLane



// retreive the extractLaneletsFromPath (new test)

#include "route_handler/route_handler.hpp" // MAKE SURE

lanelet::ConstLanelets extractLaneletsFromPath(const PathPointWithLaneId& refined_path) {
    const auto & rh = planner_data->route_handler; // HOW
    lanelet::ConstLanelets refined_path_lanelets;
    for (const auto& path_point : refined_path) {
        int64_t lane_id = path_point.lane_ids;
        lanelet::ConstLanelet lanelet = rh->getLaneletsFromId(lane_id);
        bool is_unique = true;
        for (const lanelet::ConstLanelet& existing_lanelet : refined_path_lanelets) {
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

// end of the extractLaneletsFromPath (new test)


// isPathValid (test)
// the utilization of the extractLaneletsFromPath: isAllPointsInAnyLane(refined_path,extractLaneletsFromPath(refined_path))

bool isPathValid(const PathWithLaneId &refined_path) {
  const auto lanelets = extractLaneletsFromPath(refined_path);
  return isAllPointsInAnyLane(refined_path, lanelets);
}

// end of isPathValid (test)



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

  auto refined_path = utils::refinePathForGoal(
    planner_data->parameters.refine_goal_search_radius_range, M_PI * 0.5, path, refined_goal,
    goal_lane_id);

  return refined_path;
}

}  // namespace behavior_path_planner
