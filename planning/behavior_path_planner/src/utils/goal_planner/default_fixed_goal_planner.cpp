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


#include <tier4_autoware_utils/geometry/boost_geometry.hpp>
using Point2d = tier4_autoware_utils::Point2d;

#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
using autoware_auto_planning_msgs::msg::PathWithLaneId;



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

bool isAllPointsInAnyLane(const PathWithLaneId &refined_path,
                          const lanelet::ConstLanelets &candidate_lanelets) {
  Point2d path_point_point2D;
  for (size_t i = 0; i < refined_path.points.size(); ++i) {
    const PathPointWithLaneId& path_point = refined_path.points[i];
    path_point_point2D.x = path_point.point.pose.position.x;
    path_point_point2D.y = path_point.point.pose.position.y;
    bool is_point_in_any_lanelet = isInAnyLane(candidate_lanelets, path_point_point2D);
    if (!is_point_in_any_lanelet) {
      return false;  // at least one path_point falls outside any lanelet
    }
  }
  return true;
}

// end of the isAllPointsInAnyLane



// retreive the extractLaneletsFromPath (new test)

#include "route_handler/route_handler.hpp" // MAKE SURE

lanelet::ConstLanelets extractLaneletsFromPath(const PathWithLaneId& refined_path, const std::shared_ptr<const PlannerData> & planner_data) {
    const auto & rh = planner_data->route_handler;
    lanelet::ConstLanelets refined_path_lanelets;

    for (size_t i = 0; i < refined_path.points.size(); ++i) {
        const PathPointWithLaneId& path_point = refined_path.points[i];
        int64_t lane_id = path_point.lane_ids[0];
        lanelet::ConstLanelet lanelet = rh->getLaneletsFromId(lane_id);
        bool is_unique = true;
        for (const lanelet::ConstLanelet& existing_lanelet : refined_path_lanelets) {
            if (lanelet == existing_lanelet) { //not sure whether can overload.
                is_unique = false;
                break;
            }
        }
        if (is_unique) {
            refined_path_lanelets.push_back(lanelet); //not sure whether can push_back.
        }
    }
    return refined_path_lanelets;
}

// end of the extractLaneletsFromPath (new test)


// isPathValid (test)
// the utilization of the extractLaneletsFromPath: isAllPointsInAnyLane(refined_path,extractLaneletsFromPath(refined_path))

bool isPathValid(const PathWithLaneId &refined_path, const std::shared_ptr<const PlannerData> & planner_data) {
  const lanelet::ConstLanelets lanelets = extractLaneletsFromPath(refined_path, planner_data);
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
    if (!isPathValid(refined_path, planner_data))
    {
      std::cerr << "The current planned path is NOT valid" << std::endl;
    }    
  return refined_path;
}

}  // namespace behavior_path_planner
