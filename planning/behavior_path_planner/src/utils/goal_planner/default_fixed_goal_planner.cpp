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
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"

#include <lanelet2_core/geometry/Polygon.h>
#include "lanelet2_core/LaneletMap.h"

using Point2d = tier4_autoware_utils::Point2d;
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

bool isInAnyLane(const lanelet::ConstLanelets & candidate_lanelets, const Point2d & point)
{
  for (const auto & ll : candidate_lanelets) {
    if (boost::geometry::covered_by(point, ll.polygon2d().basicPolygon())) {
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
    path_point_point2D.x() = path_point.point.pose.position.x;
    path_point_point2D.y() = path_point.point.pose.position.y;
    std::cerr << "refined_path.points[" << i << "]:" << std::endl;
    std::cerr << "X:" << path_point_point2D.x() << std::endl;
    std::cerr << "Y:" << path_point_point2D.y() << std::endl;
  }

  for (size_t i = 0; i < refined_path.points.size(); ++i) {
    const PathPointWithLaneId& path_point = refined_path.points[i];
    path_point_point2D.x() = path_point.point.pose.position.x;
    path_point_point2D.y() = path_point.point.pose.position.y;
    bool is_point_in_any_lanelet = isInAnyLane(candidate_lanelets, path_point_point2D);
    if (!is_point_in_any_lanelet) {
      std::cerr << "INVALID POINT: refined_path.points[" << i << "]" << std::endl;
      return false;  // at least one path_point falls outside any lanelet
    }
  }
  return true;
}

lanelet::ConstLanelets extractLaneletsFromPath(const PathWithLaneId& refined_path, const std::shared_ptr<const PlannerData> & planner_data) {
    const auto & rh = planner_data->route_handler;
    lanelet::ConstLanelets refined_path_lanelets;
    for (size_t i = 0; i < refined_path.points.size(); ++i) {
        const PathPointWithLaneId& path_point = refined_path.points[i];
        int64_t lane_id = path_point.lane_ids[0];
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
        for (const auto& ll : refined_path_lanelets) {
             lanelet::Id laneId = ll.id();
             std::cerr << "laneId on the Lanelet List: " << laneId << std::endl;          
        }
    return refined_path_lanelets;
}

bool isPathValid(const PathWithLaneId &refined_path, const std::shared_ptr<const PlannerData> & planner_data) {
  const lanelet::ConstLanelets lanelets = extractLaneletsFromPath(refined_path, planner_data);
  return isAllPointsInAnyLane(refined_path, lanelets);
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
  double goal_search_radius {planner_data->parameters.refine_goal_search_radius_range};
  double range_reduce_by {1}; // set a reasonable value, 10% - 20% of the refine_goal_search_radius_range is recommended
  int path_is_valid {0};
  autoware_auto_planning_msgs::msg::PathWithLaneId refined_path;
  while(goal_search_radius >= 0 && !path_is_valid) {
      refined_path = utils::refinePathForGoal(
      goal_search_radius, M_PI * 0.5, path, refined_goal,
      goal_lane_id);
    if (isPathValid(refined_path, planner_data)) {
      path_is_valid = 1;
      std::cerr << "valid refined_path :)" << std::endl;
    } 
    else {
      std::cerr << "INVALID POINTS ON THE CURRENT refined_path!" << std::endl;
    }
    std::cerr << "number of points on refined_path.points:" << refined_path.points.size() << std::endl; 
    std::cerr << "Goal Search Radius is " << goal_search_radius << " meter(s)" << std::endl;   
    std::cerr << "range_reduce_by is " << range_reduce_by << " meter(s)" << std::endl;   
    goal_search_radius -= range_reduce_by; 
  }
  return refined_path; 
}

}  // namespace behavior_path_planner
