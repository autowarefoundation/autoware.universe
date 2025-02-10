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

#include "autoware/behavior_path_goal_planner_module/pull_over_planner/freespace_pull_over.hpp"

#include "autoware/behavior_path_goal_planner_module/util.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{

using autoware::freespace_planning_algorithms::AstarSearch;
using autoware::freespace_planning_algorithms::RRTStar;

FreespacePullOver::FreespacePullOver(rclcpp::Node & node, const GoalPlannerParameters & parameters)
: PullOverPlannerBase{node, parameters},
  velocity_{parameters.freespace_parking_velocity},
  left_side_parking_{parameters.parking_policy == ParkingPolicy::LEFT_SIDE},
  use_back_{
    parameters.freespace_parking_algorithm == "astar"
      ? parameters.astar_parameters.use_back
      : true  // no option for disabling back in rrtstar
  }
{
  autoware::freespace_planning_algorithms::VehicleShape vehicle_shape(
    vehicle_info_, parameters.vehicle_shape_margin);
  if (parameters.freespace_parking_algorithm == "astar") {
    planner_ = std::make_unique<AstarSearch>(
      parameters.freespace_parking_common_parameters, vehicle_shape, parameters.astar_parameters,
      node.get_clock());
  } else if (parameters.freespace_parking_algorithm == "rrtstar") {
    planner_ = std::make_unique<RRTStar>(
      parameters.freespace_parking_common_parameters, vehicle_shape, parameters.rrt_star_parameters,
      node.get_clock());
  }
}

std::optional<PullOverPath> FreespacePullOver::plan(
  const GoalCandidate & modified_goal_pose, const size_t id,
  const std::shared_ptr<const PlannerData> planner_data,
  [[maybe_unused]] const BehaviorModuleOutput & upstream_module_output)
{
  const Pose & current_pose = planner_data->self_odometry->pose.pose;

  // offset goal pose to make straight path near goal for improving parking precision
  // todo: support straight path when using back
  constexpr double straight_distance = 1.0;
  const auto & goal_pose = modified_goal_pose.goal_pose;
  const Pose end_pose =
    use_back_ ? goal_pose
              : autoware::universe_utils::calcOffsetPose(goal_pose, -straight_distance, 0.0, 0.0);

  try {
    if (!planner_->makePlan(current_pose, end_pose)) {
      return {};
    }
  } catch (const std::exception & e) {
    return {};
  }

  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, parameters_.backward_goal_search_length, parameters_.forward_goal_search_length,
    /*forward_only_in_route*/ false);
  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *(planner_data->route_handler), left_side_parking_, parameters_.backward_goal_search_length,
    parameters_.forward_goal_search_length);
  if (road_lanes.empty() || pull_over_lanes.empty()) {
    return {};
  }
  const auto lanes = utils::combineLanelets(road_lanes, pull_over_lanes);

  PathWithLaneId path =
    utils::convertWayPointsToPathWithLaneId(planner_->getWaypoints(), velocity_, lanes);
  const auto reverse_indices = utils::getReversingIndices(path);
  std::vector<PathWithLaneId> partial_paths = utils::dividePath(path, reverse_indices);

  // remove points which are near the goal
  PathWithLaneId & last_path = partial_paths.back();
  const double th_goal_distance = 1.0;
  for (auto it = last_path.points.begin(); it != last_path.points.end(); ++it) {
    size_t index = std::distance(last_path.points.begin(), it);
    if (index == 0) continue;
    const double distance =
      autoware::universe_utils::calcDistance2d(end_pose.position, it->point.pose.position);
    if (distance < th_goal_distance) {
      last_path.points.erase(it, last_path.points.end());
      break;
    }
  }

  // add PathPointWithLaneId to last path
  auto addPose = [&last_path](const Pose & pose) {
    PathPointWithLaneId p = last_path.points.back();
    p.point.pose = pose;
    last_path.points.push_back(p);
  };

  if (use_back_) {
    addPose(end_pose);
  } else {
    // add interpolated poses
    auto addInterpolatedPoses = [&addPose](const Pose & pose1, const Pose & pose2) {
      constexpr double interval = 0.5;
      std::vector<Pose> interpolated_poses = utils::interpolatePose(pose1, pose2, interval);
      for (const auto & pose : interpolated_poses) {
        addPose(pose);
      }
    };
    addInterpolatedPoses(last_path.points.back().point.pose, end_pose);
    addPose(end_pose);
    addInterpolatedPoses(end_pose, goal_pose);
    addPose(goal_pose);
  }

  std::vector<std::pair<double, double>> pairs_terminal_velocity_and_accel{};
  pairs_terminal_velocity_and_accel.resize(partial_paths.size());
  utils::parking_departure::modifyVelocityByDirection(
    partial_paths, pairs_terminal_velocity_and_accel, velocity_, 0);

  // Check if driving forward for each path, return empty if not
  for (auto & partial_path : partial_paths) {
    if (!autoware::motion_utils::isDrivingForward(partial_path.points)) {
      return {};
    }
  }

  auto pull_over_path_opt = PullOverPath::create(
    getPlannerType(), id, partial_paths, current_pose, modified_goal_pose,
    pairs_terminal_velocity_and_accel);
  if (!pull_over_path_opt) {
    return {};
  }
  return pull_over_path_opt.value();
}
}  // namespace autoware::behavior_path_planner
