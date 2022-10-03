// Copyright 2022 Tier IV, Inc.
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

#include "collision_free_path_planner/replan_checker.hpp"

#include "motion_utils/motion_utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <vector>

namespace collision_free_path_planner
{
ReplanChecker::ReplanChecker(rclcpp::Node * node, const EgoNearestParam & ego_nearest_param)
: ego_nearest_param_(ego_nearest_param)
{
  max_path_shape_change_dist_ =
    node->declare_parameter<double>("replan.max_path_shape_change_dist");
  max_ego_moving_dist_ = node->declare_parameter<double>("replan.max_ego_moving_dist");
  max_delta_time_sec_ = node->declare_parameter<double>("replan.max_delta_time_sec");
}

void ReplanChecker::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  updateParam<double>(parameters, "replan.max_path_shape_change_dist", max_path_shape_change_dist_);
  updateParam<double>(parameters, "replan.max_ego_moving_dist", max_ego_moving_dist_);
  updateParam<double>(parameters, "replan.max_delta_time_sec", max_delta_time_sec_);
}

bool ReplanChecker::isResetRequired(const PlannerData & planner_data)
{
  const auto & p = planner_data;

  const bool reset_required = [&]() {
    // guard for invalid variables
    if (!prev_path_points_ptr_ || !prev_ego_pose_ptr_) {
      return true;
    }
    const auto & prev_path_points = *prev_path_points_ptr_;

    // path shape changes
    if (isPathShapeChanged(planner_data, prev_path_points)) {
      printInfo("Replan with resetting optimization since path shape was changed.");
      return true;
    }

    // path goal changes
    if (isPathGoalChanged(planner_data, prev_path_points)) {
      printInfo("Replan with resetting optimization since path goal was changed.");
      return true;
    }

    // ego pose is lost or new ego pose is designated in simulation
    const double delta_dist =
      tier4_autoware_utils::calcDistance2d(p.ego_pose, prev_ego_pose_ptr_->position);
    if (max_ego_moving_dist_ < delta_dist) {
      printInfo(
        "Replan with resetting optimization since current ego pose is far from previous ego pose.");
      return true;
    }

    return false;
  }();

  // update previous information required in this function
  prev_path_points_ptr_ = std::make_shared<std::vector<PathPoint>>(p.path.points);
  prev_ego_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(p.ego_pose);

  return reset_required;
}

bool ReplanChecker::isReplanRequired(
  const PlannerData & planner_data, const rclcpp::Time & current_time,
  const std::shared_ptr<std::vector<TrajectoryPoint>> prev_mpt_traj_ptr)
{
  const auto & p = planner_data;

  const bool replan_required = [&]() {
    // guard for invalid variables
    if (!prev_replanned_time_ptr_ || !prev_path_points_ptr_ /*|| !prev_mpt_traj_ptr*/) {
      return true;
    }

    /*
    // empty mpt points
    if (prev_mpt_traj_ptr->empty()) {
      printInfo("Replan with resetting optimization since previous optimized trajectory is empty.");
      return true;
    }
    */

    // time elapses
    const double delta_time_sec = (current_time - *prev_replanned_time_ptr_).seconds();
    if (max_delta_time_sec_ < delta_time_sec) {
      return true;
    }

    return false;
  }();

  // update previous information required in this function
  if (replan_required) {
    prev_replanned_time_ptr_ = std::make_shared<rclcpp::Time>(current_time);
  }

  return replan_required;
}

bool ReplanChecker::isPathShapeChanged(
  const PlannerData & planner_data, const std::vector<PathPoint> & prev_path_points) const
{
  const auto & p = planner_data;

  const double max_path_length = 50.0;

  // truncate prev points from ego pose to fixed end points
  const auto prev_begin_idx =
    points_utils::findEgoIndex(prev_path_points, p.ego_pose, ego_nearest_param_);
  const auto truncated_prev_points =
    points_utils::clipForwardPoints(prev_path_points, prev_begin_idx, max_path_length);

  // truncate points from ego pose to fixed end points
  const auto begin_idx = points_utils::findEgoIndex(p.path.points, p.ego_pose, ego_nearest_param_);
  const auto truncated_points =
    points_utils::clipForwardPoints(p.path.points, begin_idx, max_path_length);

  // guard for lateral offset
  if (truncated_prev_points.size() < 2 || truncated_points.size() < 2) {
    return false;
  }

  // calculate lateral deviations between truncated path_points and prev_path_points
  for (const auto & prev_point : truncated_prev_points) {
    const double dist =
      std::abs(motion_utils::calcLateralOffset(truncated_points, prev_point.pose.position));
    if (dist > max_path_shape_change_dist_) {
      return true;
    }
  }

  return false;
}

bool ReplanChecker::isPathGoalChanged(
  const PlannerData & planner_data, const std::vector<PathPoint> & prev_path_points) const
{
  const auto & p = planner_data;

  constexpr double min_vel = 1e-3;
  if (std::abs(p.ego_vel) > min_vel) {
    return false;
  }

  // NOTE: Path may be cropped and does not contain the goal.
  // Therefore we set a large value to distance threshold.
  constexpr double max_goal_moving_dist = 1.0;
  const double goal_moving_dist =
    tier4_autoware_utils::calcDistance2d(p.path.points.back(), prev_path_points.back());
  if (goal_moving_dist < max_goal_moving_dist) {
    return false;
  }

  return true;
}
}  // namespace collision_free_path_planner
