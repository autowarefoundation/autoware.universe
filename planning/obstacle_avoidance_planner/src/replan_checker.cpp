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

#include "obstacle_avoidance_planner/replan_checker.hpp"

#include "motion_utils/motion_utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <vector>

namespace
{
[[maybe_unused]] std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = motion_utils::resampleTrajectory(traj, interval);

  // convert Trajectory to std::vector<TrajectoryPoint>
  std::vector<TrajectoryPoint> resampled_traj_points;
  for (const auto & point : resampled_traj.points) {
    resampled_traj_points.push_back(point);
  }

  return resampled_traj_points;
}
}  // namespace

ReplanChecker::ReplanChecker(
  rclcpp::Node & node, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold)
{
  ego_nearest_dist_threshold_ = ego_nearest_dist_threshold;
  ego_nearest_yaw_threshold_ = ego_nearest_yaw_threshold;

  max_path_shape_change_dist_ = node.declare_parameter<double>("replan.max_path_shape_change_dist");
  max_ego_moving_dist_ = node.declare_parameter<double>("replan.max_ego_moving_dist_for_replan");
  max_delta_time_sec_ = node.declare_parameter<double>("replan.max_delta_time_sec_for_replan");
}

void ReplanChecker::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  updateParam<double>(parameters, "replan.max_path_shape_change_dist", max_path_shape_change_dist_);
  updateParam<double>(parameters, "replan.max_ego_moving_dist", max_ego_moving_dist_);
  updateParam<double>(parameters, "replan.max_delta_time_sec", max_delta_time_sec_);
}

bool ReplanChecker::isReplanRequired(
  const PlannerData & planner_data, const rclcpp::Time & current_time,
  const std::shared_ptr<MPTTrajs> prev_mpt_trajs_ptr)
{
  reset_optimization_ = false;
  const auto & p = planner_data;

  if (
    !prev_ego_pose_ptr_ || !prev_replanned_time_ptr_ || !prev_path_points_ptr_ ||
    !prev_mpt_trajs_ptr) {
    return true;
  }

  if (prev_mpt_trajs_ptr->mpt.empty()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ReplanChecker"),
      "Replan with resetting optimization since previous optimized trajectory is empty.");
    reset_optimization_ = true;
    return true;
  }

  if (isPathShapeChanged(p)) {
    RCLCPP_INFO(
      rclcpp::get_logger("ReplanChecker"),
      "Replan with resetting optimization since path shape was changed.");
    reset_optimization_ = true;
    return true;
  }

  if (isPathGoalChanged(planner_data)) {
    RCLCPP_INFO(
      rclcpp::get_logger("ReplanChecker"),
      "Replan with resetting optimization since path goal was changed.");
    reset_optimization_ = true;
    return true;
  }

  // For when ego pose is lost or new ego pose is designated in simulation
  const double delta_dist =
    tier4_autoware_utils::calcDistance2d(p.ego_pose, prev_ego_pose_ptr_->position);
  if (delta_dist > max_ego_moving_dist_) {
    RCLCPP_INFO(
      rclcpp::get_logger("ReplanChecker"),
      "Replan with resetting optimization since current ego pose is far from previous ego pose.");
    reset_optimization_ = true;
    return true;
  }

  const double delta_time_sec = (current_time - *prev_replanned_time_ptr_).seconds();
  if (delta_time_sec > max_delta_time_sec_) {
    return true;
  }

  return false;
}

bool ReplanChecker::isResetOptimizationRequired() { return reset_optimization_; }

bool ReplanChecker::isPathShapeChanged(const PlannerData & planner_data)
{
  if (!prev_path_points_ptr_) {
    return true;
  }

  const auto & p = planner_data;

  const double max_path_length = 50.0;

  // truncate prev points from ego pose to fixed end points
  const auto prev_begin_idx = findEgoNearestIndex(*prev_path_points_ptr_, p.ego_pose);
  const auto truncated_prev_points =
    points_utils::clipForwardPoints(*prev_path_points_ptr_, prev_begin_idx, max_path_length);

  // truncate points from ego pose to fixed end points
  const auto begin_idx = findEgoNearestIndex(p.path.points, p.ego_pose);
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

bool ReplanChecker::isPathGoalChanged(const PlannerData & planner_data)
{
  const auto & p = planner_data;

  if (prev_path_points_ptr_) {
    return true;
  }

  constexpr double min_vel = 1e-3;
  if (std::abs(p.ego_vel) > min_vel) {
    return false;
  }

  // NOTE: Path may be cropped and does not contain the goal.
  // Therefore we set a large value to distance threshold.
  constexpr double max_goal_moving_dist = 1.0;
  const double goal_moving_dist =
    tier4_autoware_utils::calcDistance2d(p.path.points.back(), prev_path_points_ptr_->back());
  if (goal_moving_dist < max_goal_moving_dist) {
    return false;
  }

  return true;
}
