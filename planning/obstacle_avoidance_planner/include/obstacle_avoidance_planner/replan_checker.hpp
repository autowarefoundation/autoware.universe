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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__REPLAN_CHECKER_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__REPLAN_CHECKER_HPP_

#include "obstacle_avoidance_planner/common_structs.hpp"
#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "obstacle_avoidance_planner/utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include <memory>
#include <vector>

using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

class ReplanChecker
{
public:
  explicit ReplanChecker(
    rclcpp::Node & node, const double ego_nearest_dist_threshold,
    const double ego_nearest_yaw_threshold);
  void onParam(const std::vector<rclcpp::Parameter> & parameters);
  bool isReplanRequired(
    const PlannerData & planner_data, const rclcpp::Time & current_time,
    const std::shared_ptr<MPTTrajs> prev_mpt_trajs_ptr);
  bool isResetOptimizationRequired();

private:
  bool check(const PlannerData & planner_data, const rclcpp::Time & current_time);
  bool isPathShapeChanged(const PlannerData & planner_data);
  bool isPathGoalChanged(const PlannerData & planner_data);

  template <class T>
  size_t findEgoNearestIndex(
    const std::vector<T> & points, const geometry_msgs::msg::Pose & ego_pose)
  {
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      points, ego_pose, ego_nearest_dist_threshold_, ego_nearest_yaw_threshold_);
  }

  // previous variables
  std::shared_ptr<std::vector<PathPoint>> prev_path_points_ptr_{nullptr};
  std::shared_ptr<rclcpp::Time> prev_replanned_time_ptr_{nullptr};
  std::shared_ptr<geometry_msgs::msg::Pose> prev_ego_pose_ptr_{nullptr};

  bool reset_optimization_{false};

  // algorithm parameters
  double max_path_shape_change_dist_;
  double max_ego_moving_dist_;
  double max_delta_time_sec_;

  double ego_nearest_dist_threshold_;
  double ego_nearest_yaw_threshold_;
};

#endif  // OBSTACLE_AVOIDANCE_PLANNER__REPLAN_CHECKER_HPP_
