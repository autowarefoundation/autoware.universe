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

#ifndef COLLISION_FREE_PATH_PLANNER__REPLAN_CHECKER_HPP_
#define COLLISION_FREE_PATH_PLANNER__REPLAN_CHECKER_HPP_

#include "collision_free_path_planner/common_structs.hpp"
#include "collision_free_path_planner/mpt_optimizer.hpp"
#include "collision_free_path_planner/type_rename.hpp"
#include "collision_free_path_planner/utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include <memory>
#include <vector>

namespace collision_free_path_planner
{
class ReplanChecker
{
public:
  explicit ReplanChecker(rclcpp::Node & node, const EgoNearestParam & ego_nearest_param);
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

  bool isResetRequired(const PlannerData & planner_data);

  bool isReplanRequired(
    const PlannerData & planner_data, const rclcpp::Time & current_time,
    const std::shared_ptr<std::vector<TrajectoryPoint>> prev_mpt_traj_ptr);

private:
  // previous variables for isResetRequired
  std::shared_ptr<std::vector<PathPoint>> prev_path_points_ptr_{nullptr};
  std::shared_ptr<geometry_msgs::msg::Pose> prev_ego_pose_ptr_{nullptr};

  // previous variable for isReplanRequired
  std::shared_ptr<rclcpp::Time> prev_replanned_time_ptr_{nullptr};

  EgoNearestParam ego_nearest_param_;

  // bool reset_optimization_{false};

  // algorithm parameters
  double max_path_shape_change_dist_;
  double max_ego_moving_dist_;
  double max_delta_time_sec_;

  bool isPathShapeChanged(
    const PlannerData & planner_data, const std::vector<PathPoint> & prev_path_points) const;
  bool isPathGoalChanged(
    const PlannerData & planner_data, const std::vector<PathPoint> & prev_path_points) const;

  void printInfo(const char * msg) const { RCLCPP_INFO(rclcpp::get_logger("ReplanChecker"), msg); }
};
}  // namespace collision_free_path_planner

#endif  // COLLISION_FREE_PATH_PLANNER__REPLAN_CHECKER_HPP_
