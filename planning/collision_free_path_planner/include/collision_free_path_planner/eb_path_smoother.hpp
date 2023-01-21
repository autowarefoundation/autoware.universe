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

#ifndef COLLISION_FREE_PATH_PLANNER__EB_PATH_SMOOTHER_HPP_
#define COLLISION_FREE_PATH_PLANNER__EB_PATH_SMOOTHER_HPP_

#include "collision_free_path_planner/common_structs.hpp"
#include "collision_free_path_planner/type_alias.hpp"
#include "eigen3/Eigen/Core"
#include "osqp_interface/osqp_interface.hpp"

#include "boost/optional.hpp"

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace collision_free_path_planner
{
class EBPathSmoother
{
public:
  struct ConstrainLines
  {
    struct Constrain
    {
      Eigen::Vector2d coef;
      double upper_bound;
      double lower_bound;
    };

    Constrain lon;
    Constrain lat;
  };

  EBPathSmoother(
    rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
    const TrajectoryParam & traj_param, const std::shared_ptr<DebugData> debug_data_ptr);

  boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>> getEBTrajectory(
    const PlannerData & planner_data,
    const std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj);

  void reset(const bool enable_debug_info, const TrajectoryParam & traj_param);
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

private:
  bool enable_debug_info_;
  EgoNearestParam ego_nearest_param_;
  TrajectoryParam traj_param_;
  EBParam eb_param_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;

  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;

  rclcpp::Publisher<Trajectory>::SharedPtr debug_eb_traj_pub_;

  std::vector<TrajectoryPoint> insertFixedPoint(
    const std::vector<TrajectoryPoint> & traj_point,
    const std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj);

  std::tuple<std::vector<TrajectoryPoint>, size_t> getPaddedTrajectoryPoints(
    const std::vector<TrajectoryPoint> & traj_points);

  void updateConstrain(const std::vector<TrajectoryPoint> & traj_points);

  ConstrainLines getConstrainLinesFromConstrainRectangle(
    const geometry_msgs::msg::Pose & pose, const double rect_size);

  boost::optional<std::vector<double>> optimizeTrajectory(
    const std::vector<TrajectoryPoint> & traj_points);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertOptimizedPointsToTrajectory(
    const std::vector<double> & optimized_points, const std::vector<TrajectoryPoint> & traj_points,
    const size_t pad_start_idx) const;
};
}  // namespace collision_free_path_planner

#endif  // COLLISION_FREE_PATH_PLANNER__EB_PATH_SMOOTHER_HPP_
