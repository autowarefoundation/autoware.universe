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

#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace collision_free_path_planner
{
class EBPathSmoother
{
public:
  struct Constraint2d
  {
    struct Constraint
    {
      Eigen::Vector2d coef;
      double upper_bound;
      double lower_bound;
    };

    Constraint lon;
    Constraint lat;
  };

  struct EBParam
  {
    // qp
    struct QPParam
    {
      int max_iteration;
      double eps_abs;
      double eps_rel;
    };

    EBParam() = default;
    explicit EBParam(rclcpp::Node * node)
    {
      {  // common
        delta_arc_length = node->declare_parameter<double>("advanced.eb.common.delta_arc_length");
        num_points = node->declare_parameter<int>("advanced.eb.common.num_points");
      }

      {  // clearance
        num_joint_points = node->declare_parameter<int>("advanced.eb.clearance.num_joint_points");
        clearance_for_fix =
          node->declare_parameter<double>("advanced.eb.clearance.clearance_for_fix");
        clearance_for_joint =
          node->declare_parameter<double>("advanced.eb.clearance.clearance_for_joint");
        clearance_for_smooth =
          node->declare_parameter<double>("advanced.eb.clearance.clearance_for_smooth");
      }

      {  // qp
        qp_param.max_iteration = node->declare_parameter<int>("advanced.eb.qp.max_iteration");
        qp_param.eps_abs = node->declare_parameter<double>("advanced.eb.qp.eps_abs");
        qp_param.eps_rel = node->declare_parameter<double>("advanced.eb.qp.eps_rel");
      }
    }

    void onParam(const std::vector<rclcpp::Parameter> & parameters)
    {
      using tier4_autoware_utils::updateParam;

      {  // common
        updateParam<double>(parameters, "advanced.eb.common.delta_arc_length", delta_arc_length);
        updateParam<int>(parameters, "advanced.eb.common.num_points", num_points);
      }

      {  // clearance
        updateParam<int>(parameters, "advanced.eb.clearance.num_joint_points", num_joint_points);
        updateParam<double>(
          parameters, "advanced.eb.clearance.clearance_for_fix", clearance_for_fix);
        updateParam<double>(
          parameters, "advanced.eb.clearance.clearance_for_joint", clearance_for_joint);
        updateParam<double>(
          parameters, "advanced.eb.clearance.clearance_for_smooth", clearance_for_smooth);
      }

      {  // qp
        updateParam<int>(parameters, "advanced.eb.qp.max_iteration", qp_param.max_iteration);
        updateParam<double>(parameters, "advanced.eb.qp.eps_abs", qp_param.eps_abs);
        updateParam<double>(parameters, "advanced.eb.qp.eps_rel", qp_param.eps_rel);
      }
    }

    // common
    double delta_arc_length;
    int num_points;

    // clearance
    int num_joint_points;
    double clearance_for_fix;
    double clearance_for_joint;
    double clearance_for_smooth;

    // qp
    QPParam qp_param;
  };

  EBPathSmoother(
    rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
    const TrajectoryParam & traj_param, const std::shared_ptr<TimeKeeper> time_keeper_ptr);

  std::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>> getEBTrajectory(
    const PlannerData & planner_data);

  void initialize(const bool enable_debug_info, const TrajectoryParam & traj_param);
  void resetPrevData();
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

private:
  bool enable_debug_info_;
  EgoNearestParam ego_nearest_param_;
  TrajectoryParam traj_param_;
  EBParam eb_param_;
  mutable std::shared_ptr<TimeKeeper> time_keeper_ptr_;

  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;

  std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj_points_ptr_{nullptr};
  rclcpp::Publisher<Trajectory>::SharedPtr debug_eb_traj_pub_;

  std::vector<TrajectoryPoint> insertFixedPoint(
    const std::vector<TrajectoryPoint> & traj_point) const;

  std::tuple<std::vector<TrajectoryPoint>, size_t> getPaddedTrajectoryPoints(
    const std::vector<TrajectoryPoint> & traj_points) const;

  void updateConstraint(
    const std::vector<TrajectoryPoint> & traj_points, const bool is_goal_contained) const;

  Constraint2d getConstraint2dFromConstraintSegment(
    const geometry_msgs::msg::Pose & pose, const double constraint_segment_length) const;

  std::optional<std::vector<double>> optimizeTrajectory(
    const std::vector<TrajectoryPoint> & traj_points);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertOptimizedPointsToTrajectory(
    const std::vector<double> & optimized_points, const std::vector<TrajectoryPoint> & traj_points,
    const size_t pad_start_idx) const;
};
}  // namespace collision_free_path_planner

#endif  // COLLISION_FREE_PATH_PLANNER__EB_PATH_SMOOTHER_HPP_
