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

#ifndef COLLISION_FREE_PATH_PLANNER__MPT_OPTIMIZER_HPP_
#define COLLISION_FREE_PATH_PLANNER__MPT_OPTIMIZER_HPP_

#include "collision_free_path_planner/common_structs.hpp"
#include "collision_free_path_planner/state_equation_generator.hpp"
#include "collision_free_path_planner/type_alias.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"
#include "gtest/gtest.h"
#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace collision_free_path_planner
{
struct Bounds
{
  double lower_bound;
  double upper_bound;

  static Bounds lerp(Bounds prev_bounds, Bounds next_bounds, double ratio)
  {
    const double lower_bound =
      interpolation::lerp(prev_bounds.lower_bound, next_bounds.lower_bound, ratio);
    const double upper_bound =
      interpolation::lerp(prev_bounds.upper_bound, next_bounds.upper_bound, ratio);

    return Bounds{lower_bound, upper_bound};
  }

  void translate(const double offset)
  {
    lower_bound -= offset;
    upper_bound -= offset;
  }
};

struct KinematicState
{
  double lat{0.0};
  double yaw{0.0};

  Eigen::Vector2d toEigenVector() const { return Eigen::Vector2d{lat, yaw}; }
};

struct ReferencePoint
{
  geometry_msgs::msg::Pose pose;
  double longitudinal_velocity_mps;

  // additional information
  double k{0.0};
  double delta_arc_length{0.0};
  double alpha{0.0};
  Bounds bounds{};  // bounds on pose
  bool near_objects{false};
  std::vector<std::optional<double>> beta{};

  // NOTE: fix_kinematic_state is used for two purposes
  //       one is fixing points around ego for stability
  //       second is fixing current ego pose when no velocity for planning from ego pose
  std::optional<KinematicState> fix_kinematic_state{std::nullopt};
  KinematicState optimized_kinematic_state{};
  double optimized_input{};

  // bounds and its local pose on each collision-free constraint
  std::vector<Bounds> bounds_on_constraints{};
  std::vector<geometry_msgs::msg::Pose> pose_on_constraints{};

  double getYaw() const { return tf2::getYaw(pose.orientation); }

  geometry_msgs::msg::Pose offsetDeviation(const double lat_dev, const double yaw_dev) const
  {
    auto pose_with_deviation = tier4_autoware_utils::calcOffsetPose(pose, 0.0, lat_dev, 0.0);
    pose_with_deviation.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(getYaw() + yaw_dev);
    return pose_with_deviation;
  }
};

struct MPTTrajs
{
  std::vector<ReferencePoint> ref_points{};
  std::vector<TrajectoryPoint> mpt{};
};

class MPTOptimizer
{
  FRIEND_TEST(CollisionFreePathPlanner, MPTOptimizer);

public:
  MPTOptimizer(
    rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
    const vehicle_info_util::VehicleInfo & vehicle_info, const TrajectoryParam & traj_param,
    const std::shared_ptr<DebugData> debug_data_ptr);

  std::optional<MPTTrajs> getModelPredictiveTrajectory(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points);

  void reset(const bool enable_debug_info, const TrajectoryParam & traj_param);
  void resetPrevData();
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

private:
  struct ValueMatrix
  {
    Eigen::SparseMatrix<double> Q;
    Eigen::SparseMatrix<double> R;
  };

  struct ObjectiveMatrix
  {
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
  };

  struct ConstraintMatrix
  {
    Eigen::MatrixXd linear;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
  };

  struct MPTParam
  {
    bool enable_warm_start;
    bool enable_manual_warm_start;
    bool steer_limit_constraint;
    bool fix_points_around_ego;
    bool is_fixed_point_single;

    std::vector<double> vehicle_circle_longitudinal_offsets;  // from base_link
    std::vector<double> vehicle_circle_radiuses;

    double delta_arc_length;
    int num_points;

    double hard_clearance_from_road;
    double soft_clearance_from_road;
    double soft_avoidance_weight;

    double lat_error_weight;
    double yaw_error_weight;
    double yaw_error_rate_weight;

    double near_objects_length;

    double terminal_lat_error_weight;
    double terminal_yaw_error_weight;
    double terminal_path_lat_error_weight;
    double terminal_path_yaw_error_weight;

    double steer_input_weight;
    double steer_rate_weight;

    double obstacle_avoid_lat_error_weight;
    double obstacle_avoid_yaw_error_weight;
    double obstacle_avoid_steer_input_weight;

    double optimization_center_offset;
    double max_steer_rad;

    std::vector<double> bounds_search_widths;

    bool soft_constraint;
    bool hard_constraint;
    bool l_inf_norm;
  };

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr debug_fixed_traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr debug_ref_traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr debug_mpt_traj_pub_;

  // argument parameter
  bool enable_debug_info_;
  EgoNearestParam ego_nearest_param_;
  vehicle_info_util::VehicleInfo vehicle_info_;
  TrajectoryParam traj_param_;
  MPTParam mpt_param_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;
  StateEquationGenerator state_equation_generator_;

  // autoware::common::osqp::OSQPInterface osqp_solver_;
  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;

  const double osqp_epsilon_ = 1.0e-3;
  int mpt_visualize_sampling_num_;

  // vehicle circles info for for mpt constraints
  std::string vehicle_circle_method_;
  int vehicle_circle_num_for_calculation_;
  std::vector<double> vehicle_circle_radius_ratios_;

  // previous information
  int prev_mat_n_ = 0;
  int prev_mat_m_ = 0;
  std::shared_ptr<std::vector<ReferencePoint>> prev_ref_points_ptr_{nullptr};

  void initializeMPTParam(rclcpp::Node * node, const vehicle_info_util::VehicleInfo & vehicle_info);

  // void calcPlanningFromEgo(
  //   const geometry_msgs::msg::Pose & ego_pose, const double ego_vel,
  //   std::vector<ReferencePoint> & ref_points) const;

  // pstd::vector<ReferencePoint> getFixedReferencePoints(
  //   const geometry_msgs::msg::Pose & ego_pose, const double ego_vel,
  //   const std::vector<TrajectoryPoint> & smoothed_points,
  //   const std::shared_ptr<MPTTrajs> prev_trajs) const;

  std::vector<ReferencePoint> calcReferencePoints(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points) const;
  void updateCurvature(
    std::vector<ReferencePoint> & ref_points,
    const SplineInterpolationPoints2d & ref_points_spline) const;
  void updateOrientation(
    std::vector<ReferencePoint> & ref_points,
    const SplineInterpolationPoints2d & ref_points_spline) const;
  void updateBounds(
    std::vector<ReferencePoint> & ref_points,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound) const;
  void updateVehicleBounds(
    std::vector<ReferencePoint> & ref_points,
    const SplineInterpolationPoints2d & ref_points_spline) const;
  void updateFixedPoint(std::vector<ReferencePoint> & ref_points) const;
  void updateArcLength(std::vector<ReferencePoint> & ref_points) const;
  void calcExtraPoints(std::vector<ReferencePoint> & ref_points) const;

  ValueMatrix calcValueMatrix(
    const std::vector<ReferencePoint> & reference_points,
    const std::vector<TrajectoryPoint> & traj_points) const;

  ObjectiveMatrix getObjectiveMatrix(
    const StateEquationGenerator::Matrix & mpt_mat, const ValueMatrix & obj_mat,
    const std::vector<ReferencePoint> & ref_points) const;

  ConstraintMatrix getConstraintMatrix(
    const StateEquationGenerator::Matrix & mpt_mat,
    const std::vector<ReferencePoint> & ref_points) const;

  std::optional<Eigen::VectorXd> calcOptimizedSteerAngles(
    const std::vector<ReferencePoint> & ref_points, const ObjectiveMatrix & obj_mat,
    const ConstraintMatrix & const_mat);
  Eigen::VectorXd calcInitialSolutionForManualWarmStart(
    const std::vector<ReferencePoint> & ref_points,
    const std::vector<ReferencePoint> & prev_ref_points) const;
  std::pair<ObjectiveMatrix, ConstraintMatrix> updateMatrixForManualWarmStart(
    const ObjectiveMatrix & obj_mat, const ConstraintMatrix & const_mat,
    const std::optional<Eigen::VectorXd> & u0) const;

  void addSteerWeightR(
    std::vector<Eigen::Triplet<double>> & R_triplet_vec,
    const std::vector<ReferencePoint> & ref_points) const;

  std::vector<TrajectoryPoint> calcMPTPoints(
    std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & Uex,
    const StateEquationGenerator::Matrix & mpt_matrix);

  // functions for debug publish
  void publishDebugTrajectories(
    const std_msgs::msg::Header & header, const std::vector<ReferencePoint> & ref_points,
    const std::vector<TrajectoryPoint> & mpt_traj_points) const;

  std::vector<TrajectoryPoint> extractFixedPoints(
    const std::vector<ReferencePoint> & ref_points) const;

  void logInfo(const char * msg) const
  {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("CollisionFreePathPlanner::MPTOptimizer"), enable_debug_info_, msg);
  }
};
}  // namespace collision_free_path_planner
#endif  // COLLISION_FREE_PATH_PLANNER__MPT_OPTIMIZER_HPP_
