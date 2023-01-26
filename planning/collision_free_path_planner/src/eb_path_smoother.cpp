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

#include "collision_free_path_planner/eb_path_smoother.hpp"

#include "collision_free_path_planner/type_alias.hpp"
#include "collision_free_path_planner/utils/geometry_utils.hpp"
#include "collision_free_path_planner/utils/trajectory_utils.hpp"
#include "motion_utils/motion_utils.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

namespace
{
Eigen::MatrixXd makePMatrix(const int num_points)
{
  // create P block matrix
  Eigen::MatrixXd P_quater = Eigen::MatrixXd::Zero(num_points, num_points);
  for (int r = 0; r < num_points; ++r) {
    for (int c = 0; c < num_points; ++c) {
      if (r == c) {
        if (r == 0 || r == num_points - 1) {
          P_quater(r, c) = 1.0;
        } else if (r == 1 || r == num_points - 2) {
          P_quater(r, c) = 5.0;
        } else {
          P_quater(r, c) = 6.0;
        }
      } else if (std::abs(c - r) == 1) {
        if (r == 0 || r == num_points - 1) {
          P_quater(r, c) = -2.0;
        } else if (c == 0 || c == num_points - 1) {
          P_quater(r, c) = -2.0;
        } else {
          P_quater(r, c) = -4.0;
        }
      } else if (std::abs(c - r) == 2) {
        P_quater(r, c) = 1.0;
      } else {
        P_quater(r, c) = 0.0;
      }
    }
  }

  // create P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(num_points * 2, num_points * 2);
  P.block(0, 0, num_points, num_points) = P_quater;
  P.block(num_points, num_points, num_points, num_points) = P_quater;

  return P;
}

// make default linear constraint matrix
// NOTE: value (1.0) is not valid. Where non-zero values exis is valid.
Eigen::MatrixXd makeDefaultAMatrix(const int num_points)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_points * 2, num_points * 2);
  for (int i = 0; i < num_points * 2; ++i) {
    if (i < num_points) {
      A(i, i + num_points) = 1.0;
    } else {
      A(i, i - num_points) = 1.0;
    }
  }
  return A;
}
}  // namespace

namespace collision_free_path_planner
{
EBPathSmoother::EBParam::EBParam(rclcpp::Node * node)
{
  {  // option
    enable_optimization_validation =
      node->declare_parameter<bool>("eb.option.enable_optimization_validation");
  }

  {  // common
    delta_arc_length = node->declare_parameter<double>("eb.common.delta_arc_length");
    num_points = node->declare_parameter<int>("eb.common.num_points");
  }

  {  // clearance
    num_joint_points = node->declare_parameter<int>("eb.clearance.num_joint_points");
    clearance_for_fix = node->declare_parameter<double>("eb.clearance.clearance_for_fix");
    clearance_for_joint = node->declare_parameter<double>("eb.clearance.clearance_for_joint");
    clearance_for_smooth = node->declare_parameter<double>("eb.clearance.clearance_for_smooth");
  }

  {  // qp
    qp_param.max_iteration = node->declare_parameter<int>("eb.qp.max_iteration");
    qp_param.eps_abs = node->declare_parameter<double>("eb.qp.eps_abs");
    qp_param.eps_rel = node->declare_parameter<double>("eb.qp.eps_rel");
  }

  // validation
  max_validation_error = node->declare_parameter<double>("eb.validation.max_error");
}

void EBPathSmoother::EBParam::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // common
    updateParam<double>(parameters, "eb.common.delta_arc_length", delta_arc_length);
    updateParam<int>(parameters, "eb.common.num_points", num_points);
  }

  {  // clearance
    updateParam<int>(parameters, "eb.clearance.num_joint_points", num_joint_points);
    updateParam<double>(parameters, "eb.clearance.clearance_for_fix", clearance_for_fix);
    updateParam<double>(parameters, "eb.clearance.clearance_for_joint", clearance_for_joint);
    updateParam<double>(parameters, "eb.clearance.clearance_for_smooth", clearance_for_smooth);
  }

  {  // qp
    updateParam<int>(parameters, "eb.qp.max_iteration", qp_param.max_iteration);
    updateParam<double>(parameters, "eb.qp.eps_abs", qp_param.eps_abs);
    updateParam<double>(parameters, "eb.qp.eps_rel", qp_param.eps_rel);
  }
}

EBPathSmoother::EBPathSmoother(
  rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
  const TrajectoryParam & traj_param, const std::shared_ptr<TimeKeeper> time_keeper_ptr)
: enable_debug_info_(enable_debug_info),
  ego_nearest_param_(ego_nearest_param),
  traj_param_(traj_param),
  time_keeper_ptr_(time_keeper_ptr),
  logger_(node->get_logger().get_child("eb_path_smoother"))
{
  // eb param
  eb_param_ = EBParam(node);

  // initialize osqp solver
  const auto & qp_param = eb_param_.qp_param;
  const int num_points = eb_param_.num_points;

  const Eigen::MatrixXd P = makePMatrix(num_points);
  const std::vector<double> q(num_points * 2, 0.0);

  const Eigen::MatrixXd A = makeDefaultAMatrix(num_points);
  const std::vector<double> lower_bound(num_points * 2, 0.0);
  const std::vector<double> upper_bound(num_points * 2, 0.0);

  osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
    P, A, q, lower_bound, upper_bound, qp_param.eps_abs);
  osqp_solver_ptr_->updateEpsRel(qp_param.eps_rel);
  osqp_solver_ptr_->updateMaxIter(qp_param.max_iteration);

  // publisher
  debug_eb_traj_pub_ = node->create_publisher<Trajectory>("~/debug/eb_trajectory", 1);
}

void EBPathSmoother::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  eb_param_.onParam(parameters);
}

void EBPathSmoother::initialize(const bool enable_debug_info, const TrajectoryParam & traj_param)
{
  enable_debug_info_ = enable_debug_info;
  traj_param_ = traj_param;
}

void EBPathSmoother::resetPrevData() { prev_eb_traj_points_ptr_ = nullptr; }

std::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathSmoother::getEBTrajectory(const PlannerData & planner_data)
{
  time_keeper_ptr_->tic(__func__);

  const auto & p = planner_data;

  // 1. crop trajectory
  const double forward_traj_length = eb_param_.num_points * eb_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;

  const size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(p.traj_points, p.ego_pose, ego_nearest_param_);
  const auto cropped_traj_points = trajectory_utils::cropPoints(
    p.traj_points, p.ego_pose.position, ego_seg_idx, forward_traj_length, -backward_traj_length);

  // check if goal is contained in cropped_traj_points
  const bool is_goal_contained =
    geometry_utils::isSamePoint(cropped_traj_points.back(), planner_data.traj_points.back());
  if (is_goal_contained) {
    // TODO(murooka) remove this.
    std::cerr << "goal is contained " << std::endl;
  }

  // 2. insert fixed point
  // NOTE: Should be after crop trajectory so that fixed point will not be cropped.
  const auto traj_points_with_fixed_point = insertFixedPoint(cropped_traj_points);

  // 3. resample trajectory with delta_arc_length
  const auto resampled_traj_points = trajectory_utils::resampleTrajectoryPoints(
    traj_points_with_fixed_point, eb_param_.delta_arc_length);

  // 4. pad trajectory points
  const auto [padded_traj_points, pad_start_idx] = getPaddedTrajectoryPoints(resampled_traj_points);

  // 5. update constraint for elastic band's QP
  updateConstraint(padded_traj_points, is_goal_contained);

  // 6. get optimization result
  const auto optimized_points = optimizeTrajectory(padded_traj_points);
  if (!optimized_points) {
    RCLCPP_INFO_EXPRESSION(
      logger_, enable_debug_info_, "return std::nullopt since smoothing failed");
    return std::nullopt;
  }

  // 7. convert optimization result to trajectory
  const auto eb_traj_points =
    convertOptimizedPointsToTrajectory(*optimized_points, padded_traj_points, pad_start_idx);
  if (!eb_traj_points) {
    RCLCPP_WARN(logger_, "return std::nullopt since x or y error is too large");
    return std::nullopt;
  }

  prev_eb_traj_points_ptr_ = std::make_shared<std::vector<TrajectoryPoint>>(*eb_traj_points);

  // 8. publish eb trajectory
  const auto eb_traj = trajectory_utils::createTrajectory(p.header, *eb_traj_points);
  debug_eb_traj_pub_->publish(eb_traj);

  time_keeper_ptr_->toc(__func__, "      ");
  return *eb_traj_points;
}

std::vector<TrajectoryPoint> EBPathSmoother::insertFixedPoint(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  time_keeper_ptr_->tic(__func__);

  if (!prev_eb_traj_points_ptr_) {
    return traj_points;
  }

  auto traj_points_with_fixed_point = traj_points;
  const size_t prev_front_seg_idx = trajectory_utils::findEgoSegmentIndex(
    *prev_eb_traj_points_ptr_, traj_points.front().pose, ego_nearest_param_);
  const size_t prev_front_point_idx = prev_front_seg_idx;
  const auto & prev_front_point =
    trajectory_utils::convertToTrajectoryPoint(prev_eb_traj_points_ptr_->at(prev_front_point_idx));

  // update front pose for fix with previous point
  trajectory_utils::updateFrontPointForFix(
    traj_points_with_fixed_point, prev_front_point.pose, eb_param_.delta_arc_length);

  time_keeper_ptr_->toc(__func__, "        ");
  return traj_points_with_fixed_point;
}

std::tuple<std::vector<TrajectoryPoint>, size_t> EBPathSmoother::getPaddedTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  time_keeper_ptr_->tic(__func__);

  const size_t pad_start_idx =
    std::min(static_cast<size_t>(eb_param_.num_points), traj_points.size());

  std::vector<TrajectoryPoint> padded_traj_points;
  for (int i = 0; i < eb_param_.num_points; ++i) {
    const size_t point_idx = i < pad_start_idx ? i : pad_start_idx - 1;
    padded_traj_points.push_back(traj_points.at(point_idx));
  }

  time_keeper_ptr_->toc(__func__, "        ");
  return {padded_traj_points, pad_start_idx};
}

void EBPathSmoother::updateConstraint(
  const std::vector<TrajectoryPoint> & traj_points, const bool is_goal_contained) const
{
  time_keeper_ptr_->tic(__func__);

  const auto & p = eb_param_;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(p.num_points * 2, p.num_points * 2);
  std::vector<double> upper_bound(p.num_points * 2, 0.0);
  std::vector<double> lower_bound(p.num_points * 2, 0.0);
  for (int i = 0; i < p.num_points; ++i) {
    const double constraint_segment_length = [&]() {
      if (i == 0 || (is_goal_contained && i == p.num_points - 1)) {
        return p.clearance_for_fix;
      } else if (i < p.num_joint_points + 1) {  // 1 is added since index 0 is fixed point
        return p.clearance_for_joint;
      }
      return p.clearance_for_smooth;
    }();

    const auto constraint =
      getConstraint2dFromConstraintSegment(traj_points.at(i).pose, constraint_segment_length);

    // longitudinal constraint
    A(i, i) = constraint.lon.coef(0);
    A(i, i + p.num_points) = constraint.lon.coef(1);
    upper_bound.at(i) = constraint.lon.upper_bound;
    lower_bound.at(i) = constraint.lon.lower_bound;

    // lateral constraint
    A(i + p.num_points, i) = constraint.lat.coef(0);
    A(i + p.num_points, i + p.num_points) = constraint.lat.coef(1);
    upper_bound.at(i + p.num_points) = constraint.lat.upper_bound;
    lower_bound.at(i + p.num_points) = constraint.lat.lower_bound;
  }

  osqp_solver_ptr_->updateA(A);
  osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
  osqp_solver_ptr_->updateEpsRel(p.qp_param.eps_rel);
  osqp_solver_ptr_->updateEpsAbs(p.qp_param.eps_abs);
  osqp_solver_ptr_->updateMaxIter(p.qp_param.max_iteration);

  time_keeper_ptr_->toc(__func__, "        ");
}

EBPathSmoother::Constraint2d EBPathSmoother::getConstraint2dFromConstraintSegment(
  const geometry_msgs::msg::Pose & pose, const double constraint_segment_length) const
{
  Constraint2d constraint;
  const double theta = tf2::getYaw(pose.orientation);

  // longitudinal constraint
  constraint.lon.coef = Eigen::Vector2d(std::cos(theta), std::sin(theta));
  const double lon_bound =
    constraint.lon.coef.transpose() * Eigen::Vector2d(pose.position.x, pose.position.y);
  constraint.lon.upper_bound = lon_bound;
  constraint.lon.lower_bound = lon_bound;

  // lateral constraint
  constraint.lat.coef = Eigen::Vector2d(std::sin(theta), -std::cos(theta));
  const auto lat_bound_pos1 =
    tier4_autoware_utils::calcOffsetPose(pose, 0.0, -constraint_segment_length / 2.0, 0.0).position;
  const auto lat_bound_pos2 =
    tier4_autoware_utils::calcOffsetPose(pose, 0.0, constraint_segment_length / 2.0, 0.0).position;
  const double lat_bound1 =
    constraint.lat.coef.transpose() * Eigen::Vector2d(lat_bound_pos1.x, lat_bound_pos1.y);
  const double lat_bound2 =
    constraint.lat.coef.transpose() * Eigen::Vector2d(lat_bound_pos2.x, lat_bound_pos2.y);
  constraint.lat.upper_bound = std::max(lat_bound1, lat_bound2);
  constraint.lat.lower_bound = std::min(lat_bound1, lat_bound2);

  return constraint;
}

std::optional<std::vector<double>> EBPathSmoother::optimizeTrajectory(
  const std::vector<TrajectoryPoint> & traj_points)
{
  time_keeper_ptr_->tic(__func__);

  // solve QP
  const auto result = osqp_solver_ptr_->optimize();
  const auto optimized_points = std::get<0>(result);

  const auto status = std::get<3>(result);

  // check status
  if (status != 1) {
    osqp_solver_ptr_->logUnsolvedStatus("[EB]");
    return std::nullopt;
  }

  time_keeper_ptr_->toc(__func__, "        ");
  return optimized_points;
}

std::optional<std::vector<TrajectoryPoint>> EBPathSmoother::convertOptimizedPointsToTrajectory(
  const std::vector<double> & optimized_points, const std::vector<TrajectoryPoint> & traj_points,
  const size_t pad_start_idx) const
{
  time_keeper_ptr_->tic(__func__);

  auto eb_traj_points = traj_points;

  // update only x and y
  for (size_t i = 0; i < pad_start_idx; ++i) {
    // validate optimization result
    if (eb_param_.enable_optimization_validation) {
      const double diff_x = optimized_points.at(i) - eb_traj_points.at(i).pose.position.x;
      const double diff_y =
        optimized_points.at(i + eb_param_.num_points) - eb_traj_points.at(i).pose.position.y;
      if (
        eb_param_.max_validation_error < std::abs(diff_x) ||
        eb_param_.max_validation_error < std::abs(diff_y)) {
        return std::nullopt;
      }
    }

    eb_traj_points.at(i).pose.position.x = optimized_points.at(i);
    eb_traj_points.at(i).pose.position.y = optimized_points.at(i + eb_param_.num_points);
  }

  // update orientation
  motion_utils::insertOrientation(eb_traj_points, true);

  time_keeper_ptr_->toc(__func__, "        ");
  return eb_traj_points;
}
}  // namespace collision_free_path_planner
