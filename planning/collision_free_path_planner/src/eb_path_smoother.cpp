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

// TODO(murooka) consider fixing goal
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
Eigen::MatrixXd makeAMatrix(const int num_points)
{
  // TODO(murooka) check
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
EBPathSmoother::EBPathSmoother(
  rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
  const TrajectoryParam & traj_param, const std::shared_ptr<DebugData> debug_data_ptr)
: enable_debug_info_(enable_debug_info),
  ego_nearest_param_(ego_nearest_param),
  traj_param_(traj_param),
  debug_data_ptr_(debug_data_ptr)
{
  // eb param
  eb_param_ = EBParam(node);

  // initialize osqp solver
  const auto & qp_param = eb_param_.qp_param;
  const int num_points = eb_param_.num_points;

  const Eigen::MatrixXd P = makePMatrix(num_points);
  const std::vector<double> q(num_points * 2, 0.0);

  const Eigen::MatrixXd A = makeAMatrix(num_points);
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

void EBPathSmoother::reset(const bool enable_debug_info, const TrajectoryParam & traj_param)
{
  enable_debug_info_ = enable_debug_info;
  traj_param_ = traj_param;
  prev_eb_traj_points_ptr_ = nullptr;
}

std::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathSmoother::getEBTrajectory(const PlannerData & planner_data)
{
  debug_data_ptr_->tic(__func__);

  const auto & p = planner_data;

  // 1. crop trajectory
  const double forward_traj_length = eb_param_.num_points * eb_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;

  const size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(p.traj_points, p.ego_pose, ego_nearest_param_);
  const auto cropped_traj_points = trajectory_utils::cropPoints(
    p.traj_points, p.ego_pose.position, ego_seg_idx, forward_traj_length, -backward_traj_length);

  // 2. insert fixed point
  // NOTE: Should be after crop trajectory so that fixed point will not be cropped.
  const auto traj_points_with_fixed_point = insertFixedPoint(cropped_traj_points);

  // 3. resample trajectory with delta_arc_length
  const auto resampled_traj_points = trajectory_utils::resampleTrajectoryPoints(
    traj_points_with_fixed_point, eb_param_.delta_arc_length);

  // 4. pad trajectory points
  const auto [padded_traj_points, pad_start_idx] = getPaddedTrajectoryPoints(resampled_traj_points);

  // 5. update constraint for elastic band's QP
  updateConstraint(padded_traj_points);

  // 6. get optimization result
  const auto optimized_points = optimizeTrajectory(padded_traj_points);
  if (!optimized_points) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathSmoother"), enable_debug_info_, "return {} since smoothing failed");
    return {};
  }

  // convert optimization result to trajectory
  const auto eb_traj_points =
    convertOptimizedPointsToTrajectory(*optimized_points, padded_traj_points, pad_start_idx);
  prev_eb_traj_points_ptr_ = std::make_shared<std::vector<TrajectoryPoint>>(eb_traj_points);

  debug_data_ptr_->eb_traj = eb_traj_points;

  // publish eb trajectory
  const auto eb_traj = trajectory_utils::createTrajectory(p.header, eb_traj_points);
  debug_eb_traj_pub_->publish(eb_traj);

  debug_data_ptr_->toc(__func__, "      ");
  return eb_traj_points;
}

std::vector<TrajectoryPoint> EBPathSmoother::insertFixedPoint(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  debug_data_ptr_->tic(__func__);

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
  const double front_velocity = traj_points_with_fixed_point.front().longitudinal_velocity_mps;
  trajectory_utils::updateFrontPoseForFix(
    traj_points_with_fixed_point, prev_front_point, eb_param_.delta_arc_length);

  // update front velocity with current point
  traj_points_with_fixed_point.front().longitudinal_velocity_mps = front_velocity;

  debug_data_ptr_->toc(__func__, "        ");
  return traj_points_with_fixed_point;
}

std::tuple<std::vector<TrajectoryPoint>, size_t> EBPathSmoother::getPaddedTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  debug_data_ptr_->tic(__func__);

  const size_t pad_start_idx =
    std::min(static_cast<size_t>(eb_param_.num_points), traj_points.size());

  std::vector<TrajectoryPoint> padded_traj_points;
  for (int i = 0; i < eb_param_.num_points; ++i) {
    const size_t point_idx = i < pad_start_idx ? i : pad_start_idx - 1;
    padded_traj_points.push_back(traj_points.at(point_idx));
  }

  debug_data_ptr_->toc(__func__, "        ");
  return {padded_traj_points, pad_start_idx};
}

void EBPathSmoother::updateConstraint(const std::vector<TrajectoryPoint> & traj_points) const
{
  debug_data_ptr_->tic(__func__);

  const auto & p = eb_param_;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(p.num_points * 2, p.num_points * 2);
  std::vector<double> upper_bound(p.num_points * 2, 0.0);
  std::vector<double> lower_bound(p.num_points * 2, 0.0);
  for (int i = 0; i < p.num_points; ++i) {
    const double rect_size = [&]() {
      if (i == 0) {
        return p.clearance_for_fix;
      } else if (i < p.num_joint_points + 1) {  // 1 is added since index 0 is fixed point
        return p.clearance_for_joint;
      }
      return p.clearance_for_smooth;
    }();

    const auto constraint =
      getConstraintLinesFromConstraintRectangle(traj_points.at(i).pose, rect_size);

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

  debug_data_ptr_->toc(__func__, "        ");
}

EBPathSmoother::ConstraintLines EBPathSmoother::getConstraintLinesFromConstraintRectangle(
  const geometry_msgs::msg::Pose & pose, const double rect_size) const
{
  ConstraintLines constraint;
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
    tier4_autoware_utils::calcOffsetPose(pose, 0.0, -rect_size / 2.0, 0.0).position;
  const auto lat_bound_pos2 =
    tier4_autoware_utils::calcOffsetPose(pose, 0.0, rect_size / 2.0, 0.0).position;
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
  debug_data_ptr_->tic(__func__);

  // solve QP
  const auto result = osqp_solver_ptr_->optimize();
  const auto optimized_points = std::get<0>(result);

  const auto status = std::get<3>(result);

  // check status
  if (status != 1) {
    osqp_solver_ptr_->logUnsolvedStatus("[EB]");
    return {};
  }

  debug_data_ptr_->toc(__func__, "        ");
  return optimized_points;
}

std::vector<TrajectoryPoint> EBPathSmoother::convertOptimizedPointsToTrajectory(
  const std::vector<double> & optimized_points, const std::vector<TrajectoryPoint> & traj_points,
  const size_t pad_start_idx) const
{
  debug_data_ptr_->tic(__func__);

  auto eb_traj_points = traj_points;

  // update only x and y
  for (size_t i = 0; i < pad_start_idx; ++i) {
    eb_traj_points.at(i).pose.position.x = optimized_points.at(i);
    eb_traj_points.at(i).pose.position.y = optimized_points.at(i + eb_param_.num_points);
  }

  // update orientation
  motion_utils::insertOrientation(eb_traj_points, true);

  debug_data_ptr_->toc(__func__, "        ");
  return eb_traj_points;
}
}  // namespace collision_free_path_planner
