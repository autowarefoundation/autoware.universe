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

#include "path_smoothing/elastic_band.hpp"

#include "motion_utils/motion_utils.hpp"
#include "path_smoothing/type_alias.hpp"
#include "path_smoothing/utils/geometry_utils.hpp"
#include "path_smoothing/utils/trajectory_utils.hpp"
#include "tf2/utils.h"

#include <algorithm>
#include <chrono>
#include <limits>

namespace
{

const auto value_dist_0(const int r, const int num_points)
{
  if (r == 0 || r == num_points - 1)
    return 1.0;
  else if (r == 1 || r == num_points - 2)
    return 5.0;
  else
    return 6.0;
}

const auto value_dist_1(const int r, const int c, const int num_points)
{
  if (r == 0 || r == num_points - 1)
    return -2.0;
  else if (c == 0 || c == num_points - 1)
    return -2.0;
  else
    return -4.0;
}

double p_matrice_value(const int r, const int c, const int num_points)
{
  double value = 0.0;
  if (r == c)
    value = value_dist_0(r, num_points);
  else if (std::abs(c - r) == 1)
    value = value_dist_1(r, c, num_points);
  else if (std::abs(c - r) == 2)
    value = 1.0;
  return value;
}

Eigen::MatrixXd makePMatrix(const int num_points)
{
  // create P block matrix
  Eigen::MatrixXd P_quarter = Eigen::MatrixXd::Zero(num_points, num_points);
  for (int r = 0; r < num_points; ++r)
    for (int c = 0; c < num_points; ++c) P_quarter(r, c) = p_matrice_value(r, c, num_points);

  // create P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(num_points * 2, num_points * 2);
  P.block(0, 0, num_points, num_points) = P_quarter;
  P.block(num_points, num_points, num_points, num_points) = P_quarter;

  return P;
}

std::vector<double> toStdVector(const Eigen::VectorXd & eigen_vec)
{
  return {eigen_vec.data(), eigen_vec.data() + eigen_vec.rows()};
}
}  // namespace

namespace path_smoothing
{

EBPathSmoother::EBPathSmoother(
  rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
  const CommonParam & common_param, const std::shared_ptr<TimeKeeper> time_keeper_ptr)
: enable_debug_info_(enable_debug_info),
  ego_nearest_param_(ego_nearest_param),
  common_param_(common_param),
  time_keeper_ptr_(time_keeper_ptr),
  logger_(node->get_logger().get_child("elastic_band_smoother"))
{
  // eb param
  eb_param_ = EBParam(node);

  // publisher
  debug_eb_traj_pub_ = node->create_publisher<Trajectory>("~/debug/eb_traj", 1);
  debug_eb_fixed_traj_pub_ = node->create_publisher<Trajectory>("~/debug/eb_fixed_traj", 1);
}

void EBPathSmoother::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  eb_param_.onParam(parameters);
}

void EBPathSmoother::initialize(const bool enable_debug_info, const CommonParam & common_param)
{
  enable_debug_info_ = enable_debug_info;
  common_param_ = common_param;
}

void EBPathSmoother::resetPreviousData()
{
  prev_eb_traj_points_ptr_ = nullptr;
}

std::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathSmoother::getEBTrajectory(const PlannerData & planner_data)
{
  time_keeper_ptr_->tic(__func__);

  const auto & p = planner_data;

  // 1. crop trajectory
  const double forward_traj_length = eb_param_.num_points * eb_param_.delta_arc_length;
  const double backward_traj_length = common_param_.output_backward_traj_length;

  const size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(p.traj_points, p.ego_pose, ego_nearest_param_);
  const auto cropped_traj_points = motion_utils::cropPoints(
    p.traj_points, p.ego_pose.position, ego_seg_idx, forward_traj_length, backward_traj_length);

  // check if goal is contained in cropped_traj_points
  const bool is_goal_contained =
    geometry_utils::isSamePoint(cropped_traj_points.back(), planner_data.traj_points.back());

  // 2. insert fixed point
  // NOTE: This should be after cropping trajectory so that fixed point will not be cropped.
  const auto traj_points_with_fixed_point = insertFixedPoint(cropped_traj_points);

  // 3. resample trajectory with delta_arc_length
  const auto resampled_traj_points = [&]() {
    // NOTE: If the interval of points is not constant, the optimization is sometimes unstable.
    //       Therefore, we do not resample a stop point here.
    auto tmp_traj_points = trajectory_utils::resampleTrajectoryPointsWithoutStopPoint(
      traj_points_with_fixed_point, eb_param_.delta_arc_length);

    // NOTE: The front point is previous optimized one, and the others are the input ones.
    //       There may be a lateral error between the points, which makes orientation unexpected.
    //       Therefore, the front pose is updated after resample.
    tmp_traj_points.front().pose = traj_points_with_fixed_point.front().pose;
    return tmp_traj_points;
  }();

  // 4. pad trajectory points
  const auto [padded_traj_points, pad_start_idx] = getPaddedTrajectoryPoints(resampled_traj_points);

  // 5. update constraint for elastic band's QP
  updateConstraint(p.header, padded_traj_points, is_goal_contained, pad_start_idx);

  // 6. get optimization result
  const auto optimized_points = optimizeTrajectory();
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
  // replace the front pose with previous points
  trajectory_utils::updateFrontPointForFix(
    traj_points_with_fixed_point, *prev_eb_traj_points_ptr_, eb_param_.delta_arc_length,
    ego_nearest_param_);

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
  for (size_t i = 0; i < static_cast<size_t>(eb_param_.num_points); ++i) {
    const size_t point_idx = i < pad_start_idx ? i : pad_start_idx - 1;
    padded_traj_points.push_back(traj_points.at(point_idx));
  }

  time_keeper_ptr_->toc(__func__, "        ");
  return {padded_traj_points, pad_start_idx};
}

std::vector<double> calculate_upper_bound(
  const EBParam & eb_param, const bool is_goal_contained, const int pad_start_idx,
  std::vector<size_t> & debug_fixed_traj_point_indexes)
{
  const auto & p = eb_param;

  const auto constraint_segment_length = [&](const auto i) {
    // NOTE: fix goal and its previous pose to keep goal orientation
    const auto is_goal_or_previous =
      is_goal_contained &&
      (p.num_points - 2 <= static_cast<int>(i) || pad_start_idx - 2 <= static_cast<int>(i));
    // NOTE: Only first point can be fixed since there is a lateral deviation
    //       between the two points.
    //       The front point is previous optimized one, and the others are the input ones.
    const auto use_clearance_for_fix = i == 0 || is_goal_or_previous;
    if (use_clearance_for_fix) return p.clearance_for_fix;
    if (i < static_cast<size_t>(p.num_joint_points) + 1)  // 1 is added since index 0 is fixed point
      return p.clearance_for_joint;
    return p.clearance_for_smooth;
  };

  std::vector<double> upper_bound(eb_param.num_points, 0.0);
  for (size_t i = 0; i < upper_bound.size(); ++i) {
    upper_bound[i] = constraint_segment_length(i);

    if (upper_bound[i] == 0.0) debug_fixed_traj_point_indexes.push_back(i);
  }
  return upper_bound;
}

void EBPathSmoother::updateConstraint(
  const std_msgs::msg::Header & header, const std::vector<TrajectoryPoint> & traj_points,
  const bool is_goal_contained, const int pad_start_idx)
{
  time_keeper_ptr_->tic(__func__);

  const auto & p = eb_param_;

  std::vector<size_t> debug_fixed_traj_point_indexes;

  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(p.num_points, p.num_points);

  const auto upper_bound = calculate_upper_bound(
    eb_param_, is_goal_contained, pad_start_idx, debug_fixed_traj_point_indexes);
  std::vector<double> lower_bound = upper_bound;
  for (auto & lb : lower_bound) lb *= -1;

  std::vector<TrajectoryPoint> debug_fixed_traj_points;
  for (const auto idx : debug_fixed_traj_point_indexes)
    debug_fixed_traj_points.push_back(traj_points[idx]);

  Eigen::VectorXd x_mat(2 * p.num_points);
  Eigen::MatrixXd theta_mat = Eigen::MatrixXd::Zero(p.num_points, 2 * p.num_points);
  for (size_t i = 0; i < static_cast<size_t>(p.num_points); ++i) {
    x_mat(i) = traj_points.at(i).pose.position.x;
    x_mat(i + p.num_points) = traj_points.at(i).pose.position.y;

    const double yaw = tf2::getYaw(traj_points.at(i).pose.orientation);
    theta_mat(i, i) = -std::sin(yaw);
    theta_mat(i, i + p.num_points) = std::cos(yaw);
  }

  // calculate P
  const Eigen::MatrixXd raw_P_for_smooth = p.smooth_weight * makePMatrix(p.num_points);
  const Eigen::MatrixXd P_for_smooth = theta_mat * raw_P_for_smooth * theta_mat.transpose();
  const Eigen::MatrixXd P_for_lat_error =
    p.lat_error_weight * Eigen::MatrixXd::Identity(p.num_points, p.num_points);
  const Eigen::MatrixXd P = P_for_smooth + P_for_lat_error;

  // calculate q
  const Eigen::VectorXd raw_q_for_smooth = theta_mat * raw_P_for_smooth * x_mat;
  const auto q = toStdVector(raw_q_for_smooth);

  if (p.enable_warm_start && osqp_solver_ptr_) {
    osqp_solver_ptr_->updateP(P);
    osqp_solver_ptr_->updateQ(q);
    osqp_solver_ptr_->updateA(A);
    osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
    osqp_solver_ptr_->updateEpsRel(p.qp_param.eps_rel);
  } else {
    osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
      P, A, q, lower_bound, upper_bound, p.qp_param.eps_abs);
    osqp_solver_ptr_->updateEpsRel(p.qp_param.eps_rel);
    osqp_solver_ptr_->updateEpsAbs(p.qp_param.eps_abs);
    osqp_solver_ptr_->updateMaxIter(p.qp_param.max_iteration);
  }

  // publish fixed trajectory
  const auto eb_fixed_traj = trajectory_utils::createTrajectory(header, debug_fixed_traj_points);
  debug_eb_fixed_traj_pub_->publish(eb_fixed_traj);

  time_keeper_ptr_->toc(__func__, "        ");
}

std::optional<std::vector<double>> EBPathSmoother::optimizeTrajectory()
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
  const auto has_nan = std::any_of(
    optimized_points.begin(), optimized_points.end(), [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    RCLCPP_WARN(logger_, "optimization failed: result contains NaN values");
    return std::nullopt;
  }

  time_keeper_ptr_->toc(__func__, "        ");
  return optimized_points;
}

std::optional<std::vector<TrajectoryPoint>> EBPathSmoother::convertOptimizedPointsToTrajectory(
  const std::vector<double> & optimized_points, const std::vector<TrajectoryPoint> & traj_points,
  const int pad_start_idx) const
{
  time_keeper_ptr_->tic(__func__);

  std::vector<TrajectoryPoint> eb_traj_points;

  // update only x and y
  for (size_t i = 0; i < static_cast<size_t>(pad_start_idx); ++i) {
    const double lat_offset = optimized_points.at(i);

    // validate optimization result
    if (eb_param_.enable_optimization_validation) {
      if (eb_param_.max_validation_error < std::abs(lat_offset)) {
        return std::nullopt;
      }
    }

    auto eb_traj_point = traj_points.at(i);
    eb_traj_point.pose =
      tier4_autoware_utils::calcOffsetPose(eb_traj_point.pose, 0.0, lat_offset, 0.0);
    eb_traj_points.push_back(eb_traj_point);
  }

  // update orientation
  motion_utils::insertOrientation(eb_traj_points, true);

  time_keeper_ptr_->toc(__func__, "        ");
  return eb_traj_points;
}
}  // namespace path_smoothing
