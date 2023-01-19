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
// make positive semidefinite matrix for objective function
// reference: https://ieeexplore.ieee.org/document/7402333
// NOTE: matrix size is (2 num_points x 2 num_points)
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

// make default linear constrain matrix
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

  {  // initialize osqp solver
    const auto & qp_param = eb_param_.qp_param;

    const Eigen::MatrixXd p = makePMatrix(eb_param_.num_sampling_points);
    const Eigen::MatrixXd a = makeAMatrix(eb_param_.num_sampling_points);

    const int num_points = eb_param_.num_sampling_points;
    const std::vector<double> q(num_points * 2, 0.0);
    const std::vector<double> lower_bound(num_points * 2, 0.0);
    const std::vector<double> upper_bound(num_points * 2, 0.0);

    osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
      p, a, q, lower_bound, upper_bound, qp_param.eps_abs);
    osqp_solver_ptr_->updateEpsRel(qp_param.eps_rel);
    osqp_solver_ptr_->updateMaxIter(qp_param.max_iteration);
  }

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
}

boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathSmoother::getEBTrajectory(
  const PlannerData & planner_data,
  const std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj)
{
  debug_data_ptr_->tic(__func__);

  const auto & p = planner_data;

  // 1. convert path points to trajectory points
  const auto traj_points = trajectory_utils::convertToTrajectoryPoints(p.path.points);

  // 2. insert fixed point
  const auto traj_points_with_fixed_point = insertFixedPoint(traj_points, prev_eb_traj);

  // 3. crop trajectory
  const double forward_traj_length = eb_param_.num_sampling_points * eb_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;

  const size_t ego_seg_idx = trajectory_utils::findEgoSegmentIndex(
    traj_points_with_fixed_point, p.ego_pose, ego_nearest_param_);
  const auto cropped_traj_points = trajectory_utils::cropPoints(
    traj_points_with_fixed_point, p.ego_pose.position, ego_seg_idx, forward_traj_length,
    -backward_traj_length);

  // 4. resample trajectory for logic purpose
  const auto resampled_traj_points =
    trajectory_utils::resampleTrajectoryPoints(cropped_traj_points, eb_param_.delta_arc_length);

  // 5. pad trajectory points
  const auto [padded_traj_points, pad_start_idx] = getPaddedTrajectoryPoints(resampled_traj_points);

  // 6. update constrain for elastic band's QP
  updateConstrain(padded_traj_points);

  // 7. get optimized smooth points with elastic band
  const auto optimized_points = optimizeTrajectory(traj_points);
  if (!optimized_points) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathSmoother"), enable_debug_info_,
      "return boost::none since smoothing failed");
    return boost::none;
  }

  // convert to trajectory
  const auto eb_traj_points =
    convertOptimizedPointsToTrajectory(optimized_points.get(), pad_start_idx);

  debug_data_ptr_->eb_traj = eb_traj_points;

  {  // publish eb trajectory
    const auto eb_traj = trajectory_utils::createTrajectory(p.path.header, eb_traj_points);
    debug_eb_traj_pub_->publish(eb_traj);
  }

  debug_data_ptr_->toc(__func__, "      ");
  return eb_traj_points;
}

std::vector<TrajectoryPoint> EBPathSmoother::insertFixedPoint(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj)
{
  if (!prev_eb_traj) {
    return traj_points;
  }

  auto traj_points_with_fixed_point = traj_points;
  const size_t prev_front_seg_idx = trajectory_utils::findEgoSegmentIndex(
    *prev_eb_traj, traj_points.front().pose, ego_nearest_param_);
  const size_t prev_front_point_idx = prev_front_seg_idx;
  const auto & prev_front_point =
    trajectory_utils::convertToTrajectoryPoint(prev_eb_traj->at(prev_front_point_idx));

  // update front point of ref_points
  trajectory_utils::insertFrontPoint(traj_points_with_fixed_point, prev_front_point);

  return traj_points_with_fixed_point;
}

std::tuple<std::vector<TrajectoryPoint>, size_t> EBPathSmoother::getPaddedTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points)
{
  const size_t pad_start_idx =
    std::min(static_cast<size_t>(eb_param_.num_sampling_points), traj_points.size());
  // std::cerr << _points.size() << std::endl;
  // std::cerr << pad_start_idx << std::endl;

  std::vector<TrajectoryPoint> padded_traj_points;
  for (int i = 0; i < eb_param_.num_sampling_points; ++i) {
    const size_t point_idx = i < pad_start_idx ? i : pad_start_idx - 1;
    padded_traj_points.push_back(traj_points.at(point_idx));

    // std::cerr << padded_path_points.back().pose.position.x << " "
    //           << padded_path_points.back().pose.position.y << std::endl;
  }

  return {padded_traj_points, pad_start_idx};
}

void EBPathSmoother::updateConstrain(const std::vector<TrajectoryPoint> & traj_points)
{
  const int num_points = eb_param_.num_sampling_points;

  Eigen::MatrixXd A = makeAMatrix(eb_param_.num_sampling_points);
  std::vector<double> upper_bound(num_points * 2, 0.0);
  std::vector<double> lower_bound(num_points * 2, 0.0);
  for (int i = 0; i < num_points; ++i) {
    const double rect_size =
      (i == 0) ? eb_param_.clearance_for_fixing : eb_param_.clearance_for_smoothing;

    const auto constrain =
      getConstrainLinesFromConstrainRectangle(traj_points.at(i).pose, rect_size);

    // constraint for x
    A(i, i) = constrain.x.coef(0);
    A(i, i + num_points) = constrain.x.coef(1);
    upper_bound[i] = constrain.x.upper_bound;
    lower_bound[i] = constrain.x.lower_bound;

    // constraint for y
    A(i + num_points, i) = constrain.y.coef(0);
    A(i + num_points, i + num_points) = constrain.y.coef(1);
    upper_bound[i + num_points] = constrain.x.upper_bound;
    lower_bound[i + num_points] = constrain.y.lower_bound;
  }

  // std::cerr << A << std::endl;
  osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
  osqp_solver_ptr_->updateA(A);
}

EBPathSmoother::ConstrainLines EBPathSmoother::getConstrainLinesFromConstrainRectangle(
  const geometry_msgs::msg::Pose & pose, const double rect_size)
{
  ConstrainLines constrain;
  const double theta = tf2::getYaw(pose.orientation);

  {  // x constrain
    constrain.x.coef = Eigen::Vector2d(std::sin(theta), -std::cos(theta));
    Eigen::Vector2d upper_point(
      pose.position.x - rect_size / 2.0 * std::sin(theta),
      pose.position.y + rect_size / 2.0 * std::cos(theta));
    Eigen::Vector2d lower_point(
      pose.position.x + rect_size / 2.0 * std::sin(theta),
      pose.position.y - rect_size / 2.0 * std::cos(theta));
    const double upper_bound = constrain.x.coef.transpose() * upper_point;
    const double lower_bound = constrain.x.coef.transpose() * lower_point;
    constrain.x.upper_bound = std::max(upper_bound, lower_bound);
    constrain.x.lower_bound = std::min(upper_bound, lower_bound);
  }

  {  // y constrain
    constrain.y.coef = Eigen::Vector2d(std::cos(theta), -std::sin(theta));
    Eigen::Vector2d upper_point(
      pose.position.x + rect_size / 2.0 * std::cos(theta),
      pose.position.y + rect_size / 2.0 * std::sin(theta));
    Eigen::Vector2d lower_point(
      pose.position.x - rect_size / 2.0 * std::cos(theta),
      pose.position.y - rect_size / 2.0 * std::sin(theta));
    const double upper_bound = constrain.y.coef.transpose() * upper_point;
    const double lower_bound = constrain.y.coef.transpose() * lower_point;
    constrain.y.upper_bound = std::max(upper_bound, lower_bound);
    constrain.y.lower_bound = std::min(upper_bound, lower_bound);
  }

  return constrain;
}

boost::optional<std::vector<double>> EBPathSmoother::optimizeTrajectory(
  const std::vector<TrajectoryPoint> & traj_points)
{
  debug_data_ptr_->tic(__func__);

  // update QP param
  const auto & qp_param = eb_param_.qp_param;
  osqp_solver_ptr_->updateEpsRel(qp_param.eps_rel);
  osqp_solver_ptr_->updateEpsAbs(qp_param.eps_abs);

  // solve QP
  const auto result = osqp_solver_ptr_->optimize();
  const auto optimized_points = std::get<0>(result);
  const auto status = std::get<3>(result);

  // check status
  if (status != 1) {
    osqp_solver_ptr_->logUnsolvedStatus("[EB]");
    return boost::none;
  }

  debug_data_ptr_->toc(__func__, "          ");
  return optimized_points;
}

// TODO(murooka): velocity
std::vector<TrajectoryPoint> EBPathSmoother::convertOptimizedPointsToTrajectory(
  const std::vector<double> optimized_points, const size_t pad_start_idx)
{
  std::vector<TrajectoryPoint> traj_points;

  // update position
  for (size_t i = 0; i < pad_start_idx; ++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = optimized_points.at(i);
    tmp_point.pose.position.y = optimized_points.at(i + eb_param_.num_sampling_points);
    traj_points.push_back(tmp_point);

    // std::cerr << tmp_point.pose.position.x << " " << tmp_point.pose.position.y << std::endl;
  }
  // std::cerr << optimized_points.size() << " " << pad_start_idx << std::endl;

  for (const auto & d : optimized_points) {
    // std::cerr << d << std::endl;
  }

  // update orientation
  motion_utils::insertOrientation(traj_points, true);
  /*
  for (size_t i = 0; i < pad_start_idx; ++i) {
    if (i == 0) {
      traj_points[i].pose.orientation = geometry_utils::getQuaternionFromPoints(
        traj_points[i + 1].pose.position, traj_points[i].pose.position);
    } else {
      traj_points[i].pose.orientation = geometry_utils::getQuaternionFromPoints(
        traj_points[i].pose.position, traj_points[i - 1].pose.position);
    }
  }
  */

  return traj_points;
}
}  // namespace collision_free_path_planner
