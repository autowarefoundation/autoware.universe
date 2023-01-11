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

#include "collision_free_path_planner/eb_path_optimizer.hpp"

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
Eigen::MatrixXd makePMatrix(const int num_points)
{
  // create P block matrix
  Eigen::MatrixXd P_block = Eigen::MatrixXd::Zero(num_points, num_points);
  for (int r = 0; r < num_points; ++r) {
    for (int c = 0; c < num_points; ++c) {
      if (r == c) {
        if (r == 0 || r == num_points - 1) {
          P_block(r, c) = 1.0;
        } else if (r == 1 || r == num_points - 2) {
          P_block(r, c) = 5.0;
        } else {
          P_block(r, c) = 6.0;
        }
      } else if (std::abs(c - r) == 1) {
        if (r == 0 || r == num_points - 1) {
          P_block(r, c) = -2.0;
        } else if (c == 0 || c == num_points - 1) {
          P_block(r, c) = -2.0;
        } else {
          P_block(r, c) = -4.0;
        }
      } else if (std::abs(c - r) == 2) {
        P_block(r, c) = 1.0;
      } else {
        P_block(r, c) = 0.0;
      }
    }
  }

  // create P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(num_points * 2, num_points * 2);
  P.block(0, 0, num_points, num_points) = P_block;
  P.block(num_points, num_points, num_points, num_points) = P_block;

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
EBPathOptimizer::EBPathOptimizer(
  rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
  const TrajectoryParam & traj_param, const std::shared_ptr<DebugData> debug_data_ptr)
: enable_debug_info_(enable_debug_info),
  ego_nearest_param_(ego_nearest_param),
  traj_param_(traj_param),
  debug_data_ptr_(debug_data_ptr)
{
  // eb param
  initializeEBParam(node);

  {  // osqp_solver_ptr_
    const auto & qp_param = eb_param_.qp_param;

    const Eigen::MatrixXd p = makePMatrix(eb_param_.num_sampling_points_for_eb);
    const Eigen::MatrixXd a = makeAMatrix(eb_param_.num_sampling_points_for_eb);

    const int num_points = eb_param_.num_sampling_points_for_eb;
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

void EBPathOptimizer::initializeEBParam(rclcpp::Node * node)
{
  eb_param_ = EBParam{};

  {  // common
    eb_param_.num_joint_buffer_points =
      node->declare_parameter<int>("advanced.eb.common.num_joint_buffer_points");
    eb_param_.num_offset_for_begin_idx =
      node->declare_parameter<int>("advanced.eb.common.num_offset_for_begin_idx");
    eb_param_.delta_arc_length_for_eb =
      node->declare_parameter<double>("advanced.eb.common.delta_arc_length_for_eb");
    eb_param_.num_sampling_points_for_eb =
      node->declare_parameter<int>("advanced.eb.common.num_sampling_points_for_eb");
  }

  {  // clearance
    eb_param_.clearance_for_straight_line =
      node->declare_parameter<double>("advanced.eb.clearance.clearance_for_straight_line");
    eb_param_.clearance_for_joint =
      node->declare_parameter<double>("advanced.eb.clearance.clearance_for_joint");
    eb_param_.clearance_for_only_smoothing =
      node->declare_parameter<double>("advanced.eb.clearance.clearance_for_only_smoothing");
  }

  {  // qp
    eb_param_.qp_param.max_iteration = node->declare_parameter<int>("advanced.eb.qp.max_iteration");
    eb_param_.qp_param.eps_abs = node->declare_parameter<double>("advanced.eb.qp.eps_abs");
    eb_param_.qp_param.eps_rel = node->declare_parameter<double>("advanced.eb.qp.eps_rel");
  }

  {  // other
    eb_param_.clearance_for_fixing = 0.0;
  }
}

void EBPathOptimizer::reset(const bool enable_debug_info, const TrajectoryParam & traj_param)
{
  enable_debug_info_ = enable_debug_info;
  traj_param_ = traj_param;
}

void EBPathOptimizer::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // common
    updateParam<int>(
      parameters, "advanced.eb.common.num_joint_buffer_points", eb_param_.num_joint_buffer_points);
    updateParam<int>(
      parameters, "advanced.eb.common.num_offset_for_begin_idx",
      eb_param_.num_offset_for_begin_idx);
    updateParam<double>(
      parameters, "advanced.eb.common.delta_arc_length_for_eb", eb_param_.delta_arc_length_for_eb);
    updateParam<int>(
      parameters, "advanced.eb.common.num_sampling_points_for_eb",
      eb_param_.num_sampling_points_for_eb);
  }

  {  // clearance
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_straight_line",
      eb_param_.clearance_for_straight_line);
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_joint", eb_param_.clearance_for_joint);
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_only_smoothing",
      eb_param_.clearance_for_only_smoothing);
  }

  {  // qp
    updateParam<int>(parameters, "advanced.eb.qp.max_iteration", eb_param_.qp_param.max_iteration);
    updateParam<double>(parameters, "advanced.eb.qp.eps_abs", eb_param_.qp_param.eps_abs);
    updateParam<double>(parameters, "advanced.eb.qp.eps_rel", eb_param_.qp_param.eps_rel);
  }
}

boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathOptimizer::getEBTrajectory(
  const PlannerData & planner_data,
  const std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj)
{
  stop_watch_.tic(__func__);

  const auto & p = planner_data;
  const auto & path = p.path;
  const auto & ego_pose = p.ego_pose;
  const double ego_vel = p.ego_vel;

  // current_ego_vel_ = ego_vel;

  /*
  // get candidate points for optimization
  // decide fix or non fix, might not required only for smoothing purpose
  const CandidatePoints candidate_points =
    getCandidatePoints(ego_pose, path.points, prev_eb_traj);
  if (candidate_points.fixed_points.empty() && candidate_points.non_fixed_points.empty()) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathOptimizer"), enable_debug_info_,
      "return boost::none since empty candidate points");
    return boost::none;
  }
  */

  // get optimized smooth points with elastic band
  const auto eb_traj_points_opt = getOptimizedTrajectory(ego_pose, path);
  if (!eb_traj_points_opt) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathOptimizer"), enable_debug_info_,
      "return boost::none since smoothing failed");
    return boost::none;
  }
  const auto eb_traj_points = eb_traj_points_opt.get();

  debug_data_ptr_->eb_traj = eb_traj_points;

  {  // publish eb trajectory
    const auto eb_traj = trajectory_utils::createTrajectory(p.path.header, eb_traj_points);
    debug_eb_traj_pub_->publish(eb_traj);
  }

  debug_data_ptr_->msg_stream << "      " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return eb_traj_points;
}

boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathOptimizer::getOptimizedTrajectory(
  const geometry_msgs::msg::Pose & ego_pose, const Path & path)
{
  stop_watch_.tic(__func__);

  // make function: trimPathPoints

  const double forward_distance = 10;  // 50;
  // TODO(murooka)    traj_param_.num_sampling_points * mpt_param_.delta_arc_length;
  const double backward_distance = 10;  // traj_param_.output_backward_traj_length;

  const double tmp_margin = 0;  // 20.0;

  // crop path
  const size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(path.points, ego_pose, ego_nearest_param_);
  const auto cropped_path_points = trajectory_utils::cropPoints(
    path.points, ego_pose.position, ego_seg_idx, forward_distance + tmp_margin,
    -backward_distance - tmp_margin);

  // interpolate points for logic purpose
  Path cropped_path;
  cropped_path.points = cropped_path_points;
  const auto resampled_path =
    motion_utils::resamplePath(cropped_path, eb_param_.delta_arc_length_for_eb);

  // pad path points
  const auto [padded_path_points, pad_start_idx] = getPaddedPathPoints(resampled_path.points);

  // calculate rectangle size vector
  const size_t num_fixed_points = 1;
  const auto rect_size_vec = getRectangleSizeVector(padded_path_points, num_fixed_points);

  // update constrain for elastic band's QP
  updateConstrain(padded_path_points, rect_size_vec);

  // optimize trajectory by elastic band
  const auto optimized_points = optimizeTrajectory(padded_path_points);
  if (!optimized_points) {
    return boost::none;
  }

  // convert to trajectory
  const auto traj_points =
    convertOptimizedPointsToTrajectory(optimized_points.get(), pad_start_idx);

  debug_data_ptr_->msg_stream << "        " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return traj_points;
}

std::tuple<std::vector<PathPoint>, size_t> EBPathOptimizer::getPaddedPathPoints(
  const std::vector<PathPoint> & path_points)
{
  const size_t pad_start_idx =
    std::min(static_cast<size_t>(eb_param_.num_sampling_points_for_eb), path_points.size());
  std::cerr << path_points.size() << std::endl;
  std::cerr << pad_start_idx << std::endl;

  std::vector<PathPoint> padded_path_points;
  for (int i = 0; i < eb_param_.num_sampling_points_for_eb; ++i) {
    const size_t point_idx = i < pad_start_idx ? i : pad_start_idx - 1;
    padded_path_points.push_back(path_points.at(point_idx));

    std::cerr << padded_path_points.back().pose.position.x << " "
              << padded_path_points.back().pose.position.y << std::endl;
  }

  return {padded_path_points, pad_start_idx};
}

std::vector<double> EBPathOptimizer::getRectangleSizeVector(
  const std::vector<PathPoint> & path_points, const int num_fixed_points)
{
  std::vector<double> rect_size_vec;
  for (int i = 0; i < eb_param_.num_sampling_points_for_eb; ++i) {
    if (i < num_fixed_points) {
      rect_size_vec.push_back(eb_param_.clearance_for_fixing);
    } else {
      rect_size_vec.push_back(eb_param_.clearance_for_only_smoothing);
    }
  }

  return rect_size_vec;
}

void EBPathOptimizer::updateConstrain(
  const std::vector<PathPoint> & path_points, const std::vector<double> & rect_size_vec)
{
  const int num_points = eb_param_.num_sampling_points_for_eb;

  Eigen::MatrixXd A = makeAMatrix(eb_param_.num_sampling_points_for_eb);
  std::vector<double> lower_bound(num_points * 2, 0.0);
  std::vector<double> upper_bound(num_points * 2, 0.0);
  for (int i = 0; i < num_points; ++i) {
    const auto constrain =
      getConstrainLinesFromConstrainRectangle(path_points.at(i).pose, rect_size_vec.at(i));
    // constraint for x
    A(i, i) = constrain.x.coef(0);
    A(i, i + num_points) = constrain.x.coef(1);
    lower_bound[i] = constrain.x.lower_bound;
    upper_bound[i] = constrain.x.upper_bound;
    // constraint for y
    A(i + num_points, i) = constrain.y.coef(0);
    A(i + num_points, i + num_points) = constrain.y.coef(1);
    lower_bound[i + num_points] = constrain.y.lower_bound;
    upper_bound[i + num_points] = constrain.x.upper_bound;

    std::cerr << "[" << constrain.x.lower_bound << "," << constrain.x.upper_bound << "], ["
              << constrain.y.lower_bound << "," << constrain.x.upper_bound << "]" << std::endl;
  }

  std::cerr << A << std::endl;
  osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
  osqp_solver_ptr_->updateA(A);
}

EBPathOptimizer::ConstrainLines EBPathOptimizer::getConstrainLinesFromConstrainRectangle(
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

boost::optional<std::vector<double>> EBPathOptimizer::optimizeTrajectory(
  const std::vector<PathPoint> & path_points)
{
  stop_watch_.tic(__func__);

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

  debug_data_ptr_->msg_stream << "          " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";

  return optimized_points;
}

// TODO(murooka): velocity
std::vector<TrajectoryPoint> EBPathOptimizer::convertOptimizedPointsToTrajectory(
  const std::vector<double> optimized_points, const size_t pad_start_idx)
{
  std::vector<TrajectoryPoint> traj_points;

  // update position
  for (size_t i = 0; i < pad_start_idx; ++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = optimized_points.at(i);
    tmp_point.pose.position.y = optimized_points.at(i + eb_param_.num_sampling_points_for_eb);
    traj_points.push_back(tmp_point);

    // std::cerr << tmp_point.pose.position.x << " " << tmp_point.pose.position.y << std::endl;
  }
  // std::cerr << optimized_points.size() << " " << pad_start_idx << std::endl;

  for (const auto & d : optimized_points) {
    std::cerr << d << std::endl;
  }

  // update orientationn
  for (size_t i = 0; i < pad_start_idx; ++i) {
    if (i == 0) {
      traj_points[i].pose.orientation = geometry_utils::getQuaternionFromPoints(
        traj_points[i + 1].pose.position, traj_points[i].pose.position);
    } else {
      traj_points[i].pose.orientation = geometry_utils::getQuaternionFromPoints(
        traj_points[i].pose.position, traj_points[i - 1].pose.position);
    }
  }

  return traj_points;
}

// std::vector<geometry_msgs::msg::Pose> EBPathOptimizer::getFixedPoints(
//   const geometry_msgs::msg::Pose & ego_pose,
//   [[maybe_unused]] const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
//   const std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj)
// {
//   /* use of prev_traj_points(fine resolution) instead of prev_opt_traj(coarse resolution)
//      stabilize trajectory's yaw*/
//   if (prev_eb_traj) {
//     if (prev_eb_traj->empty()) {
//       std::vector<geometry_msgs::msg::Pose> empty_points;
//       return empty_points;
//     }
//     const size_t begin_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
//       prev_eb_traj, ego_pose, traj_param_.ego_nearest_dist_threshold,
//       traj_param_.ego_nearest_yaw_threshold);
//     const int backward_fixing_idx = std::max(
//       static_cast<int>(
//         begin_idx -
//         traj_param_.output_backward_traj_length / traj_param_.output_delta_arc_length),
//       0);
//
//     // NOTE: Fixed length of EB has to be longer than that of MPT.
//     constexpr double forward_fixed_length_margin = 5.0;
//     const double forward_fixed_length = std::max(
//       current_ego_vel_ * traj_param_.forward_fixing_min_time + forward_fixed_length_margin,
//       traj_param_.forward_fixing_min_distance);
//
//     const int forward_fixing_idx = std::min(
//       static_cast<int>(
//         begin_idx + forward_fixed_length / traj_param_.output_delta_arc_length),
//       static_cast<int>(prev_eb_traj->size() - 1));
//     std::vector<geometry_msgs::msg::Pose> fixed_points;
//     for (int i = backward_fixing_idx; i <= forward_fixing_idx; i++) {
//       fixed_points.push_back(prev_eb_traj->at(i).pose);
//     }
//     return fixed_points;
//   } else {
//     std::vector<geometry_msgs::msg::Pose> empty_points;
//     return empty_points;
//   }
// }
//
// EBPathOptimizer::CandidatePoints EBPathOptimizer::getCandidatePoints(
//   const geometry_msgs::msg::Pose & ego_pose,
//   const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
//   const std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj)
// {
//   const std::vector<geometry_msgs::msg::Pose> fixed_points =
//     getFixedPoints(ego_pose, path_points, prev_eb_traj);
//   if (fixed_points.empty()) {
//     CandidatePoints candidate_points = getDefaultCandidatePoints(path_points);
//     return candidate_points;
//   }
//
//   // try to find non-fix points
//   const auto opt_begin_idx = motion_utils::findNearestIndex(
//     path_points, fixed_points.back(), std::numeric_limits<double>::max(),
//     traj_param_.delta_yaw_threshold_for_closest_point);
//   if (!opt_begin_idx) {
//     CandidatePoints candidate_points;
//     candidate_points.fixed_points = fixed_points;
//     candidate_points.begin_path_idx = path_points.size();
//     candidate_points.end_path_idx = path_points.size();
//     return candidate_points;
//   }
//
//   const int begin_idx = std::min(
//     static_cast<int>(opt_begin_idx.get()) + eb_param_.num_offset_for_begin_idx,
//     static_cast<int>(path_points.size()) - 1);
//
//   std::vector<geometry_msgs::msg::Pose> non_fixed_points;
//   for (size_t i = begin_idx; i < path_points.size(); i++) {
//     non_fixed_points.push_back(path_points[i].pose);
//   }
//   CandidatePoints candidate_points;
//   candidate_points.fixed_points = fixed_points;
//   candidate_points.non_fixed_points = non_fixed_points;
//   candidate_points.begin_path_idx = begin_idx;
//   candidate_points.end_path_idx = path_points.size() - 1;
//
//   debug_data_ptr_->fixed_points = candidate_points.fixed_points;
//   debug_data_ptr_->non_fixed_points = candidate_points.non_fixed_points;
//   return candidate_points;
// }
//
// EBPathOptimizer::CandidatePoints EBPathOptimizer::getDefaultCandidatePoints(
//   const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points)
// {
//   double accum_arc_length = 0;
//   int end_path_idx = 0;
//   std::vector<geometry_msgs::msg::Pose> fixed_points;
//   for (size_t i = 0; i < path_points.size(); i++) {
//     if (i > 0) {
//       accum_arc_length += tier4_autoware_utils::calcDistance2d(
//         path_points[i].pose.position, path_points[i - 1].pose.position);
//     }
//     if (
//       accum_arc_length > eb_param_.num_sampling_points_for_eb *
//       eb_param_.delta_arc_length_for_eb) { break;
//     }
//     end_path_idx = i;
//     fixed_points.push_back(path_points[i].pose);
//   }
//   CandidatePoints candidate_points;
//   candidate_points.fixed_points = fixed_points;
//   candidate_points.begin_path_idx = 0;
//   candidate_points.end_path_idx = end_path_idx;
//   return candidate_points;
// }
//
// int EBPathOptimizer::getNumFixedPoints(
//   const std::vector<geometry_msgs::msg::Pose> & fixed_points,
//   const std::vector<geometry_msgs::msg::Pose> & interpolated_points, const int farthest_idx)
// {
//   int num_fixed_points = 0;
//   if (!fixed_points.empty() && !interpolated_points.empty()) {
//     std::vector<geometry_msgs::msg::Pose> interpolated_points =
//     motion_utils::resamplePath(fixed_points, eb_param_.delta_arc_length_for_eb); std::cerr <<
//     "PO2 " << interpolated_points.size() << std::endl; num_fixed_points =
//     interpolated_points.size();
//   }
//   num_fixed_points = std::min(num_fixed_points, farthest_idx);
//   return num_fixed_points;
// }
}  // namespace collision_free_path_planner
