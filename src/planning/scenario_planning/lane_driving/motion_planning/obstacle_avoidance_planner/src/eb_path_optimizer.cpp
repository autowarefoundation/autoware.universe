/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/MapMetaData.h>

#include <ros/console.h>
#include <tf2/utils.h>

#include <osqp_interface/osqp_interface.h>

#include "obstacle_avoidance_planner/eb_path_optimizer.h"
#include "obstacle_avoidance_planner/process_cv.h"
#include "obstacle_avoidance_planner/util.h"

EBPathOptimizer::EBPathOptimizer(
  const bool is_showing_debug_info, const QPParam qp_param, const TrajectoryParam traj_pram,
  const ConstrainParam constrain_param)
: is_showing_debug_info_(is_showing_debug_info),
  epsilon_(1e-8),
  qp_param_(qp_param),
  traj_param_(traj_pram),
  constrain_param_(constrain_param)
{
  geometry_msgs::Vector3 keep_space_shape;
  keep_space_shape.x = constrain_param_.keep_space_shape_x;
  keep_space_shape.y = constrain_param_.keep_space_shape_y;
  keep_space_shape_ptr_ = std::make_unique<geometry_msgs::Vector3>(keep_space_shape);
  initializeSolver();
}

EBPathOptimizer::~EBPathOptimizer() {}

void EBPathOptimizer::initializeSolver()
{
  Eigen::MatrixXd p = makePMatrix();
  default_a_matrix_ = makeAMatrix();

  std::vector<double> q(traj_param_.num_sampling_points * 2, 0.0);
  std::vector<double> lower_bound(traj_param_.num_sampling_points * 2, 0.0);
  std::vector<double> upper_bound(traj_param_.num_sampling_points * 2, 0.0);
  osqp_solver_ptr_ = std::make_unique<osqp::OSQPInterface>(
    p, default_a_matrix_, q, lower_bound, upper_bound, qp_param_.eps_abs);
  osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel);
  osqp_solver_ptr_->updateMaxIter(qp_param_.max_iteration);
}

/* make positive semidefinite matrix for objective function
   reference: https://ieeexplore.ieee.org/document/7402333 */
Eigen::MatrixXd EBPathOptimizer::makePMatrix()
{
  Eigen::MatrixXd P =
    Eigen::MatrixXd::Zero(traj_param_.num_sampling_points * 2, traj_param_.num_sampling_points * 2);
  for (int r = 0; r < traj_param_.num_sampling_points * 2; r++) {
    for (int c = 0; c < traj_param_.num_sampling_points * 2; c++) {
      if (r == c) {
        P(r, c) = 6;
      } else if (std::abs(c - r) == 1) {
        P(r, c) = -4;
      } else if (std::abs(c - r) == 2) {
        P(r, c) = 1;
      } else {
        P(r, c) = 0;
      }
    }
  }
  return P;
}

// make default linear constrain matrix
Eigen::MatrixXd EBPathOptimizer::makeAMatrix()
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(
    traj_param_.num_sampling_points * 2, traj_param_.num_sampling_points * 2);
  for (int i = 0; i < traj_param_.num_sampling_points * 2; i++) {
    if (i < traj_param_.num_sampling_points) {
      A(i, i + traj_param_.num_sampling_points) = 1;
    } else {
      A(i, i - traj_param_.num_sampling_points) = 1;
    }
  }
  return A;
}

std::vector<autoware_planning_msgs::TrajectoryPoint> EBPathOptimizer::generateOptimizedTrajectory(
  const bool enable_avoidance, const geometry_msgs::Pose & ego_pose,
  const autoware_planning_msgs::Path & path,
  const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> &
    prev_optimized_points,
  const std::vector<autoware_perception_msgs::DynamicObject> & objects, DebugData & debug_data)
{
  // processing drivable area
  auto t_start1 = std::chrono::high_resolution_clock::now();
  cv::Mat drivable_area = process_cv::getDrivableAreaInCV(path.drivable_area);
  cv::Mat clearance_map = process_cv::getClearanceMap(path.drivable_area, drivable_area, objects);
  cv::Mat only_objects_clearance_map = process_cv::getOnlyObjectsClearanceMap(
    clearance_map, objects, path.drivable_area, constrain_param_.max_avoiding_objects_velocity_ms);
  debug_data.clearance_map = clearance_map;
  debug_data.only_object_clearance_map = only_objects_clearance_map;
  auto t_end1 = std::chrono::high_resolution_clock::now();
  float elapsed_ms1 = std::chrono::duration<float, std::milli>(t_end1 - t_start1).count();
  ROS_INFO_COND(is_showing_debug_info_, "Processing driveable area time: = %f [ms]", elapsed_ms1);

  // get candidate points for optimization
  CandidatePoints candidate_points = getCandidatePoints(
    ego_pose, path.points, prev_optimized_points, drivable_area, path.drivable_area.info);
  debug_data.fixed_points = candidate_points.fixed_points;
  debug_data.non_fixed_points = candidate_points.non_fixed_points;
  if (candidate_points.fixed_points.empty() && candidate_points.non_fixed_points.empty()) {
    std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points =
      util::convertPathToTrajectory(path.points);
    return traj_points;
  }

  // get optimized smooth points
  std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points = getOptimizedTrajectory(
    enable_avoidance, path, candidate_points, clearance_map, only_objects_clearance_map,
    debug_data);

  // extend trajectory when trajectory length is less than trajectory_length param
  auto t_start2 = std::chrono::high_resolution_clock::now();
  std::vector<autoware_planning_msgs::TrajectoryPoint> extended_traj_points =
    getExtendedOptimizedTrajectory(path.points, traj_points, candidate_points);
  auto t_end2 = std::chrono::high_resolution_clock::now();
  float elapsed_ms2 = std::chrono::duration<float, std::milli>(t_end2 - t_start2).count();
  ROS_INFO_COND(is_showing_debug_info_, "Extending trajectory time: = %f [ms]", elapsed_ms2);
  return extended_traj_points;
}

std::vector<autoware_planning_msgs::TrajectoryPoint> EBPathOptimizer::getOptimizedTrajectory(
  const bool enable_avoidance, const autoware_planning_msgs::Path & path,
  const CandidatePoints & candidate_points, const cv::Mat & clearance_map,
  const cv::Mat & only_objects_clearance_map, DebugData & debug_data)
{
  // get constrain rectablges around each point
  std::vector<geometry_msgs::Point> interpolated_points = util::getInterpolatedPoints(
    candidate_points.fixed_points, candidate_points.non_fixed_points,
    traj_param_.delta_arc_length_for_optimization);
  debug_data.interpolated_points = interpolated_points;
  const int farrest_idx =
    std::min((int)(traj_param_.num_sampling_points - 1), (int)(interpolated_points.size() - 1));
  const int num_fixed_points =
    getNumFixiedPoints(candidate_points.fixed_points, interpolated_points, farrest_idx);
  const int straight_line_idx = getStraightLineIdx(
    interpolated_points, farrest_idx, only_objects_clearance_map, path.drivable_area.info,
    debug_data.straight_points);
  std::vector<geometry_msgs::Point> padded_interpolated_points =
    getPaddedInterpolatedPoints(interpolated_points, farrest_idx);
  auto t_start1 = std::chrono::high_resolution_clock::now();
  std::vector<ConstrainRectangle> rectangles = getConstrainRectangleVec(
    enable_avoidance, path, padded_interpolated_points, num_fixed_points, farrest_idx,
    straight_line_idx, clearance_map, only_objects_clearance_map);
  debug_data.constrain_rectangles = rectangles;
  auto t_end1 = std::chrono::high_resolution_clock::now();
  float elapsed_ms1 = std::chrono::duration<float, std::milli>(t_end1 - t_start1).count();
  ROS_INFO_COND(is_showing_debug_info_, "Make constrain rectangle time: = %f [ms]", elapsed_ms1);

  // update constrain for QP based on constrain rectangles
  updateConstrain(padded_interpolated_points, rectangles);

  // solve QP and get optimized trajectory
  auto t_start2 = std::chrono::high_resolution_clock::now();
  const bool is_extending = false;
  std::vector<double> optimized_points = solveQP(is_extending);
  auto t_end2 = std::chrono::high_resolution_clock::now();
  float elapsed_ms2 = std::chrono::duration<float, std::milli>(t_end2 - t_start2).count();
  ROS_INFO_COND(is_showing_debug_info_, "Optimization time: = %f [ms]", elapsed_ms2);
  const auto traj_points = convertOptimizedPointsToTrajectory(optimized_points, farrest_idx);
  return traj_points;
}

std::vector<autoware_planning_msgs::TrajectoryPoint>
EBPathOptimizer::getExtendedOptimizedTrajectory(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points,
  const CandidatePoints & candidate_points)
{
  const double accum_arc_length = getArcLength(optimized_points);
  if (
    accum_arc_length > traj_param_.trajectory_length ||
    candidate_points.end_path_idx == path_points.size() ||
    optimized_points.size() <= traj_param_.num_fix_points_for_extending) {
    return optimized_points;
  }
  const double extending_trajectory_length = traj_param_.trajectory_length - accum_arc_length;
  const int end_extented_path_idx =
    getEndPathIdx(path_points, candidate_points.end_path_idx, extending_trajectory_length);
  std::vector<geometry_msgs::Pose> non_fixed_points;
  for (int i = candidate_points.end_path_idx; i <= end_extented_path_idx; i++) {
    non_fixed_points.push_back(path_points[i].pose);
  }

  std::vector<geometry_msgs::Pose> fixed_poitns;
  for (int i = traj_param_.num_fix_points_for_extending; i > 0; i--) {
    fixed_poitns.push_back(optimized_points[optimized_points.size() - i].pose);
  }

  const double delat_arc_length_for_extending_trajectory = std::fmax(
    traj_param_.delta_arc_length_for_optimization,
    (float)(extending_trajectory_length / traj_param_.num_sampling_points));
  std::vector<geometry_msgs::Point> interpolated_points = util::getInterpolatedPoints(
    fixed_poitns, non_fixed_points, delat_arc_length_for_extending_trajectory);
  const int farrest_idx =
    std::min((int)(traj_param_.num_sampling_points - 1), (int)(interpolated_points.size() - 1));
  std::vector<geometry_msgs::Point> padded_interpolated_points =
    getPaddedInterpolatedPoints(interpolated_points, farrest_idx);

  std::vector<ConstrainRectangle> constrain_rectangles = getConstrainRectangleVec(
    path_points, padded_interpolated_points, traj_param_.num_fix_points_for_extending, farrest_idx);

  updateConstrain(padded_interpolated_points, constrain_rectangles);
  const bool is_extending = true;
  std::vector<double> extended_optimized_points = solveQP(is_extending);

  const auto extended_traj_points =
    convertOptimizedPointsToTrajectory(extended_optimized_points, farrest_idx);
  auto merged_points = optimized_points;
  for (int i = traj_param_.num_fix_points_for_extending; i < extended_traj_points.size(); i++) {
    merged_points.push_back(extended_traj_points[i]);
  }
  return merged_points;
}

std::vector<double> EBPathOptimizer::solveQP(const bool is_extending)
{
  double eps_abs = 0;
  double eps_rel = 0;
  if (is_extending) {
    eps_abs = qp_param_.eps_abs_for_extending;
    eps_rel = qp_param_.eps_rel_for_extending;
  } else {
    eps_abs = qp_param_.eps_abs;
    eps_rel = qp_param_.eps_rel;
  }
  osqp_solver_ptr_->updateEpsAbs(eps_abs);
  osqp_solver_ptr_->updateEpsRel(eps_rel);
  std::tuple<std::vector<double>, std::vector<double>, int> result = osqp_solver_ptr_->optimize();
  std::vector<double> optimized_points = std::get<0>(result);
  return optimized_points;
}

std::vector<geometry_msgs::Pose> EBPathOptimizer::getFixedPoints(
  const geometry_msgs::Pose & ego_pose,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> &
    prev_optimized_points,
  const cv::Mat & drivable_area, const nav_msgs::MapMetaData & map_info)
{
  if (prev_optimized_points) {
    if (prev_optimized_points->empty()) {
      std::vector<geometry_msgs::Pose> empty_points;
      return empty_points;
    }
    const int default_idx = 0;
    int begin_idx = util::getNearestIdx(
      *prev_optimized_points, ego_pose, default_idx,
      traj_param_.delta_yaw_threshold_for_closest_point);
    if (!isPointInsideDrivableArea(
          prev_optimized_points->at(begin_idx).pose.position, drivable_area, map_info)) {
      begin_idx = default_idx;
    }
    const int backward_fixing_idx = std::max(
      (int)(begin_idx - traj_param_.backward_fixing_distance / traj_param_.delta_arc_length_for_trajectory),
      0);
    const int forward_fixing_idx = std::min(
      (int)(begin_idx + traj_param_.forward_fixing_distance / traj_param_.delta_arc_length_for_trajectory),
      (int)(prev_optimized_points->size() - 1));
    std::vector<geometry_msgs::Pose> fixed_points;
    for (int i = backward_fixing_idx; i <= forward_fixing_idx; i++) {
      fixed_points.push_back(prev_optimized_points->at(i).pose);
    }
    return fixed_points;
  } else {
    std::vector<geometry_msgs::Pose> empty_points;
    return empty_points;
  }
}

CandidatePoints EBPathOptimizer::getCandidatePoints(
  const geometry_msgs::Pose & ego_pose,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> &
    prev_optimized_points,
  const cv::Mat & drivable_area, const nav_msgs::MapMetaData & map_info)
{
  std::vector<geometry_msgs::Pose> fixed_points =
    getFixedPoints(ego_pose, path_points, prev_optimized_points, drivable_area, map_info);
  if (fixed_points.empty()) {
    CandidatePoints candidate_points = getDefaultCandidatePoints(path_points);
    return candidate_points;
  }
  const int default_idx = -1;
  int begin_idx = util::getNearestIdx(
    path_points, fixed_points.back(), default_idx,
    traj_param_.delta_yaw_threshold_for_closest_point);
  if (begin_idx == default_idx) {
    CandidatePoints candidate_points;
    candidate_points.fixed_points = fixed_points;
    candidate_points.begin_path_idx = path_points.size();
    candidate_points.end_path_idx = path_points.size();
    return candidate_points;
  }
  begin_idx =
    std::min((int)begin_idx + traj_param_.num_offset_for_begin_idx, (int)path_points.size());

  if (!isPointInsideDrivableArea(path_points[begin_idx].pose.position, drivable_area, map_info)) {
    CandidatePoints candidate_points = getDefaultCandidatePoints(path_points);
    return candidate_points;
  }

  const double required_trajectory_length =
    traj_param_.num_sampling_points * traj_param_.delta_arc_length_for_optimization -
    traj_param_.forward_fixing_distance - traj_param_.backward_fixing_distance;
  const int end_path_idx = getEndPathIdx(path_points, begin_idx, required_trajectory_length);

  const int end_path_idx_inside_drivable_area =
    getEndPathIdxInsideArea(path_points, begin_idx, end_path_idx, drivable_area, map_info);
  std::vector<geometry_msgs::Pose> non_fixed_points;
  for (size_t i = begin_idx; i < end_path_idx_inside_drivable_area; i++) {
    non_fixed_points.push_back(path_points[i].pose);
  }
  CandidatePoints candidate_points;
  candidate_points.fixed_points = fixed_points;
  candidate_points.non_fixed_points = non_fixed_points;
  candidate_points.begin_path_idx = begin_idx;
  candidate_points.end_path_idx = end_path_idx_inside_drivable_area;
  return candidate_points;
}

std::vector<geometry_msgs::Point> EBPathOptimizer::getPaddedInterpolatedPoints(
  const std::vector<geometry_msgs::Point> & interpolated_points, const int farrest_point_idx)
{
  std::vector<geometry_msgs::Point> padded_interpolated_points;
  for (size_t i = 0; i < traj_param_.num_sampling_points; i++) {
    if (i > farrest_point_idx) {
      padded_interpolated_points.push_back(interpolated_points[farrest_point_idx]);
    } else {
      padded_interpolated_points.push_back(interpolated_points[i]);
    }
  }
  return padded_interpolated_points;
}

CandidatePoints EBPathOptimizer::getDefaultCandidatePoints(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points)
{
  double accum_arc_length = 0;
  int end_path_idx = 0;
  std::vector<geometry_msgs::Pose> fixed_points;
  for (size_t i = 0; i < path_points.size(); i++) {
    if (i > 0) {
      accum_arc_length +=
        util::calculate2DDistance(path_points[i].pose.position, path_points[i - 1].pose.position);
    }
    if (
      accum_arc_length >
      traj_param_.num_sampling_points * traj_param_.delta_arc_length_for_optimization) {
      break;
    }
    end_path_idx = i;
    fixed_points.push_back(path_points[i].pose);
  }
  CandidatePoints candidate_points;
  candidate_points.fixed_points = fixed_points;
  candidate_points.begin_path_idx = 0;
  candidate_points.end_path_idx = end_path_idx;
  return candidate_points;
}

bool EBPathOptimizer::isPointInsideDrivableArea(
  const geometry_msgs::Point & point, const cv::Mat & drivable_area,
  const nav_msgs::MapMetaData & map_info)
{
  bool is_inside = true;
  std::shared_ptr<geometry_msgs::Point> image_ptr = util::transformMapToImagePtr(point, map_info);
  unsigned char occupancy_value = std::numeric_limits<unsigned char>::max();
  if (image_ptr) {
    occupancy_value = drivable_area.ptr<unsigned char>((int)image_ptr->y)[(int)image_ptr->x];
  }
  if (image_ptr == nullptr || occupancy_value < epsilon_) {
    is_inside = false;
  }
  return is_inside;
}

int EBPathOptimizer::getEndPathIdx(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points, const int begin_path_idx,
  const double required_trajectory_length)
{
  double accum_dist = 0;
  int end_path_idx = begin_path_idx;
  for (int i = begin_path_idx; i < path_points.size(); i++) {
    if (i > begin_path_idx) {
      const double dist =
        util::calculate2DDistance(path_points[i].pose.position, path_points[i - 1].pose.position);
      accum_dist += dist;
    }
    end_path_idx = i;
    if (accum_dist > required_trajectory_length) {
      break;
    }
  }
  return end_path_idx;
}

int EBPathOptimizer::getEndPathIdxInsideArea(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points, const int begin_path_idx,
  const int end_path_idx, const cv::Mat & drivable_area, const nav_msgs::MapMetaData & map_info)
{
  int end_path_idx_inside_drivable_area = begin_path_idx;
  for (int i = begin_path_idx; i <= end_path_idx; i++) {
    geometry_msgs::Point rel_top_point;
    rel_top_point.x = keep_space_shape_ptr_->x;
    rel_top_point.y = 0;
    geometry_msgs::Point abs_top_point =
      util::transformToAbsoluteCoordinate2D(rel_top_point, path_points[i].pose);
    geometry_msgs::Point top_image_point;
    if (util::transformMapToImage(abs_top_point, map_info, top_image_point)) {
      const unsigned char top_occupancy_value =
        drivable_area.ptr<unsigned char>((int)top_image_point.y)[(int)top_image_point.x];
      if (top_occupancy_value < epsilon_) {
        break;
      }
      end_path_idx_inside_drivable_area = i;
    } else {
      break;
    }
  }
  return end_path_idx_inside_drivable_area;
}

double EBPathOptimizer::getArcLength(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points)
{
  double accum_arc_length = 0;
  for (size_t i = 0; i < optimized_points.size(); i++) {
    if (i > 0) {
      const double dist = util::calculate2DDistance(
        optimized_points[i].pose.position, optimized_points[i - 1].pose.position);
      accum_arc_length += dist;
    }
  }
  return accum_arc_length;
}

int EBPathOptimizer::getNumFixiedPoints(
  const std::vector<geometry_msgs::Pose> & fixed_points,
  const std::vector<geometry_msgs::Point> & interpolated_points, const int farrest_idx)
{
  int num_fixed_points = 0;
  if (!fixed_points.empty() && !interpolated_points.empty()) {
    std::vector<geometry_msgs::Point> interpolated_points =
      util::getInterpolatedPoints(fixed_points, traj_param_.delta_arc_length_for_optimization);
    num_fixed_points = interpolated_points.size();
  }
  num_fixed_points = std::min(num_fixed_points, farrest_idx);
  return num_fixed_points;
}

std::vector<autoware_planning_msgs::TrajectoryPoint>
EBPathOptimizer::convertOptimizedPointsToTrajectory(
  const std::vector<double> optimized_points, const int farrest_idx)
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points;
  for (size_t i = 0; i <= farrest_idx; i++) {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = optimized_points[i];
    tmp_point.pose.position.y = optimized_points[i + traj_param_.num_sampling_points];
    traj_points.push_back(tmp_point);
  }
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (i > 0) {
      traj_points[i].pose.orientation = util::getQuaternionFromPoints(
        traj_points[i].pose.position, traj_points[i - 1].pose.position);
    } else if (i == 0 && traj_points.size() > 1) {
      traj_points[i].pose.orientation = util::getQuaternionFromPoints(
        traj_points[i + 1].pose.position, traj_points[i].pose.position);
    }
  }
  return traj_points;
}

std::vector<ConstrainRectangle> EBPathOptimizer::getConstrainRectangleVec(
  const bool enable_avoidance, const autoware_planning_msgs::Path & path,
  const std::vector<geometry_msgs::Point> & interpolated_points, const int num_fixed_points,
  const int farrest_point_idx, const int straight_idx, const cv::Mat & clearance_map,
  const cv::Mat & only_objects_clearance_map)
{
  std::vector<ConstrainRectangle> object_road_constrain_ranges(traj_param_.num_sampling_points);
  std::vector<ConstrainRectangle> road_constrain_ranges(traj_param_.num_sampling_points);
  std::vector<ConstrainRectangle> only_smooth_constrain_ranges(traj_param_.num_sampling_points);
  for (int i = 0; i < traj_param_.num_sampling_points; i++) {
    geometry_msgs::Pose origin_pose = getOriginPose(interpolated_points, i, path.points);
    if (i == 0 || i == 1 || i >= farrest_point_idx - 1 || i < num_fixed_points - 1) {
      ConstrainRectangle rectangle =
        getConstrainRectangle(origin_pose, constrain_param_.clearance_for_fixing);
      object_road_constrain_ranges[i] = rectangle;
      road_constrain_ranges[i] = rectangle;
      only_smooth_constrain_ranges[i] = rectangle;
    } else if (
      i >= num_fixed_points - traj_param_.num_joint_buffer_points &&
      i <= num_fixed_points + traj_param_.num_joint_buffer_points) {
      ConstrainRectangle rectangle =
        getConstrainRectangle(path.points, origin_pose, clearance_map, path.drivable_area.info);
      object_road_constrain_ranges[i] = rectangle;
      road_constrain_ranges[i] = rectangle;
      only_smooth_constrain_ranges[i] = rectangle;
    } else if (i >= straight_idx) {
      ConstrainRectangle rectangle =
        getConstrainRectangle(origin_pose, constrain_param_.clearance_for_straight_line);
      object_road_constrain_ranges[i] = rectangle;
      road_constrain_ranges[i] = rectangle;
      only_smooth_constrain_ranges[i] = rectangle;
    } else {
      ConstrainRectangles constrain_rectangles = getConstrainRectangles(
        origin_pose, clearance_map, only_objects_clearance_map, path.drivable_area.info);
      object_road_constrain_ranges[i] = constrain_rectangles.object_constrain_rectangle;
      road_constrain_ranges[i] = constrain_rectangles.road_constrain_rectangle;
      only_smooth_constrain_ranges[i] =
        getConstrainRectangle(origin_pose, constrain_param_.clearance_for_only_smoothing);
    }
  }
  std::vector<ConstrainRectangle> constrain_ranges = getPostProcessedConstrainRectangles(
    enable_avoidance, object_road_constrain_ranges, road_constrain_ranges,
    only_smooth_constrain_ranges, interpolated_points, path.points, farrest_point_idx,
    num_fixed_points, straight_idx);

  return constrain_ranges;
}

std::vector<ConstrainRectangle> EBPathOptimizer::getConstrainRectangleVec(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::vector<geometry_msgs::Point> & interpolated_points, const int num_fixed_points,
  const int farrest_point_idx)
{
  std::vector<ConstrainRectangle> only_smooth_constrain_ranges(traj_param_.num_sampling_points);
  for (int i = 0; i < traj_param_.num_sampling_points; i++) {
    geometry_msgs::Pose origin_pose = getOriginPose(interpolated_points, i, path_points);
    if (i == 0 || i == 1 || i >= farrest_point_idx - 1) {
      ConstrainRectangle rectangle = getConstrainRectangle(origin_pose, 0);
      only_smooth_constrain_ranges[i] = rectangle;
    } else if (
      i >= num_fixed_points, i <= num_fixed_points + traj_param_.num_fix_points_for_extending) {
      ConstrainRectangle rectangle =
        getConstrainRectangle(origin_pose, constrain_param_.clearance_for_joint);
      only_smooth_constrain_ranges[i] = rectangle;
    } else {
      ConstrainRectangle rectangle =
        getConstrainRectangle(origin_pose, constrain_param_.clearance_for_only_smoothing);
      only_smooth_constrain_ranges[i] = rectangle;
    }
  }
  return only_smooth_constrain_ranges;
}

std::vector<ConstrainRectangle> EBPathOptimizer::getPostProcessedConstrainRectangles(
  const bool enable_avoidance, const std::vector<ConstrainRectangle> & object_constrains,
  const std::vector<ConstrainRectangle> & road_constrains,
  const std::vector<ConstrainRectangle> & only_smooth_constrains,
  const std::vector<geometry_msgs::Point> & interpolated_points,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points, const int farrest_point_idx,
  const int num_fixed_points, const int straight_idx)
{
  std::vector<ConstrainRectangle> constrain_ranges(traj_param_.num_sampling_points);
  size_t origin_dynamic_joint_idx = traj_param_.num_sampling_points;
  bool is_using_road_constrain = false;
  if (!enable_avoidance) {
    is_using_road_constrain = true;
  }
  bool is_using_only_smooth_constrain = false;
  if (isFixingPathPoint(path_points)) {
    is_using_only_smooth_constrain = true;
  }
  for (size_t i = 0; i < traj_param_.num_sampling_points; i++) {
    if (
      i == 0 || i == 1 || i >= farrest_point_idx - 1 || i < num_fixed_points - 1 ||
      (i >= num_fixed_points - traj_param_.num_joint_buffer_points &&
       i <= num_fixed_points + traj_param_.num_joint_buffer_points) ||
      i >= straight_idx) {
      constrain_ranges[i] = object_constrains[i];
    } else {
      if (
        i > origin_dynamic_joint_idx &&
        i <= origin_dynamic_joint_idx + traj_param_.num_joint_buffer_points) {
        geometry_msgs::Pose origin_pose = getOriginPose(interpolated_points, i, path_points);
        ConstrainRectangle rectangle =
          getConstrainRectangle(origin_pose, constrain_param_.clearance_for_joint);
        constrain_ranges[i] = rectangle;
      } else if (is_using_only_smooth_constrain) {
        constrain_ranges[i] = only_smooth_constrains[i];
      } else if (is_using_road_constrain) {
        constrain_ranges[i] = road_constrains[i];
        if (constrain_ranges[i].is_dynamic_joint_rectangle) {
          ROS_INFO_COND(
            is_showing_debug_info_, "Only road clearance optimization failed at %zu", i);
          is_using_only_smooth_constrain = true;
          origin_dynamic_joint_idx = i;
        }
      } else {
        constrain_ranges[i] = object_constrains[i];
        if (constrain_ranges[i].is_dynamic_joint_rectangle) {
          ROS_INFO_COND(is_showing_debug_info_, "Object clearance optimization failed at %zu", i);
          is_using_road_constrain = true;
          origin_dynamic_joint_idx = i;
        }
      }
    }
  }
  return constrain_ranges;
}

void EBPathOptimizer::updateConstrain(
  const std::vector<geometry_msgs::Point> & interpolated_points,
  const std::vector<ConstrainRectangle> & rectangle_points)
{
  Eigen::MatrixXd A = default_a_matrix_;
  std::vector<double> lower_bound(traj_param_.num_sampling_points * 2, 0.0);
  std::vector<double> upper_bound(traj_param_.num_sampling_points * 2, 0.0);
  for (int i = 0; i < traj_param_.num_sampling_points; ++i) {
    Constrain constrain =
      getConstrainFromConstrainRectangle(interpolated_points[i], rectangle_points[i]);
    A(i, i) = constrain.top_and_bottom.x_coef;
    A(i, i + traj_param_.num_sampling_points) = constrain.top_and_bottom.y_coef;
    A(i + traj_param_.num_sampling_points, i) = constrain.left_and_right.x_coef;
    A(i + traj_param_.num_sampling_points, i + traj_param_.num_sampling_points) =
      constrain.left_and_right.y_coef;
    lower_bound[i] = constrain.top_and_bottom.lower_bound;
    upper_bound[i] = constrain.top_and_bottom.upper_bound;
    lower_bound[i + traj_param_.num_sampling_points] = constrain.left_and_right.lower_bound;
    upper_bound[i + traj_param_.num_sampling_points] = constrain.left_and_right.upper_bound;
  }
  osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
  osqp_solver_ptr_->updateA(A);
}

Rectangle EBPathOptimizer::getAbsShapeRectangle(
  const Rectangle & rel_shape_rectangle_points, const geometry_msgs::Point & offset_point,
  const geometry_msgs::Pose & origin)
{
  geometry_msgs::Point abs_target_point =
    util::transformToAbsoluteCoordinate2D(offset_point, origin);

  geometry_msgs::Point abs_top_left;
  abs_top_left.x = (rel_shape_rectangle_points.top_left.x + abs_target_point.x);
  abs_top_left.y = (rel_shape_rectangle_points.top_left.y + abs_target_point.y);

  geometry_msgs::Point abs_top_right;
  abs_top_right.x = (rel_shape_rectangle_points.top_right.x + abs_target_point.x);
  abs_top_right.y = (rel_shape_rectangle_points.top_right.y + abs_target_point.y);

  geometry_msgs::Point abs_bottom_left;
  abs_bottom_left.x = (rel_shape_rectangle_points.bottom_left.x + abs_target_point.x);
  abs_bottom_left.y = (rel_shape_rectangle_points.bottom_left.y + abs_target_point.y);

  geometry_msgs::Point abs_bottom_right;
  abs_bottom_right.x = (rel_shape_rectangle_points.bottom_right.x + abs_target_point.x);
  abs_bottom_right.y = (rel_shape_rectangle_points.bottom_right.y + abs_target_point.y);

  Rectangle abs_shape_rectangle_points;
  abs_shape_rectangle_points.top_left = abs_top_left;
  abs_shape_rectangle_points.top_right = abs_top_right;
  abs_shape_rectangle_points.bottom_left = abs_bottom_left;
  abs_shape_rectangle_points.bottom_right = abs_bottom_right;
  return abs_shape_rectangle_points;
}

geometry_msgs::Pose EBPathOptimizer::getOriginPose(
  const std::vector<geometry_msgs::Point> & interpolated_points, const int interpolated_idx,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points)
{
  double min_dist = std::numeric_limits<double>::max();
  geometry_msgs::Pose pose;
  pose.position = interpolated_points[interpolated_idx];
  if (interpolated_idx > 0) {
    pose.orientation = util::getQuaternionFromPoints(
      interpolated_points[interpolated_idx], interpolated_points[interpolated_idx - 1]);
  } else if (interpolated_idx == 0 && interpolated_points.size() > 1) {
    pose.orientation = util::getQuaternionFromPoints(
      interpolated_points[interpolated_idx + 1], interpolated_points[interpolated_idx]);
  }
  const int default_idx = 0;
  const int nearest_id = util::getNearestIdx(
    path_points, pose, default_idx, traj_param_.delta_yaw_threshold_for_closest_point);
  const geometry_msgs::Quaternion nearest_q = path_points[nearest_id].pose.orientation;
  geometry_msgs::Pose origin;
  origin.position = interpolated_points[interpolated_idx];
  origin.orientation = nearest_q;
  return origin;
}

std::shared_ptr<std::vector<std::vector<geometry_msgs::Point>>> EBPathOptimizer::getOccupancyPoints(
  const geometry_msgs::Pose & origin, const cv::Mat & clearance_map,
  const nav_msgs::MapMetaData & map_info)
{
  std::shared_ptr<geometry_msgs::Point> interpolated_image_point_ptr =
    util::transformMapToImagePtr(origin.position, map_info);
  if (interpolated_image_point_ptr == nullptr) {
    return nullptr;
  }
  const float clearance =
    clearance_map.ptr<float>(
      (int)interpolated_image_point_ptr->y)[(int)interpolated_image_point_ptr->x] *
    map_info.resolution;
  const float y_constrain_search_range = clearance - 0.5 * keep_space_shape_ptr_->y;
  int y_side_length = 0;
  for (float y = -y_constrain_search_range; y <= y_constrain_search_range + epsilon_;
       y += map_info.resolution * constrain_param_.coef_y_cosntrain_search_resolution) {
    y_side_length++;
  }
  const float x_constrain_search_range =
    std::fmin(constrain_param_.max_x_constrain_search_range, y_constrain_search_range);
  int x_side_length = 0;
  for (float x = -1 * x_constrain_search_range; x <= x_constrain_search_range + epsilon_;
       x += map_info.resolution * constrain_param_.coef_x_cosntrain_search_resolution) {
    x_side_length++;
  }
  if (x_side_length == 0 || y_side_length == 0) {
    return nullptr;
  }
  std::vector<std::vector<geometry_msgs::Point>> occupancy_points(
    x_side_length, std::vector<geometry_msgs::Point>(y_side_length));
  int x_idx_in_occupancy_map = 0;
  int y_idx_in_occupancy_map = 0;
  for (float x = -1 * x_constrain_search_range; x <= x_constrain_search_range + epsilon_;
       x += map_info.resolution * constrain_param_.coef_x_cosntrain_search_resolution) {
    for (float y = -1 * y_constrain_search_range; y <= y_constrain_search_range + epsilon_;
         y += map_info.resolution * constrain_param_.coef_y_cosntrain_search_resolution) {
      geometry_msgs::Point relative_point;
      relative_point.x = x;
      relative_point.y = y;
      geometry_msgs::Point abs_point =
        util::transformToAbsoluteCoordinate2D(relative_point, origin);
      occupancy_points[x_side_length - x_idx_in_occupancy_map - 1]
                      [y_side_length - y_idx_in_occupancy_map - 1] = abs_point;
      y_idx_in_occupancy_map++;
    }
    x_idx_in_occupancy_map++;
    y_idx_in_occupancy_map = 0;
  }
  std::shared_ptr<std::vector<std::vector<geometry_msgs::Point>>> occupancy_points_ptr;
  occupancy_points_ptr =
    std::make_shared<std::vector<std::vector<geometry_msgs::Point>>>(occupancy_points);
  return occupancy_points_ptr;
}

Rectangle EBPathOptimizer::getRelShapeRectangle(
  const geometry_msgs::Vector3 & vehicle_shape, const geometry_msgs::Pose & origin)
{
  geometry_msgs::Point top_left;
  top_left.x = vehicle_shape.x;
  top_left.y = 0.5 * vehicle_shape.y;
  geometry_msgs::Point top_right;
  top_right.x = vehicle_shape.x;
  top_right.y = -0.5 * vehicle_shape.y;
  geometry_msgs::Point bottom_left;
  bottom_left.x = 0.0;
  bottom_left.y = 0.5 * vehicle_shape.y;
  geometry_msgs::Point bottom_right;
  bottom_right.x = 0.0;
  bottom_right.y = -0.5 * vehicle_shape.y;

  geometry_msgs::Pose tmp_origin;
  tmp_origin.orientation = origin.orientation;
  top_left = util::transformToAbsoluteCoordinate2D(top_left, tmp_origin);
  top_right = util::transformToAbsoluteCoordinate2D(top_right, tmp_origin);
  bottom_left = util::transformToAbsoluteCoordinate2D(bottom_left, tmp_origin);
  bottom_right = util::transformToAbsoluteCoordinate2D(bottom_right, tmp_origin);
  Rectangle rectangle;
  rectangle.top_left = top_left;
  rectangle.top_right = top_right;
  rectangle.bottom_left = bottom_left;
  rectangle.bottom_right = bottom_right;
  return rectangle;
}

ConstrainRectangles EBPathOptimizer::getConstrainRectangles(
  const geometry_msgs::Pose & origin_pose, const cv::Mat & clearance_map,
  const cv::Mat & only_objects_clearance_map, const nav_msgs::MapMetaData & map_info)
{
  const auto occupancy_points_ptr = getOccupancyPoints(origin_pose, clearance_map, map_info);

  std::shared_ptr<geometry_msgs::Point> image_point_ptr =
    util::transformMapToImagePtr(origin_pose.position, map_info);
  if (!image_point_ptr || !occupancy_points_ptr) {
    ConstrainRectangle rectangle =
      getConstrainRectangle(origin_pose, constrain_param_.clearance_for_joint);
    rectangle.is_dynamic_joint_rectangle = true;
    ConstrainRectangles constrain_rectangles;
    constrain_rectangles.object_constrain_rectangle = rectangle;
    constrain_rectangles.road_constrain_rectangle = rectangle;
    return constrain_rectangles;
  }
  OccupancyMaps occupancy_maps = getOccupancyMaps(
    *occupancy_points_ptr, origin_pose, *image_point_ptr, clearance_map, only_objects_clearance_map,
    map_info);

  ConstrainRectangles constrain_rectangles;
  constrain_rectangles.object_constrain_rectangle =
    getConstrainRectangle(occupancy_maps.object_occupancy_map, *occupancy_points_ptr, origin_pose);
  constrain_rectangles.road_constrain_rectangle =
    getConstrainRectangle(occupancy_maps.road_occupancy_map, *occupancy_points_ptr, origin_pose);
  return constrain_rectangles;
}

OccupancyMaps EBPathOptimizer::getOccupancyMaps(
  const std::vector<std::vector<geometry_msgs::Point>> & occupancy_points,
  const geometry_msgs::Pose & origin_pose, const geometry_msgs::Point & origin_point_in_image,
  const cv::Mat & clearance_map, const cv::Mat & only_objects_clearance_map,
  const nav_msgs::MapMetaData & map_info)
{
  Rectangle rel_shape_rectangles = getRelShapeRectangle(*keep_space_shape_ptr_, origin_pose);
  const float clearance =
    clearance_map.ptr<float>((int)origin_point_in_image.y)[(int)origin_point_in_image.x] *
    map_info.resolution;
  const float y_constrain_search_range = clearance - 0.5 * keep_space_shape_ptr_->y;
  const float x_constrain_search_range =
    std::fmin(constrain_param_.max_x_constrain_search_range, y_constrain_search_range);
  std::vector<std::vector<int>> object_occupancy_map(
    occupancy_points.size(), std::vector<int>(occupancy_points.front().size(), 0));
  std::vector<std::vector<int>> road_occupancy_map(
    occupancy_points.size(), std::vector<int>(occupancy_points.front().size(), 0));
  int x_idx_in_occupancy_map = 0;
  int y_idx_in_occupancy_map = 0;
  for (float x = -1 * x_constrain_search_range; x <= x_constrain_search_range + epsilon_;
       x += map_info.resolution * constrain_param_.coef_x_cosntrain_search_resolution) {
    for (float y = -1 * y_constrain_search_range; y <= y_constrain_search_range + epsilon_;
         y += map_info.resolution * constrain_param_.coef_y_cosntrain_search_resolution) {
      geometry_msgs::Point rel_target_point;
      rel_target_point.x = x;
      rel_target_point.y = y;
      Rectangle abs_shape_rectangles =
        getAbsShapeRectangle(rel_shape_rectangles, rel_target_point, origin_pose);
      float top_left_clearance = std::numeric_limits<float>::lowest();
      float top_left_objects_clearance = std::numeric_limits<float>::lowest();
      geometry_msgs::Point top_left_image;
      if (util::transformMapToImage(abs_shape_rectangles.top_left, map_info, top_left_image)) {
        top_left_clearance =
          clearance_map.ptr<float>((int)top_left_image.y)[(int)top_left_image.x] *
          map_info.resolution;
        top_left_objects_clearance =
          only_objects_clearance_map.ptr<float>((int)top_left_image.y)[(int)top_left_image.x] *
          map_info.resolution;
      }

      float top_right_clearance = std::numeric_limits<float>::lowest();
      float top_right_objects_clearance = std::numeric_limits<float>::lowest();
      geometry_msgs::Point top_right_image;
      if (util::transformMapToImage(abs_shape_rectangles.top_right, map_info, top_right_image)) {
        top_right_clearance =
          clearance_map.ptr<float>((int)top_right_image.y)[(int)top_right_image.x] *
          map_info.resolution;
        top_right_objects_clearance =
          only_objects_clearance_map.ptr<float>((int)top_right_image.y)[(int)top_right_image.x] *
          map_info.resolution;
      }
      float bottom_left_clearance = std::numeric_limits<float>::lowest();
      float bottom_left_objects_clearance = std::numeric_limits<float>::lowest();
      geometry_msgs::Point bottom_left_image;
      if (util::transformMapToImage(
            abs_shape_rectangles.bottom_left, map_info, bottom_left_image)) {
        bottom_left_clearance =
          clearance_map.ptr<float>((int)bottom_left_image.y)[(int)bottom_left_image.x] *
          map_info.resolution;
        bottom_left_objects_clearance = only_objects_clearance_map.ptr<float>(
                                          (int)bottom_left_image.y)[(int)bottom_left_image.x] *
                                        map_info.resolution;
      }
      float bottom_right_clearance = std::numeric_limits<float>::lowest();
      float bottom_right_objects_clearance = std::numeric_limits<float>::lowest();
      geometry_msgs::Point bottom_right_image;
      if (util::transformMapToImage(
            abs_shape_rectangles.bottom_right, map_info, bottom_right_image)) {
        bottom_right_clearance =
          clearance_map.ptr<float>((int)bottom_right_image.y)[(int)bottom_right_image.x] *
          map_info.resolution;
        bottom_right_objects_clearance = only_objects_clearance_map.ptr<float>(
                                           (int)bottom_right_image.y)[(int)bottom_right_image.x] *
                                         map_info.resolution;
      }
      if (
        top_left_clearance < constrain_param_.min_clearance_from_road ||
        top_right_clearance < constrain_param_.min_clearance_from_road ||
        bottom_right_clearance < constrain_param_.min_clearance_from_road ||
        bottom_left_clearance < constrain_param_.min_clearance_from_road ||
        top_left_objects_clearance < constrain_param_.min_clearance_from_object ||
        top_right_objects_clearance < constrain_param_.min_clearance_from_object ||
        bottom_right_objects_clearance < constrain_param_.min_clearance_from_object ||
        bottom_left_objects_clearance < constrain_param_.min_clearance_from_object) {
        object_occupancy_map[occupancy_points.size() - x_idx_in_occupancy_map - 1]
                            [occupancy_points.front().size() - y_idx_in_occupancy_map - 1] = 1;
      }
      if (
        top_left_clearance < constrain_param_.min_clearance_from_road ||
        top_right_clearance < constrain_param_.min_clearance_from_road ||
        bottom_right_clearance < constrain_param_.min_clearance_from_road ||
        bottom_left_clearance < constrain_param_.min_clearance_from_road) {
        road_occupancy_map[occupancy_points.size() - x_idx_in_occupancy_map - 1]
                          [occupancy_points.front().size() - y_idx_in_occupancy_map - 1] = 1;
      }
      y_idx_in_occupancy_map++;
    }
    x_idx_in_occupancy_map++;
    y_idx_in_occupancy_map = 0;
  }
  OccupancyMaps occupancy_maps;
  occupancy_maps.object_occupancy_map = object_occupancy_map;
  occupancy_maps.road_occupancy_map = road_occupancy_map;
  return occupancy_maps;
}

int EBPathOptimizer::getStraightLineIdx(
  const std::vector<geometry_msgs::Point> & interpolated_points, const int farrest_point_idx,
  const cv::Mat & only_objects_clearance_map, const nav_msgs::MapMetaData & map_info,
  std::vector<geometry_msgs::Point> & debug_detected_straight_points)
{
  double prev_yaw = 0;
  int straight_line_idx = farrest_point_idx;
  for (int i = farrest_point_idx; i >= 0; i--) {
    if (i < farrest_point_idx) {
      const double yaw = util::getYawFromPoints(interpolated_points[i + 1], interpolated_points[i]);
      const double delta_yaw = yaw - prev_yaw;
      const double norm_delta_yaw = util::normalizeRadian(delta_yaw);
      float clearance_from_object = std::numeric_limits<float>::max();
      std::shared_ptr<geometry_msgs::Point> image_point_ptr =
        util::transformMapToImagePtr(interpolated_points[i], map_info);
      if (image_point_ptr) {
        clearance_from_object =
          only_objects_clearance_map.ptr<float>((int)image_point_ptr->y)[(int)image_point_ptr->x] *
          map_info.resolution;
      }
      if (
        std::fabs(norm_delta_yaw) > traj_param_.delta_yaw_threshold_for_straight ||
        clearance_from_object < constrain_param_.clearance_from_object_for_straight) {
        break;
      }
      straight_line_idx = i;
      prev_yaw = yaw;
    } else if (i == farrest_point_idx && farrest_point_idx >= 1) {
      const double yaw = util::getYawFromPoints(interpolated_points[i], interpolated_points[i - 1]);
      prev_yaw = yaw;
    }
  }
  for (int i = straight_line_idx; i <= farrest_point_idx; i++) {
    debug_detected_straight_points.push_back(interpolated_points[i]);
  }
  return straight_line_idx;
}

Constrain EBPathOptimizer::getConstrainFromConstrainRectangle(
  const geometry_msgs::Point & interpolated_point, const ConstrainRectangle & constrain_range)
{
  Constrain constrain;
  const double top_dx = constrain_range.top_left.x - constrain_range.top_right.x;
  const double top_dy = constrain_range.top_left.y - constrain_range.top_right.y;
  const double left_dx = constrain_range.top_left.x - constrain_range.bottom_left.x;
  const double left_dy = constrain_range.top_left.y - constrain_range.bottom_left.y;
  if (
    std::fabs(top_dx) < epsilon_ && std::fabs(top_dy) < epsilon_ && std::fabs(left_dx) < epsilon_ &&
    std::fabs(left_dy) < epsilon_) {
    constrain.top_and_bottom.x_coef = 1;
    constrain.top_and_bottom.y_coef = 1;
    constrain.top_and_bottom.lower_bound = interpolated_point.x + interpolated_point.y;
    constrain.top_and_bottom.upper_bound = interpolated_point.x + interpolated_point.y;
    constrain.left_and_right.x_coef = -1;
    constrain.left_and_right.y_coef = 1;
    constrain.left_and_right.lower_bound = interpolated_point.y - interpolated_point.x;
    constrain.left_and_right.upper_bound = interpolated_point.y - interpolated_point.x;
  } else if (std::fabs(top_dx) < epsilon_) {
    constrain.top_and_bottom.x_coef = 1;
    constrain.top_and_bottom.y_coef = epsilon_;
    constrain.top_and_bottom.lower_bound = interpolated_point.x;
    constrain.top_and_bottom.upper_bound = interpolated_point.x;
    constrain.left_and_right =
      getConstrainLines(left_dx, left_dy, constrain_range.top_left, constrain_range.top_right);
  } else if (std::fabs(top_dy) < epsilon_) {
    constrain.top_and_bottom.x_coef = epsilon_;
    constrain.top_and_bottom.y_coef = 1;
    constrain.top_and_bottom.lower_bound = interpolated_point.y;
    constrain.top_and_bottom.upper_bound = interpolated_point.y;
    constrain.left_and_right =
      getConstrainLines(left_dx, left_dy, constrain_range.top_left, constrain_range.top_right);
  } else if (std::fabs(left_dx) < epsilon_) {
    constrain.left_and_right.x_coef = 1;
    constrain.left_and_right.y_coef = epsilon_;
    constrain.left_and_right.lower_bound = interpolated_point.x;
    constrain.left_and_right.upper_bound = interpolated_point.x;
    constrain.top_and_bottom =
      getConstrainLines(top_dx, top_dy, constrain_range.top_left, constrain_range.bottom_left);
  } else if (std::fabs(left_dy) < epsilon_) {
    constrain.left_and_right.x_coef = epsilon_;
    constrain.left_and_right.y_coef = 1;
    constrain.left_and_right.lower_bound = interpolated_point.y;
    constrain.left_and_right.upper_bound = interpolated_point.y;
    constrain.top_and_bottom =
      getConstrainLines(top_dx, top_dy, constrain_range.top_left, constrain_range.bottom_left);
  } else {
    constrain.top_and_bottom =
      getConstrainLines(top_dx, top_dy, constrain_range.top_left, constrain_range.bottom_left);
    constrain.left_and_right =
      getConstrainLines(left_dx, left_dy, constrain_range.top_left, constrain_range.top_right);
  }
  return constrain;
}

ConstrainLines EBPathOptimizer::getConstrainLines(
  const double dx, const double dy, const geometry_msgs::Point & point,
  const geometry_msgs::Point & oppsite_point)
{
  ConstrainLines constrain_point;

  const double slope = dy / dx;
  const double intercept = point.y - slope * point.x;
  const double intercept2 = oppsite_point.y - slope * oppsite_point.x;
  constrain_point.x_coef = -1 * slope;
  constrain_point.y_coef = 1;
  if (intercept > intercept2) {
    constrain_point.lower_bound = intercept2;
    constrain_point.upper_bound = intercept;
  } else {
    constrain_point.lower_bound = intercept;
    constrain_point.upper_bound = intercept2;
  }
  return constrain_point;
}

ConstrainRectangle EBPathOptimizer::getConstrainRectangle(
  const geometry_msgs::Pose & origin_pose, const double clearance)
{
  ConstrainRectangle constrain_range;
  geometry_msgs::Point top_left;
  top_left.x = clearance;
  top_left.y = clearance;
  constrain_range.top_left = util::transformToAbsoluteCoordinate2D(top_left, origin_pose);
  geometry_msgs::Point top_right;
  top_right.x = clearance;
  top_right.y = -1 * clearance;
  constrain_range.top_right = util::transformToAbsoluteCoordinate2D(top_right, origin_pose);
  geometry_msgs::Point bottom_left;
  bottom_left.x = -1 * clearance;
  bottom_left.y = clearance;
  constrain_range.bottom_left = util::transformToAbsoluteCoordinate2D(bottom_left, origin_pose);
  geometry_msgs::Point bottom_right;
  bottom_right.x = -1 * clearance;
  bottom_right.y = -1 * clearance;
  constrain_range.bottom_right = util::transformToAbsoluteCoordinate2D(bottom_right, origin_pose);
  return constrain_range;
}

ConstrainRectangle EBPathOptimizer::getConstrainRectangle(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const geometry_msgs::Pose & origin_pose, const cv::Mat & clearance_map,
  const nav_msgs::MapMetaData & map_info)
{
  const int default_idx = -1;
  const auto interpolated_points =
    util::getInterpolatedPoints(path_points, traj_param_.delta_arc_length_for_trajectory);
  const int nearest_idx = util::getNearestIdx(
    interpolated_points, origin_pose, default_idx,
    traj_param_.delta_yaw_threshold_for_closest_point);

  float clearance = std::numeric_limits<float>::lowest();
  geometry_msgs::Point image_point;
  if (util::transformMapToImage(interpolated_points[nearest_idx], map_info, image_point)) {
    clearance =
      clearance_map.ptr<float>((int)image_point.y)[(int)image_point.x] * map_info.resolution;
  }
  const double dist =
    util::calculate2DDistance(origin_pose.position, interpolated_points[nearest_idx]);
  ConstrainRectangle constrain_rectangle;
  if (nearest_idx > default_idx && dist < clearance) {
    geometry_msgs::Pose replaced_pose = origin_pose;
    replaced_pose.position = interpolated_points[nearest_idx];
    if (nearest_idx > 0) {
      replaced_pose.orientation = util::getQuaternionFromPoints(
        interpolated_points[nearest_idx], interpolated_points[nearest_idx - 1]);
    } else if (nearest_idx == 0 && interpolated_points.size() > 1) {
      replaced_pose.orientation = util::getQuaternionFromPoints(
        interpolated_points[nearest_idx + 1], interpolated_points[nearest_idx]);
    }
    ConstrainRectangles rectangles =
      getConstrainRectangles(replaced_pose, clearance_map, clearance_map, map_info);
    const double rel_plus_y = util::calculate2DDistance(
      rectangles.road_constrain_rectangle.top_left, replaced_pose.position);
    const double rel_minus_y = util::calculate2DDistance(
      rectangles.road_constrain_rectangle.top_right, replaced_pose.position);
    geometry_msgs::Point top_left;
    top_left.x = constrain_param_.clearance_for_joint;
    top_left.y = rel_plus_y;
    constrain_rectangle.top_left = util::transformToAbsoluteCoordinate2D(top_left, replaced_pose);
    geometry_msgs::Point top_right;
    top_right.x = constrain_param_.clearance_for_joint;
    top_right.y = -1 * rel_minus_y;
    constrain_rectangle.top_right = util::transformToAbsoluteCoordinate2D(top_right, replaced_pose);
    geometry_msgs::Point bottom_left;
    bottom_left.x = -1 * constrain_param_.clearance_for_joint;
    bottom_left.y = rel_plus_y;
    constrain_rectangle.bottom_left =
      util::transformToAbsoluteCoordinate2D(bottom_left, replaced_pose);
    geometry_msgs::Point bottom_right;
    bottom_right.x = -1 * constrain_param_.clearance_for_joint;
    bottom_right.y = -1 * rel_minus_y;
    constrain_rectangle.bottom_right =
      util::transformToAbsoluteCoordinate2D(bottom_right, replaced_pose);
  } else {
    constrain_rectangle = getConstrainRectangle(origin_pose, constrain_param_.clearance_for_joint);
  }
  return constrain_rectangle;
}

ConstrainRectangle EBPathOptimizer::getConstrainRectangle(
  const std::vector<std::vector<int>> & occupancy_map,
  const std::vector<std::vector<geometry_msgs::Point>> & occupancy_points,
  const geometry_msgs::Pose & origin_pose)
{
  util::Rectangle util_rect = util::getLargestRectangle(occupancy_map);
  double lat_space = util::calculate2DDistance(
    occupancy_points[util_rect.max_x_idx][util_rect.max_y_idx],
    occupancy_points[util_rect.max_x_idx][util_rect.min_y_idx]);
  double lon_space = util::calculate2DDistance(
    occupancy_points[util_rect.max_x_idx][util_rect.max_y_idx],
    occupancy_points[util_rect.min_x_idx][util_rect.max_y_idx]);
  if (
    lat_space <= constrain_param_.min_lat_constrain_space + epsilon_ ||
    lon_space <= constrain_param_.min_lon_constrain_space + epsilon_) {
    constexpr int occupied_value = 1;
    std::vector<std::vector<int>> tmp_occupancy_map = occupancy_map;
    for (int i = util_rect.max_x_idx; i <= util_rect.min_x_idx; i++) {
      for (int j = util_rect.max_y_idx; j <= util_rect.min_y_idx; j++) {
        tmp_occupancy_map[i][j] = occupied_value;
      }
    }
    util_rect = util::getLargestRectangle(tmp_occupancy_map);
    lat_space = util::calculate2DDistance(
      occupancy_points[util_rect.max_x_idx][util_rect.max_y_idx],
      occupancy_points[util_rect.max_x_idx][util_rect.min_y_idx]);
    lon_space = util::calculate2DDistance(
      occupancy_points[util_rect.max_x_idx][util_rect.max_y_idx],
      occupancy_points[util_rect.min_x_idx][util_rect.max_y_idx]);
  }

  ConstrainRectangle constrain_rectangle;
  if (
    util_rect.area < epsilon_ || lat_space <= constrain_param_.min_lat_constrain_space + epsilon_ ||
    lon_space <= constrain_param_.min_lon_constrain_space + epsilon_) {
    constrain_rectangle = getConstrainRectangle(origin_pose, constrain_param_.clearance_for_joint);
    constrain_rectangle.is_dynamic_joint_rectangle = true;
  } else {
    constrain_rectangle.bottom_left = occupancy_points[util_rect.min_x_idx][util_rect.max_y_idx];
    constrain_rectangle.bottom_right = occupancy_points[util_rect.min_x_idx][util_rect.min_y_idx];
    constrain_rectangle.top_left = occupancy_points[util_rect.max_x_idx][util_rect.max_y_idx];
    constrain_rectangle.top_right = occupancy_points[util_rect.max_x_idx][util_rect.min_y_idx];
  }
  return constrain_rectangle;
}

bool EBPathOptimizer::isFixingPathPoint(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points)
{
  for (const auto & point : path_points) {
    if (point.type == point.FIXED) {
      return true;
    }
  }
  return false;
}
