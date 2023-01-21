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

#ifndef COLLISION_FREE_PATH_PLANNER__UTILS__TRAJECTORY_UTILS_HPP_
#define COLLISION_FREE_PATH_PLANNER__UTILS__TRAJECTORY_UTILS_HPP_

#include "collision_free_path_planner/common_structs.hpp"
#include "collision_free_path_planner/type_alias.hpp"
#include "eigen3/Eigen/Core"
#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/trajectory/trajectory.hpp"

#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include "boost/optional/optional_fwd.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace tier4_autoware_utils
{
template <>
geometry_msgs::msg::Point getPoint(const collision_free_path_planner::ReferencePoint & p);

template <>
geometry_msgs::msg::Pose getPose(const collision_free_path_planner::ReferencePoint & p);

template <>
double getLongitudinalVelocity(const collision_free_path_planner::ReferencePoint & p);
}  // namespace tier4_autoware_utils

namespace collision_free_path_planner
{
namespace trajectory_utils
{
template <typename T>
size_t findForwardIndex(const T & points, const size_t begin_idx, const double forward_length)
{
  double sum_length = 0.0;
  for (size_t i = begin_idx; i < points.size() - 1; ++i) {
    sum_length += tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
    if (sum_length > forward_length) {
      return i;
    }
  }
  return points.size() - 1;
}

template <typename T>
T cropPointsAfterOffsetPoint(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double offset)
{
  if (points.empty()) {
    return T{};
  }

  double sum_length =
    -motion_utils::calcLongitudinalOffsetToSegment(points, target_seg_idx, target_pos);

  // search forward
  if (sum_length < offset) {
    for (size_t i = target_seg_idx + 1; i < points.size(); ++i) {
      sum_length += tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i - 1));
      if (offset < sum_length) {
        return T{points.begin() + i - 1, points.end()};
      }
    }

    return T{};
  }

  // search backward
  for (size_t i = target_seg_idx; 0 < i;
       --i) {  // NOTE: use size_t since i is always positive value
    sum_length -= tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i - 1));
    if (sum_length < offset) {
      return T{points.begin() + i - 1, points.end()};
    }
  }

  return points;
}

template <typename T>
T cropForwardPoints(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length)
{
  if (points.empty()) {
    return T{};
  }

  double sum_length =
    -motion_utils::calcLongitudinalOffsetToSegment(points, target_seg_idx, target_pos);
  for (size_t i = target_seg_idx + 1; i < points.size(); ++i) {
    sum_length += tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i - 1));
    if (forward_length < sum_length) {
      const size_t end_idx = i;
      return T{points.begin(), points.begin() + end_idx};
    }
  }

  return points;
}

template <typename T>
T cropBackwardPoints(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double backward_length)
{
  if (points.empty()) {
    return T{};
  }

  double sum_length =
    -motion_utils::calcLongitudinalOffsetToSegment(points, target_seg_idx, target_pos);
  for (size_t i = target_seg_idx; 0 < i;
       --i) {  // NOTE: use size_t since i is always positive value
    sum_length -= tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i - 1));
    if (sum_length < backward_length) {
      const size_t begin_idx = i - 1;
      return T{points.begin() + begin_idx, points.end()};
    }
  }

  return points;
}

template <typename T>
T cropPoints(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length)
{
  if (points.empty()) {
    return T{};
  }

  // NOTE: Cropping forward must be done first in order to keep target_seg_idx.
  const auto cropped_forward_points =
    cropForwardPoints(points, target_pos, target_seg_idx, forward_length);

  const size_t modified_target_seg_idx =
    std::min(target_seg_idx, cropped_forward_points.size() - 2);
  const auto cropped_points = cropBackwardPoints(
    cropped_forward_points, target_pos, modified_target_seg_idx, backward_length);

  return cropped_points;
}

template <typename T>
T clipForwardPoints(const T & points, const size_t begin_idx, const double forward_length)
{
  if (points.empty()) {
    return T{};
  }

  const size_t end_idx = findForwardIndex(points, begin_idx, forward_length);
  return T{points.begin() + begin_idx, points.begin() + end_idx};
}

template <typename T>
T clipBackwardPoints(
  const T & points, const size_t target_idx, const double backward_length,
  const double delta_length)
{
  if (points.empty()) {
    return T{};
  }

  const int begin_idx =
    std::max(0, static_cast<int>(target_idx) - static_cast<int>(backward_length / delta_length));
  return T{points.begin() + begin_idx, points.end()};
}

template <typename T>
T clipBackwardPoints(
  const T & points, const geometry_msgs::msg::Pose pose, const double backward_length,
  const double delta_length, const double delta_yaw)
{
  if (points.empty()) {
    return T{};
  }

  const auto target_idx_optional =
    motion_utils::findNearestIndex(points, pose, std::numeric_limits<double>::max(), delta_yaw);

  const size_t target_idx = target_idx_optional
                              ? *target_idx_optional
                              : motion_utils::findNearestIndex(points, pose.position);

  const int begin_idx =
    std::max(0, static_cast<int>(target_idx) - static_cast<int>(backward_length / delta_length));
  return T{points.begin() + begin_idx, points.end()};
}

template <typename T>
TrajectoryPoint convertToTrajectoryPoint(const T & point)
{
  TrajectoryPoint traj_point;
  traj_point.pose = tier4_autoware_utils::getPose(point);
  traj_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
  traj_point.lateral_velocity_mps = point.lateral_velocity_mps;
  traj_point.heading_rate_rps = point.heading_rate_rps;
  return traj_point;
}

template <>
inline TrajectoryPoint convertToTrajectoryPoint(const ReferencePoint & ref_point)
{
  TrajectoryPoint traj_point;
  traj_point.pose = tier4_autoware_utils::getPose(ref_point);
  traj_point.longitudinal_velocity_mps = tier4_autoware_utils::getLongitudinalVelocity(ref_point);
  return traj_point;
}

// functions to convert to another type of points
template <typename T>
std::vector<TrajectoryPoint> convertToTrajectoryPoints(const std::vector<T> & points)
{
  std::vector<TrajectoryPoint> traj_points;
  for (const auto & point : points) {
    const auto traj_point = convertToTrajectoryPoint(point);
    traj_points.push_back(traj_point);
  }
  return traj_points;
}

ReferencePoint convertToReferencePoint(const TrajectoryPoint & traj_point);
std::vector<ReferencePoint> convertToReferencePoints(
  const std::vector<TrajectoryPoint> & traj_points);

/*
template <typename T>
ReferencePoint convertToReferencePoint(const T & point);

template <typename T>
std::vector<ReferencePoint> convertToReferencePoints(const std::vector<T> & points)
{
  std::vector<ReferencePoint> ref_points;
  for (const auto & point : points) {
    const auto ref_point = convertToReferencePoint(point);
    ref_points.push_back(ref_point);
  }
  return ref_points;
}
*/

void compensateLastPose(
  const PathPoint & last_path_point, std::vector<TrajectoryPoint> & traj_points,
  const double delta_dist_threshold, const double delta_yaw_threshold);

geometry_msgs::msg::Point getNearestPosition(
  const std::vector<ReferencePoint> & points, const int target_idx, const double offset);

template <typename T>
bool isNearLastPathPoint(
  const T & ref_point, const std::vector<TrajectoryPoint> & traj_points,
  const double delta_dist_threshold, const double delta_yaw_threshold)
{
  const geometry_msgs::msg::Pose last_ref_pose = tier4_autoware_utils::getPose(ref_point);

  if (
    tier4_autoware_utils::calcDistance2d(last_ref_pose, traj_points.back().pose) >
    delta_dist_threshold) {
    return false;
  }
  if (
    std::fabs(tier4_autoware_utils::calcYawDeviation(last_ref_pose, traj_points.back().pose)) >
    delta_yaw_threshold) {
    return false;
  }
  return true;
}

template <class T>
size_t findEgoIndex(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & ego_pose,
  const EgoNearestParam & ego_nearest_param)
{
  return motion_utils::findFirstNearestIndexWithSoftConstraints(
    points, ego_pose, ego_nearest_param.dist_threshold, ego_nearest_param.yaw_threshold);
}

template <class T>
size_t findEgoSegmentIndex(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & ego_pose,
  const EgoNearestParam & ego_nearest_param)
{
  return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, ego_pose, ego_nearest_param.dist_threshold, ego_nearest_param.yaw_threshold);
}

Trajectory createTrajectory(
  const std_msgs::msg::Header & header, const std::vector<TrajectoryPoint> & traj_points);

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> traj_points, const double interval);

template <typename T>
void updateFrontPointForFix(
  std::vector<T> & points, const geometry_msgs::msg::Pose & target_pose, const double epsilon)
{
  const double dist = tier4_autoware_utils::calcDistance2d(points.front(), target_pose);

  if (dist < epsilon) {
    // only pose is updated
    points.front().pose = target_pose;
  } else {
    // add new front point
    T new_front_point;
    new_front_point.pose = target_pose;
    new_front_point.longitudinal_velocity_mps = points.front().longitudinal_velocity_mps;

    points.insert(points.begin(), new_front_point);
  }
}
}  // namespace trajectory_utils
}  // namespace collision_free_path_planner
#endif  // COLLISION_FREE_PATH_PLANNER__UTILS__TRAJECTORY_UTILS_HPP_
