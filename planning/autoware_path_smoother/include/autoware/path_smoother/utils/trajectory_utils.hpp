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

#ifndef AUTOWARE__PATH_SMOOTHER__UTILS__TRAJECTORY_UTILS_HPP_
#define AUTOWARE__PATH_SMOOTHER__UTILS__TRAJECTORY_UTILS_HPP_

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_smoother/common_structs.hpp"
#include "autoware/path_smoother/type_alias.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <Eigen/Core>

#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include "boost/optional/optional_fwd.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::path_smoother
{
namespace trajectory_utils
{
template <typename T>
std::optional<size_t> getPointIndexAfter(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double max_offset, const double min_offset)
{
  if (points.empty()) {
    return std::nullopt;
  }

  double sum_length =
    -autoware::motion_utils::calcLongitudinalOffsetToSegment(points, target_seg_idx, target_pos);

  std::optional<size_t> output_idx{std::nullopt};

  // search forward
  if (sum_length < min_offset) {
    for (size_t i = target_seg_idx + 1; i < points.size(); ++i) {
      sum_length += autoware::universe_utils::calcDistance2d(points.at(i), points.at(i - 1));
      if (min_offset < sum_length) {
        output_idx = i - 1;
      }
      if (max_offset < sum_length) {
        break;
      }
    }
    return output_idx;
  }

  // search backward
  for (size_t i = target_seg_idx; 0 < i;
       --i) {  // NOTE: use size_t since i is always positive value
    sum_length -= autoware::universe_utils::calcDistance2d(points.at(i), points.at(i - 1));
    if (sum_length < min_offset) {
      output_idx = i - 1;
    }
    if (sum_length < max_offset) {
      break;
    }
  }

  return output_idx;
}

template <typename T>
TrajectoryPoint convertToTrajectoryPoint(const T & point)
{
  TrajectoryPoint traj_point;
  traj_point.pose = autoware::universe_utils::getPose(point);
  traj_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
  traj_point.lateral_velocity_mps = point.lateral_velocity_mps;
  traj_point.heading_rate_rps = point.heading_rate_rps;
  return traj_point;
}

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

template <class T>
size_t findEgoSegmentIndex(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & ego_pose,
  const EgoNearestParam & ego_nearest_param)
{
  return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, ego_pose, ego_nearest_param.dist_threshold, ego_nearest_param.yaw_threshold);
}

Path create_path(Path path_msg, const std::vector<TrajectoryPoint> & traj_points);

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval);

std::vector<TrajectoryPoint> resampleTrajectoryPointsWithoutStopPoint(
  const std::vector<TrajectoryPoint> & traj_points, const double interval);

template <typename T>
std::optional<size_t> updateFrontPointForFix(
  std::vector<T> & points, std::vector<T> & points_for_fix, const double delta_arc_length,
  const EgoNearestParam & ego_nearest_param)
{
  // calculate front point to insert in points as a fixed point
  const size_t front_seg_idx_for_fix = trajectory_utils::findEgoSegmentIndex(
    points_for_fix, autoware::universe_utils::getPose(points.front()), ego_nearest_param);
  const size_t front_point_idx_for_fix = front_seg_idx_for_fix;
  const auto & front_fix_point = points_for_fix.at(front_point_idx_for_fix);

  // check if the points_for_fix is longer in front than points
  const double lon_offset_to_prev_front =
    autoware::motion_utils::calcSignedArcLength(points, 0, front_fix_point.pose.position);
  if (0 < lon_offset_to_prev_front) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("autoware_path_smoother.trajectory_utils"),
      "Fixed point will not be inserted due to the error during calculation.");
    return std::nullopt;
  }

  const double dist = autoware::universe_utils::calcDistance2d(points.front(), front_fix_point);

  // check if deviation is not too large
  constexpr double max_lat_error = 3.0;
  if (max_lat_error < dist) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("autoware_path_smoother.trajectory_utils"),
      "New Fixed point is too far from points %f [m]", dist);
  }

  // update pose
  if (dist < delta_arc_length) {
    // only pose is updated
    points.front() = front_fix_point;
  } else {
    // add new front point
    T new_front_point;
    new_front_point = front_fix_point;
    points.insert(points.begin(), new_front_point);
  }

  return front_point_idx_for_fix;
}

void insertStopPoint(
  std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & input_stop_pose,
  const size_t stop_seg_idx);
}  // namespace trajectory_utils
}  // namespace autoware::path_smoother
#endif  // AUTOWARE__PATH_SMOOTHER__UTILS__TRAJECTORY_UTILS_HPP_
