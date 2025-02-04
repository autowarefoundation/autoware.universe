// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY__TRAJECTORY_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY__TRAJECTORY_HPP_

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/geometry/pose_deviation.hpp"
#include "autoware/universe_utils/math/constants.hpp"
#include "autoware/universe_utils/system/backtrace.hpp"

#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_utils
{
static inline rclcpp::Logger get_logger()
{
  constexpr const char * logger{"autoware_motion_utils.trajectory"};
  return rclcpp::get_logger(logger);
}

/**
 * @brief validate if points container is empty or not
 * @param points points of trajectory, path, ...
 */
template <class T>
void validateNonEmpty(const T & points)
{
  if (points.empty()) {
    autoware::universe_utils::print_backtrace();
    throw std::invalid_argument("[autoware_motion_utils] validateNonEmpty(): Points is empty.");
  }
}

extern template void validateNonEmpty<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> &);
extern template void validateNonEmpty<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &);
extern template void validateNonEmpty<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &);

/**
 * @brief validate a point is in a non-sharp angle between two points or not
 * @param point1 front point
 * @param point2 point to be validated
 * @param point3 back point
 */
template <class T>
void validateNonSharpAngle(
  const T & point1, const T & point2, const T & point3,
  const double angle_threshold = autoware::universe_utils::pi / 4)
{
  const auto p1 = autoware::universe_utils::getPoint(point1);
  const auto p2 = autoware::universe_utils::getPoint(point2);
  const auto p3 = autoware::universe_utils::getPoint(point3);

  const std::vector vec_1to2 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
  const std::vector vec_3to2 = {p2.x - p3.x, p2.y - p3.y, p2.z - p3.z};
  const auto product = std::inner_product(vec_1to2.begin(), vec_1to2.end(), vec_3to2.begin(), 0.0);

  const auto dist_1to2 = autoware::universe_utils::calcDistance3d(p1, p2);
  const auto dist_3to2 = autoware::universe_utils::calcDistance3d(p3, p2);

  constexpr double epsilon = 1e-3;
  if (std::cos(angle_threshold) < product / dist_1to2 / dist_3to2 + epsilon) {
    autoware::universe_utils::print_backtrace();
    throw std::invalid_argument(
      "[autoware_motion_utils] validateNonSharpAngle(): Too sharp angle.");
  }
}

/**
 * @brief checks whether a path of trajectory has forward driving direction
 * @param points points of trajectory, path, ...
 * @return (forward / backward) driving (true / false)
 */
template <class T>
std::optional<bool> isDrivingForward(const T & points)
{
  if (points.size() < 2) {
    return std::nullopt;
  }

  // check the first point direction
  const auto & first_pose = autoware::universe_utils::getPose(points.at(0));
  const auto & second_pose = autoware::universe_utils::getPose(points.at(1));

  return autoware::universe_utils::isDrivingForward(first_pose, second_pose);
}

extern template std::optional<bool>
isDrivingForward<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> &);
extern template std::optional<bool>
isDrivingForward<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &);
extern template std::optional<bool>
isDrivingForward<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &);

/**
 * @brief checks whether a path of trajectory has forward driving direction using its longitudinal
 * velocity
 * @param points_with_twist points of trajectory, path, ... (with velocity)
 * @return (forward / backward) driving (true, false, none "if velocity is zero")
 */
template <class T>
std::optional<bool> isDrivingForwardWithTwist(const T & points_with_twist)
{
  if (points_with_twist.empty()) {
    return std::nullopt;
  }
  if (points_with_twist.size() == 1) {
    if (0.0 < autoware::universe_utils::getLongitudinalVelocity(points_with_twist.front())) {
      return true;
    }
    if (0.0 > autoware::universe_utils::getLongitudinalVelocity(points_with_twist.front())) {
      return false;
    }
    return std::nullopt;
  }

  return isDrivingForward(points_with_twist);
}

extern template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> &);
extern template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &);
extern template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &);

/**
 * @brief remove overlapping points through points container.
 * Overlapping is determined by calculating the distance between 2 consecutive points.
 * If the distance between them is less than a threshold, they will be considered overlapping.
 * @param points points of trajectory, path, ...
 * @param start_idx index to start the overlap remove calculation from through the points
 * container. Indices before that index will be considered non-overlapping. Default = 0
 * @return points container without overlapping points
 */
template <class T>
T removeOverlapPoints(const T & points, const size_t start_idx = 0)
{
  if (points.size() < start_idx + 1) {
    return points;
  }

  T dst;
  dst.reserve(points.size());

  for (size_t i = 0; i <= start_idx; ++i) {
    dst.push_back(points.at(i));
  }

  constexpr double eps = 1.0E-08;
  for (size_t i = start_idx + 1; i < points.size(); ++i) {
    const auto prev_p = autoware::universe_utils::getPoint(dst.back());
    const auto curr_p = autoware::universe_utils::getPoint(points.at(i));
    if (std::abs(prev_p.x - curr_p.x) < eps && std::abs(prev_p.y - curr_p.y) < eps) {
      continue;
    }
    dst.push_back(points.at(i));
  }

  return dst;
}

extern template std::vector<autoware_planning_msgs::msg::PathPoint>
removeOverlapPoints<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const size_t start_idx = 0);
extern template std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
removeOverlapPoints<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t start_idx = 0);
extern template std::vector<autoware_planning_msgs::msg::TrajectoryPoint>
removeOverlapPoints<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t start_idx = 0);

/**
 * @brief search through points container from specified start and end indices about first matching
 * index of a zero longitudinal velocity point.
 * @param points_with_twist points of trajectory, path, ... (with velocity)
 * @param src_idx start index of the search
 * @param dst_idx end index of the search
 * @return first matching index of a zero velocity point inside the points container.
 */
template <class T>
std::optional<size_t> searchZeroVelocityIndex(
  const T & points_with_twist, const size_t src_idx, const size_t dst_idx)
{
  try {
    validateNonEmpty(points_with_twist);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return {};
  }

  constexpr double epsilon = 1e-3;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    if (std::fabs(points_with_twist.at(i).longitudinal_velocity_mps) < epsilon) {
      return i;
    }
  }

  return {};
}

extern template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const size_t src_idx, const size_t dst_idx);

/**
 * @brief search through points container from specified start index till end of points container
 * about first matching index of a zero longitudinal velocity point.
 * @param points_with_twist points of trajectory, path, ... (with velocity)
 * @param src_idx start index of the search
 * @return first matching index of a zero velocity point inside the points container.
 */
template <class T>
std::optional<size_t> searchZeroVelocityIndex(const T & points_with_twist, const size_t src_idx)
{
  try {
    validateNonEmpty(points_with_twist);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return {};
  }

  return searchZeroVelocityIndex(points_with_twist, src_idx, points_with_twist.size());
}

extern template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const size_t src_idx);

/**
 * @brief search through points container from its start to end about first matching index of a zero
 * longitudinal velocity point.
 * @param points_with_twist points of trajectory, path, ... (with velocity)
 * @return first matching index of a zero velocity point inside the points container.
 */
template <class T>
std::optional<size_t> searchZeroVelocityIndex(const T & points_with_twist)
{
  return searchZeroVelocityIndex(points_with_twist, 0, points_with_twist.size());
}

extern template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist);

/**
 * @brief find nearest point index through points container for a given point.
 * Finding nearest point is determined by looping through the points container,
 * and calculating the 2D squared distance between each point in the container and the given point.
 * The index of the point with minimum distance and yaw deviation comparing to the given point will
 * be returned.
 * @param points points of trajectory, path, ...
 * @param point given point
 * @return index of nearest point
 */
template <class T>
size_t findNearestIndex(const T & points, const geometry_msgs::msg::Point & point)
{
  validateNonEmpty(points);

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = autoware::universe_utils::calcSquaredDistance2d(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

extern template size_t findNearestIndex<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & point);
extern template size_t findNearestIndex<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & point);
extern template size_t findNearestIndex<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & point);

/**
 * @brief find nearest point index through points container for a given pose.
 * Finding nearest point is determined by looping through the points container,
 * and finding the nearest point to the given pose in terms of squared 2D distance and yaw
 * deviation. The index of the point with minimum distance and yaw deviation comparing to the given
 * pose will be returned.
 * @param points points of trajectory, path, ...
 * @param pose given pose
 * @param max_dist max distance used to get squared distance for finding the nearest point to given
 * pose
 * @param max_yaw max yaw used for finding nearest point to given pose
 * @return index of nearest point (index or none if not found)
 */
template <class T>
std::optional<size_t> findNearestIndex(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return {};
  }

  const double max_squared_dist = max_dist * max_dist;

  double min_squared_dist = std::numeric_limits<double>::max();
  bool is_nearest_found = false;
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto squared_dist = autoware::universe_utils::calcSquaredDistance2d(points.at(i), pose);
    if (squared_dist > max_squared_dist || squared_dist >= min_squared_dist) {
      continue;
    }

    const auto yaw = autoware::universe_utils::calcYawDeviation(
      autoware::universe_utils::getPose(points.at(i)), pose);
    if (std::fabs(yaw) > max_yaw) {
      continue;
    }

    min_squared_dist = squared_dist;
    min_idx = i;
    is_nearest_found = true;
  }

  if (is_nearest_found) {
    return min_idx;
  }
  return std::nullopt;
}

extern template std::optional<size_t>
findNearestIndex<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max());
extern template std::optional<size_t>
findNearestIndex<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max());
extern template std::optional<size_t>
findNearestIndex<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max());

/**
 * @brief calculate longitudinal offset (length along trajectory from seg_idx point to nearest point
 * to p_target on trajectory). If seg_idx point is after that nearest point, length is negative.
 * Segment is straight path between two continuous points of trajectory.
 * @param points points of trajectory, path, ...
 * @param seg_idx segment index of point at beginning of length
 * @param p_target target point at end of length
 * @param throw_exception flag to enable/disable exception throwing
 * @return signed length
 */
template <class T>
double calcLongitudinalOffsetToSegment(
  const T & points, const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  const bool throw_exception = false)
{
  if (seg_idx >= points.size() - 1) {
    const std::string error_message(
      "[autoware_motion_utils] " + std::string(__func__) +
      ": Failed to calculate longitudinal offset because the given segment index is out of the "
      "points size.");
    autoware::universe_utils::print_backtrace();
    if (throw_exception) {
      throw std::out_of_range(error_message);
    }
    RCLCPP_DEBUG(
      get_logger(),
      "%s Return NaN since no_throw option is enabled. The maintainer must check the code.",
      error_message.c_str());
    return std::nan("");
  }

  const auto overlap_removed_points = removeOverlapPoints(points, seg_idx);

  if (throw_exception) {
    validateNonEmpty(overlap_removed_points);
  } else {
    try {
      validateNonEmpty(overlap_removed_points);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(get_logger(), "%s", e.what());
      return std::nan("");
    }
  }

  if (seg_idx >= overlap_removed_points.size() - 1) {
    const std::string error_message(
      "[autoware_motion_utils] " + std::string(__func__) +
      ": Longitudinal offset calculation is not supported for the same points.");
    autoware::universe_utils::print_backtrace();
    if (throw_exception) {
      throw std::runtime_error(error_message);
    }
    RCLCPP_DEBUG(
      get_logger(),
      "%s Return NaN since no_throw option is enabled. The maintainer must check the code.",
      error_message.c_str());
    return std::nan("");
  }

  const auto p_front = autoware::universe_utils::getPoint(overlap_removed_points.at(seg_idx));
  const auto p_back = autoware::universe_utils::getPoint(overlap_removed_points.at(seg_idx + 1));

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0};

  return segment_vec.dot(target_vec) / segment_vec.norm();
}

extern template double
calcLongitudinalOffsetToSegment<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const size_t seg_idx,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception = false);
extern template double
calcLongitudinalOffsetToSegment<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points, const size_t seg_idx,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception = false);
extern template double
calcLongitudinalOffsetToSegment<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const size_t seg_idx,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception = false);

/**
 * @brief find nearest segment index to point.
 * Segment is straight path between two continuous points of trajectory.
 * When point is on a trajectory point whose index is nearest_idx, return nearest_idx - 1
 * @param points points of trajectory, path, ...
 * @param point point to which to find nearest segment index
 * @return nearest index
 */
template <class T>
size_t findNearestSegmentIndex(const T & points, const geometry_msgs::msg::Point & point)
{
  const size_t nearest_idx = findNearestIndex(points, point);

  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, point);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

extern template size_t findNearestSegmentIndex<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & point);
extern template size_t
findNearestSegmentIndex<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & point);
extern template size_t
findNearestSegmentIndex<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & point);

/**
 * @brief find nearest segment index to pose
 * Segment is straight path between two continuous points of trajectory.
 * When pose is on a trajectory point whose index is nearest_idx, return nearest_idx - 1
 * @param points points of trajectory, path, ..
 * @param pose pose to which to find nearest segment index
 * @param max_dist max distance used for finding the nearest index to given pose
 * @param max_yaw max yaw used for finding nearest index to given pose
 * @return nearest index
 */
template <class T>
std::optional<size_t> findNearestSegmentIndex(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())
{
  const auto nearest_idx = findNearestIndex(points, pose, max_dist, max_yaw);

  if (!nearest_idx) {
    return std::nullopt;
  }

  if (*nearest_idx == 0) {
    return 0;
  }
  if (*nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, *nearest_idx, pose.position);

  if (signed_length <= 0) {
    return *nearest_idx - 1;
  }

  return *nearest_idx;
}

extern template std::optional<size_t>
findNearestSegmentIndex<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max());
extern template std::optional<size_t>
findNearestSegmentIndex<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max());
extern template std::optional<size_t>
findNearestSegmentIndex<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max());

/**
 * @brief calculate lateral offset from p_target (length from p_target to trajectory) using given
 * segment index. Segment is straight path between two continuous points of trajectory.
 * @param points points of trajectory, path, ...
 * @param p_target target point
 * @param seg_idx segment index of point at beginning of length
 * @param throw_exception flag to enable/disable exception throwing
 * @return length (unsigned)
 */
template <class T>
double calcLateralOffset(
  const T & points, const geometry_msgs::msg::Point & p_target, const size_t seg_idx,
  const bool throw_exception = false)
{
  const auto overlap_removed_points = removeOverlapPoints(points, 0);

  if (throw_exception) {
    validateNonEmpty(overlap_removed_points);
  } else {
    try {
      validateNonEmpty(overlap_removed_points);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(
        get_logger(),
        "%s Return NaN since no_throw option is enabled. The maintainer must check the code.",
        e.what());
      return std::nan("");
    }
  }

  if (overlap_removed_points.size() == 1) {
    const std::string error_message(
      "[autoware_motion_utils] " + std::string(__func__) +
      ": Lateral offset calculation is not supported for the same points.");
    autoware::universe_utils::print_backtrace();
    if (throw_exception) {
      throw std::runtime_error(error_message);
    }
    RCLCPP_DEBUG(
      get_logger(),
      "%s Return NaN since no_throw option is enabled. The maintainer must check the code.",
      error_message.c_str());
    return std::nan("");
  }

  const auto p_indices = overlap_removed_points.size() - 2;
  const auto p_front_idx = (p_indices > seg_idx) ? seg_idx : p_indices;
  const auto p_back_idx = p_front_idx + 1;

  const auto p_front = autoware::universe_utils::getPoint(overlap_removed_points.at(p_front_idx));
  const auto p_back = autoware::universe_utils::getPoint(overlap_removed_points.at(p_back_idx));

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0.0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0.0};

  const Eigen::Vector3d cross_vec = segment_vec.cross(target_vec);
  return cross_vec(2) / segment_vec.norm();
}

extern template double calcLateralOffset<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & p_target, const size_t seg_idx,
  const bool throw_exception = false);
extern template double
calcLateralOffset<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & p_target, const size_t seg_idx,
  const bool throw_exception = false);
extern template double calcLateralOffset<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & p_target, const size_t seg_idx,
  const bool throw_exception = false);

/**
 * @brief calculate lateral offset from p_target (length from p_target to trajectory).
 * The function gets the nearest segment index between the points of trajectory and the given target
 * point, then uses that segment index to calculate lateral offset. Segment is straight path between
 * two continuous points of trajectory.
 * @param points points of trajectory, path, ...
 * @param p_target target point
 * @param throw_exception flag to enable/disable exception throwing
 * @return length (unsigned)
 */
template <class T>
double calcLateralOffset(
  const T & points, const geometry_msgs::msg::Point & p_target, const bool throw_exception = false)
{
  const auto overlap_removed_points = removeOverlapPoints(points, 0);

  if (throw_exception) {
    validateNonEmpty(overlap_removed_points);
  } else {
    try {
      validateNonEmpty(overlap_removed_points);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(
        get_logger(),
        "%s Return NaN since no_throw option is enabled. The maintainer must check the code.",
        e.what());
      return std::nan("");
    }
  }

  if (overlap_removed_points.size() == 1) {
    const std::string error_message(
      "[autoware_motion_utils] " + std::string(__func__) +
      ": Lateral offset calculation is not supported for the same points.");
    autoware::universe_utils::print_backtrace();
    if (throw_exception) {
      throw std::runtime_error(error_message);
    }
    RCLCPP_DEBUG(
      get_logger(),
      "%s Return NaN since no_throw option is enabled. The maintainer must check the code.",
      error_message.c_str());
    return std::nan("");
  }

  const size_t seg_idx = findNearestSegmentIndex(overlap_removed_points, p_target);
  return calcLateralOffset(points, p_target, seg_idx, throw_exception);
}

extern template double calcLateralOffset<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception = false);
extern template double
calcLateralOffset<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception = false);
extern template double calcLateralOffset<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception = false);

/**
 * @brief calculate length of 2D distance between two points, specified by start and end points
 * indicies through points container.
 * @param points points of trajectory, path, ...
 * @param src_idx index of start point
 * @param dst_idx index of end point
 * @return length of distance between two points.
 * Length is positive if dst_idx is greater that src_idx (i.e. after it in trajectory, path, ...)
 * and negative otherwise.
 */
template <class T>
double calcSignedArcLength(const T & points, const size_t src_idx, const size_t dst_idx)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return 0.0;
  }

  if (src_idx > dst_idx) {
    return -calcSignedArcLength(points, dst_idx, src_idx);
  }

  double dist_sum = 0.0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    dist_sum += autoware::universe_utils::calcDistance2d(points.at(i), points.at(i + 1));
  }
  return dist_sum;
}

extern template double calcSignedArcLength<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const size_t dst_idx);
extern template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points, const size_t src_idx,
  const size_t dst_idx);
extern template double
calcSignedArcLength<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const size_t src_idx,
  const size_t dst_idx);

/**
 * @brief Computes the partial sums of the elements in the sub-ranges of the range [src_idx,
 * dst_idx) and return these sum as vector
 * @param points points of trajectory, path, ...
 * @param src_idx index of start point
 * @param dst_idx index of end point
 * @return partial sums container
 */
template <class T>
std::vector<double> calcSignedArcLengthPartialSum(
  const T & points, const size_t src_idx, const size_t dst_idx)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return {};
  }

  if (src_idx + 1 > dst_idx) {
    auto copied = points;
    std::reverse(copied.begin(), copied.end());
    return calcSignedArcLengthPartialSum(points, dst_idx, src_idx);
  }

  std::vector<double> partial_dist;
  partial_dist.reserve(dst_idx - src_idx);

  double dist_sum = 0.0;
  partial_dist.push_back(dist_sum);
  for (size_t i = src_idx; i < dst_idx - 1; ++i) {
    dist_sum += autoware::universe_utils::calcDistance2d(points.at(i), points.at(i + 1));
    partial_dist.push_back(dist_sum);
  }
  return partial_dist;
}

extern template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const size_t dst_idx);
extern template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points, const size_t src_idx,
  const size_t dst_idx);
extern template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const size_t src_idx,
  const size_t dst_idx);

/**
 * @brief calculate length of 2D distance between two points, specified by start point and end point
 * index of points container.
 * @param points points of trajectory, path, ...
 * @param src_point start point
 * @param dst_idx index of end point
 * @return length of distance between two points.
 * Length is positive if destination point associated to dst_idx is greater that src_idx (i.e. after
 * it in trajectory, path, ...) and negative otherwise.
 */
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t dst_idx)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return 0.0;
  }

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return signed_length_on_traj - signed_length_src_offset;
}

extern template double calcSignedArcLength<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t dst_idx);
extern template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &,
  const geometry_msgs::msg::Point & src_point, const size_t dst_idx);
extern template double
calcSignedArcLength<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &,
  const geometry_msgs::msg::Point & src_point, const size_t dst_idx);

/**
 * @brief calculate length of 2D distance between two points, specified by start index of points
 * container and end point.
 * @param points points of trajectory, path, ...
 * @param src_idx index of start point
 * @param dst_point end point
 * @return length of distance between two points
 * Length is positive if destination point is greater that source point associated to src_idx (i.e.
 * after it in trajectory, path, ...) and negative otherwise.
 */
template <class T>
double calcSignedArcLength(
  const T & points, const size_t src_idx, const geometry_msgs::msg::Point & dst_point)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return 0.0;
  }

  return -calcSignedArcLength(points, dst_point, src_idx);
}

extern template double calcSignedArcLength<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const geometry_msgs::msg::Point & dst_point);
extern template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points, const size_t src_idx,
  const geometry_msgs::msg::Point & dst_point);
extern template double
calcSignedArcLength<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const size_t src_idx,
  const geometry_msgs::msg::Point & dst_point);

/**
 * @brief calculate length of 2D distance between two points, specified by start point and end
 * point.
 * @param points points of trajectory, path, ...
 * @param src_point start point
 * @param dst_point end point
 * @return length of distance between two points.
 * Length is positive if destination point is greater that source point (i.e. after it in
 * trajectory, path, ...) and negative otherwise.
 *
 */
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point,
  const geometry_msgs::msg::Point & dst_point)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return 0.0;
  }

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const size_t dst_seg_idx = findNearestSegmentIndex(points, dst_point);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_seg_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);
  const double signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj - signed_length_src_offset + signed_length_dst_offset;
}

extern template double calcSignedArcLength<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const geometry_msgs::msg::Point & dst_point);
extern template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const geometry_msgs::msg::Point & dst_point);
extern template double
calcSignedArcLength<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const geometry_msgs::msg::Point & dst_point);

/**
 * @brief calculate length of 2D distance for whole points container, from its start to its end.
 * @param points points of trajectory, path, ...
 * @return length of 2D distance for points container
 */
template <class T>
double calcArcLength(const T & points)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return 0.0;
  }

  return calcSignedArcLength(points, 0, points.size() - 1);
}

extern template double calcArcLength<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points);
extern template double calcArcLength<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points);
extern template double calcArcLength<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points);

/**
 * @brief calculate curvature through points container.
 * The method used for calculating the curvature is using 3 consecutive points through the points
 * container. Then the curvature is the reciprocal of the radius of the circle that passes through
 * these three points.
 * @details more details here : https://en.wikipedia.org/wiki/Menger_curvature
 * @param points points of trajectory, path, ...
 * @return calculated curvature container through points container
 */
template <class T>
std::vector<double> calcCurvature(const T & points)
{
  std::vector<double> curvature_vec(points.size(), 0.0);
  if (points.size() < 3) {
    return curvature_vec;
  }

  for (size_t i = 1; i < points.size() - 1; ++i) {
    const auto p1 = autoware::universe_utils::getPoint(points.at(i - 1));
    const auto p2 = autoware::universe_utils::getPoint(points.at(i));
    const auto p3 = autoware::universe_utils::getPoint(points.at(i + 1));
    curvature_vec.at(i) = (autoware::universe_utils::calcCurvature(p1, p2, p3));
  }
  curvature_vec.at(0) = curvature_vec.at(1);
  curvature_vec.at(curvature_vec.size() - 1) = curvature_vec.at(curvature_vec.size() - 2);

  return curvature_vec;
}

extern template std::vector<double>
calcCurvature<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points);
extern template std::vector<double>
calcCurvature<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points);
extern template std::vector<double>
calcCurvature<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points);

/**
 * @brief calculate curvature through points container and length of 2d distance for segment used
 * for curvature calculation. The method used for calculating the curvature is using 3 consecutive
 * points through the points container. Then the curvature is the reciprocal of the radius of the
 * circle that passes through these three points. Then length of 2D distance of these points is
 * calculated
 * @param points points of trajectory, path, ...
 * @return Container of pairs, calculated curvature and length of 2D distance for segment used for
 * curvature calculation
 */
template <class T>
std::vector<std::pair<double, std::pair<double, double>>> calcCurvatureAndSegmentLength(
  const T & points)
{
  // segment length is pair of segment length between {p1, p2} and {p2, p3}
  std::vector<std::pair<double, std::pair<double, double>>> curvature_and_segment_length_vec;
  curvature_and_segment_length_vec.reserve(points.size());
  curvature_and_segment_length_vec.emplace_back(0.0, std::make_pair(0.0, 0.0));
  for (size_t i = 1; i < points.size() - 1; ++i) {
    const auto p1 = autoware::universe_utils::getPoint(points.at(i - 1));
    const auto p2 = autoware::universe_utils::getPoint(points.at(i));
    const auto p3 = autoware::universe_utils::getPoint(points.at(i + 1));
    const double curvature = autoware::universe_utils::calcCurvature(p1, p2, p3);

    // The first point has only the next point, so put the distance to that point.
    // In other words, assign the first segment length at the second point to the
    // second_segment_length at the first point.
    if (i == 1) {
      curvature_and_segment_length_vec.at(0).second.second =
        autoware::universe_utils::calcDistance2d(p1, p2);
    }

    // The second_segment_length of the previous point and the first segment length of the current
    // point are equal.
    const std::pair<double, double> arc_length{
      curvature_and_segment_length_vec.back().second.second,
      autoware::universe_utils::calcDistance2d(p2, p3)};

    curvature_and_segment_length_vec.emplace_back(curvature, arc_length);
  }

  // set to the last point
  curvature_and_segment_length_vec.emplace_back(
    0.0, std::make_pair(curvature_and_segment_length_vec.back().second.second, 0.0));

  return curvature_and_segment_length_vec;
}

extern template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points);
extern template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points);
extern template std::vector<std::pair<double, std::pair<double, double>>>
calcCurvatureAndSegmentLength<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points);

/**
 * @brief calculate length of 2D distance between given start point index in points container and
 * first point in container with zero longitudinal velocity
 * @param points_with_twist points of trajectory, path, ... (with velocity)
 * @return Length of 2D distance between start point index in points container and first point in
 * container with zero longitudinal velocity
 */
template <class T>
std::optional<double> calcDistanceToForwardStopPoint(
  const T & points_with_twist, const size_t src_idx = 0)
{
  try {
    validateNonEmpty(points_with_twist);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return {};
  }

  const auto closest_stop_idx =
    searchZeroVelocityIndex(points_with_twist, src_idx, points_with_twist.size());
  if (!closest_stop_idx) {
    return std::nullopt;
  }

  return std::max(0.0, calcSignedArcLength(points_with_twist, src_idx, *closest_stop_idx));
}

extern template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const size_t src_idx = 0);

/**
 * @brief calculate the point offset from source point index along the trajectory (or path) (points
 * container)
 * @param points points of trajectory, path, ...
 * @param src_idx index of source point
 * @param offset length of offset from source point
 * @param throw_exception flag to enable/disable exception throwing
 * @return offset point
 */
template <class T>
std::optional<geometry_msgs::msg::Point> calcLongitudinalOffsetPoint(
  const T & points, const size_t src_idx, const double offset, const bool throw_exception = false)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return {};
  }

  if (points.size() - 1 < src_idx) {
    const std::string error_message(
      "[autoware_motion_utils] " + std::string(__func__) +
      " error: The given source index is out of the points size. Failed to calculate longitudinal "
      "offset.");
    autoware::universe_utils::print_backtrace();
    if (throw_exception) {
      throw std::out_of_range(error_message);
    }
    RCLCPP_DEBUG(
      get_logger(),
      "%s Return NaN since no_throw option is enabled. The maintainer must check the code.",
      error_message.c_str());
    return {};
  }

  if (points.size() == 1) {
    return {};
  }

  if (src_idx + 1 == points.size() && offset == 0.0) {
    return autoware::universe_utils::getPoint(points.at(src_idx));
  }

  if (offset < 0.0) {
    auto reverse_points = points;
    std::reverse(reverse_points.begin(), reverse_points.end());
    return calcLongitudinalOffsetPoint(
      reverse_points, reverse_points.size() - src_idx - 1, -offset);
  }

  double dist_sum = 0.0;

  for (size_t i = src_idx; i < points.size() - 1; ++i) {
    const auto & p_front = points.at(i);
    const auto & p_back = points.at(i + 1);

    const auto dist_segment = autoware::universe_utils::calcDistance2d(p_front, p_back);
    dist_sum += dist_segment;

    const auto dist_res = offset - dist_sum;
    if (dist_res <= 0.0) {
      return autoware::universe_utils::calcInterpolatedPoint(
        p_back, p_front, std::abs(dist_res / dist_segment));
    }
  }

  // not found (out of range)
  return {};
}

extern template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const double offset, const bool throw_exception = false);
extern template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points, const size_t src_idx,
  const double offset, const bool throw_exception = false);
extern template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const size_t src_idx,
  const double offset, const bool throw_exception = false);

/**
 * @brief calculate the point offset from source point along the trajectory (or path) (points
 * container)
 * @param points points of trajectory, path, ...
 * @param src_point source point
 * @param offset length of offset from source point
 * @return offset point
 */
template <class T>
std::optional<geometry_msgs::msg::Point> calcLongitudinalOffsetPoint(
  const T & points, const geometry_msgs::msg::Point & src_point, const double offset)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "Failed to calculate longitudinal offset: %s", e.what());
    return {};
  }

  if (offset < 0.0) {
    auto reverse_points = points;
    std::reverse(reverse_points.begin(), reverse_points.end());
    return calcLongitudinalOffsetPoint(reverse_points, src_point, -offset);
  }

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return calcLongitudinalOffsetPoint(points, src_seg_idx, offset + signed_length_src_offset);
}

extern template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const double offset);
extern template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const double offset);
extern template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const double offset);

/**
 * @brief calculate the point offset from source point index along the trajectory (or path) (points
 * container)
 * @param points points of trajectory, path, ...
 * @param src_idx index of source point
 * @param offset length of offset from source point
 * @param set_orientation_from_position_direction set orientation by spherical interpolation if
 * false
 * @return offset pose
 */
template <class T>
std::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const T & points, const size_t src_idx, const double offset,
  const bool set_orientation_from_position_direction = true, const bool throw_exception = false)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "Failed to calculate longitudinal offset: %s", e.what());
    return {};
  }

  if (points.size() - 1 < src_idx) {
    const std::string error_message(
      "[autoware_motion_utils] " + std::string(__func__) +
      " error: The given source index is out of the points size. Failed to calculate longitudinal "
      "offset.");
    autoware::universe_utils::print_backtrace();
    if (throw_exception) {
      throw std::out_of_range(error_message);
    }
    RCLCPP_DEBUG(get_logger(), "%s", error_message.c_str());
    return {};
  }

  if (points.size() == 1) {
    RCLCPP_DEBUG(get_logger(), "Failed to calculate longitudinal offset: points size is one.");
    return {};
  }

  if (src_idx + 1 == points.size() && offset == 0.0) {
    return autoware::universe_utils::getPose(points.at(src_idx));
  }

  if (offset < 0.0) {
    auto reverse_points = points;
    std::reverse(reverse_points.begin(), reverse_points.end());

    double dist_sum = 0.0;

    for (size_t i = reverse_points.size() - src_idx - 1; i < reverse_points.size() - 1; ++i) {
      const auto & p_front = reverse_points.at(i);
      const auto & p_back = reverse_points.at(i + 1);

      const auto dist_segment = autoware::universe_utils::calcDistance2d(p_front, p_back);
      dist_sum += dist_segment;

      const auto dist_res = -offset - dist_sum;
      if (dist_res <= 0.0) {
        return autoware::universe_utils::calcInterpolatedPose(
          p_back, p_front, std::abs(dist_res / dist_segment),
          set_orientation_from_position_direction);
      }
    }
  } else {
    double dist_sum = 0.0;

    for (size_t i = src_idx; i < points.size() - 1; ++i) {
      const auto & p_front = points.at(i);
      const auto & p_back = points.at(i + 1);

      const auto dist_segment = autoware::universe_utils::calcDistance2d(p_front, p_back);
      dist_sum += dist_segment;

      const auto dist_res = offset - dist_sum;
      if (dist_res <= 0.0) {
        return autoware::universe_utils::calcInterpolatedPose(
          p_front, p_back, 1.0 - std::abs(dist_res / dist_segment),
          set_orientation_from_position_direction);
      }
    }
  }

  // not found (out of range)
  return {};
}

extern template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction = true,
  const bool throw_exception = false);
extern template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction = true,
  const bool throw_exception = false);
extern template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction = true,
  const bool throw_exception = false);

/**
 * @brief calculate the point offset from source point along the trajectory (or path) (points
 * container)
 * @param points points of trajectory, path, ...
 * @param src_point source point
 * @param offset length of offset from source point
 * @param set_orientation_from_position_direction set orientation by spherical interpolation if
 * false
 * @return offset pose
 */
template <class T>
std::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const T & points, const geometry_msgs::msg::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction = true)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return {};
  }

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return calcLongitudinalOffsetPose(
    points, src_seg_idx, offset + signed_length_src_offset,
    set_orientation_from_position_direction);
}

extern template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction = true);
extern template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction = true);
extern template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction = true);

/**
 * @brief insert a point in points container (trajectory, path, ...) using segment id
 * @param seg_idx segment index of point at beginning of length
 * @param p_target point to be inserted
 * @param points output points of trajectory, path, ...
 * @param overlap_threshold distance threshold, used to check if the inserted point is between start
 * and end of nominated segment to be added in.
 * @return index of segment id, where point is inserted
 */
template <class T>
std::optional<size_t> insertTargetPoint(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target, T & points,
  const double overlap_threshold = 1e-3)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return {};
  }

  // invalid segment index
  if (seg_idx + 1 >= points.size()) {
    return {};
  }

  const auto p_front = autoware::universe_utils::getPoint(points.at(seg_idx));
  const auto p_back = autoware::universe_utils::getPoint(points.at(seg_idx + 1));

  try {
    validateNonSharpAngle(p_front, p_target, p_back);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return {};
  }

  const auto overlap_with_front =
    autoware::universe_utils::calcDistance2d(p_target, p_front) < overlap_threshold;
  const auto overlap_with_back =
    autoware::universe_utils::calcDistance2d(p_target, p_back) < overlap_threshold;

  const auto is_driving_forward = isDrivingForward(points);
  if (!is_driving_forward) {
    return {};
  }

  geometry_msgs::msg::Pose target_pose;
  {
    const auto p_base = is_driving_forward.value() ? p_back : p_front;
    const auto pitch = autoware::universe_utils::calcElevationAngle(p_target, p_base);
    const auto yaw = autoware::universe_utils::calcAzimuthAngle(p_target, p_base);

    target_pose.position = p_target;
    target_pose.orientation = autoware::universe_utils::createQuaternionFromRPY(0.0, pitch, yaw);
  }

  auto p_insert = points.at(seg_idx);
  autoware::universe_utils::setPose(target_pose, p_insert);

  geometry_msgs::msg::Pose base_pose;
  {
    const auto p_base = is_driving_forward.value() ? p_front : p_back;
    const auto pitch = autoware::universe_utils::calcElevationAngle(p_base, p_target);
    const auto yaw = autoware::universe_utils::calcAzimuthAngle(p_base, p_target);

    base_pose.position = autoware::universe_utils::getPoint(p_base);
    base_pose.orientation = autoware::universe_utils::createQuaternionFromRPY(0.0, pitch, yaw);
  }

  if (!overlap_with_front && !overlap_with_back) {
    if (is_driving_forward.value()) {
      autoware::universe_utils::setPose(base_pose, points.at(seg_idx));
    } else {
      autoware::universe_utils::setPose(base_pose, points.at(seg_idx + 1));
    }
    points.insert(points.begin() + seg_idx + 1, p_insert);
    return seg_idx + 1;
  }

  if (overlap_with_back) {
    return seg_idx + 1;
  }

  return seg_idx;
}

extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const double overlap_threshold = 1e-3);

/**
 * @brief insert a point in points container (trajectory, path, ...) using length of point to be
 * inserted
 * @param insert_point_length length to insert point from the beginning of the points
 * @param p_target point to be inserted
 * @param points output points of trajectory, path, ...
 * @param overlap_threshold distance threshold, used to check if the inserted point is between start
 * and end of nominated segment to be added in.
 * @return index of segment id, where point is inserted
 */
template <class T>
std::optional<size_t> insertTargetPoint(
  const double insert_point_length, const geometry_msgs::msg::Point & p_target, T & points,
  const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points);

  if (insert_point_length < 0.0) {
    return std::nullopt;
  }

  // Get Nearest segment index
  std::optional<size_t> segment_idx = std::nullopt;
  for (size_t i = 1; i < points.size(); ++i) {
    // TODO(Mamoru Sobue): find accumulated sum beforehand
    const double length = calcSignedArcLength(points, 0, i);
    if (insert_point_length <= length) {
      segment_idx = i - 1;
      break;
    }
  }

  if (!segment_idx) {
    return std::nullopt;
  }

  return insertTargetPoint(*segment_idx, p_target, points, overlap_threshold);
}

extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const double insert_point_length, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const double insert_point_length, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const double insert_point_length, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const double overlap_threshold = 1e-3);

/**
 * @brief insert a point in points container (trajectory, path, ...) using segment index and length
 * of point to be inserted
 * @param src_segment_idx source segment index on the trajectory
 * @param insert_point_length length to insert point from the beginning of the points
 * @param points output points of trajectory, path, ...
 * @param overlap_threshold distance threshold, used to check if the inserted point is between start
 * and end of nominated segment to be added in.
 * @return index of insert point
 */
template <class T>
std::optional<size_t> insertTargetPoint(
  const size_t src_segment_idx, const double insert_point_length, T & points,
  const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points);

  if (src_segment_idx >= points.size() - 1) {
    return std::nullopt;
  }

  // Get Nearest segment index
  std::optional<size_t> segment_idx = std::nullopt;
  if (0.0 <= insert_point_length) {
    for (size_t i = src_segment_idx + 1; i < points.size(); ++i) {
      const double length = calcSignedArcLength(points, src_segment_idx, i);
      if (insert_point_length <= length) {
        segment_idx = i - 1;
        break;
      }
    }
  } else {
    for (int i = src_segment_idx - 1; 0 <= i; --i) {
      const double length = calcSignedArcLength(points, src_segment_idx, i);
      if (length <= insert_point_length) {
        segment_idx = i;
        break;
      }
    }
  }

  if (!segment_idx) {
    return std::nullopt;
  }

  // Get Target Point
  const double segment_length = calcSignedArcLength(points, *segment_idx, *segment_idx + 1);
  const double target_length =
    insert_point_length - calcSignedArcLength(points, src_segment_idx, *segment_idx);
  const double ratio = std::clamp(target_length / segment_length, 0.0, 1.0);
  const auto p_target = autoware::universe_utils::calcInterpolatedPoint(
    autoware::universe_utils::getPoint(points.at(*segment_idx)),
    autoware::universe_utils::getPoint(points.at(*segment_idx + 1)), ratio);

  return insertTargetPoint(*segment_idx, p_target, points, overlap_threshold);
}

extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const double overlap_threshold = 1e-3);

/**
 * @brief Insert a target point from a source pose on the trajectory
 * @param src_pose source pose on the trajectory
 * @param insert_point_length length to insert point from the beginning of the points
 * @param points output points of trajectory, path, ...
 * @param max_dist max distance, used to search for nearest segment index in points container to the
 * given source pose
 * @param max_yaw max yaw, used to search for nearest segment index in points container to the given
 * source pose
 * @param overlap_threshold distance threshold, used to check if the inserted point is between start
 * and end of nominated segment to be added in.
 * @return index of insert point
 */
template <class T>
std::optional<size_t> insertTargetPoint(
  const geometry_msgs::msg::Pose & src_pose, const double insert_point_length, T & points,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points);

  if (insert_point_length < 0.0) {
    return std::nullopt;
  }

  const auto nearest_segment_idx = findNearestSegmentIndex(points, src_pose, max_dist, max_yaw);
  if (!nearest_segment_idx) {
    return std::nullopt;
  }

  const double offset_length =
    calcLongitudinalOffsetToSegment(points, *nearest_segment_idx, src_pose.position);

  return insertTargetPoint(
    *nearest_segment_idx, insert_point_length + offset_length, points, overlap_threshold);
}

extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const geometry_msgs::msg::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const geometry_msgs::msg::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertTargetPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const geometry_msgs::msg::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3);

/**
 * @brief Insert stop point from the source segment index
 * @param src_segment_idx start segment index on the trajectory
 * @param distance_to_stop_point distance to stop point from the source index
 * @param points_with_twist output points of trajectory, path, ... (with velocity)
 * @param overlap_threshold distance threshold, used to check if the inserted point is between start
 * and end of nominated segment to be added in.
 * @return index of stop point
 */
template <class T>
std::optional<size_t> insertStopPoint(
  const size_t src_segment_idx, const double distance_to_stop_point, T & points_with_twist,
  const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points_with_twist);

  if (distance_to_stop_point < 0.0 || src_segment_idx >= points_with_twist.size() - 1) {
    return std::nullopt;
  }

  const auto stop_idx = insertTargetPoint(
    src_segment_idx, distance_to_stop_point, points_with_twist, overlap_threshold);
  if (!stop_idx) {
    return std::nullopt;
  }

  for (size_t i = *stop_idx; i < points_with_twist.size(); ++i) {
    autoware::universe_utils::setLongitudinalVelocity(0.0, points_with_twist.at(i));
  }

  return stop_idx;
}

extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::msg::PathPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points_with_twist,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);

/**
 * @brief Insert stop point from the front point of the path
 * @param distance_to_stop_point distance to stop point from the front point of the path
 * @param points_with_twist output points of trajectory, path, ... (with velocity)
 * @param overlap_threshold distance threshold, used to check if the inserted point is between start
 * and end of nominated segment to be added in.
 * @return index of stop point
 */
template <class T>
std::optional<size_t> insertStopPoint(
  const double distance_to_stop_point, T & points_with_twist, const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points_with_twist);

  if (distance_to_stop_point < 0.0) {
    return std::nullopt;
  }

  double accumulated_length = 0;
  for (size_t i = 0; i < points_with_twist.size() - 1; ++i) {
    const auto curr_pose = autoware::universe_utils::getPose(points_with_twist.at(i));
    const auto next_pose = autoware::universe_utils::getPose(points_with_twist.at(i + 1));
    const double length = autoware::universe_utils::calcDistance2d(curr_pose, next_pose);
    if (accumulated_length + length + overlap_threshold > distance_to_stop_point) {
      const double insert_length = distance_to_stop_point - accumulated_length;
      return insertStopPoint(i, insert_length, points_with_twist, overlap_threshold);
    }
    accumulated_length += length;
  }

  return std::nullopt;
}

extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::msg::PathPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const double distance_to_stop_point,
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points_with_twist,
  const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);

/**
 * @brief Insert Stop point from the source pose
 * @param src_pose source pose
 * @param distance_to_stop_point  distance to stop point from the src point
 * @param points_with_twist output points of trajectory, path, ... (with velocity)
 * @param max_dist max distance, used to search for nearest segment index in points container to the
 * given source pose
 * @param max_yaw max yaw, used to search for nearest segment index in points container to the given
 * source pose
 * @param overlap_threshold distance threshold, used to check if the inserted point is between start
 * and end of nominated segment to be added in.
 * @return index of stop point
 */
template <class T>
std::optional<size_t> insertStopPoint(
  const geometry_msgs::msg::Pose & src_pose, const double distance_to_stop_point,
  T & points_with_twist, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points_with_twist);

  if (distance_to_stop_point < 0.0) {
    return std::nullopt;
  }

  const auto stop_idx = insertTargetPoint(
    src_pose, distance_to_stop_point, points_with_twist, max_dist, max_yaw, overlap_threshold);

  if (!stop_idx) {
    return std::nullopt;
  }

  for (size_t i = *stop_idx; i < points_with_twist.size(); ++i) {
    autoware::universe_utils::setLongitudinalVelocity(0.0, points_with_twist.at(i));
  }

  return stop_idx;
}

extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const geometry_msgs::msg::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::msg::PathPoint> & points_with_twist,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const geometry_msgs::msg::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points_with_twist,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3);
extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const geometry_msgs::msg::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3);

/**
 * @brief Insert Stop point that is in the stop segment index
 * @param stop_seg_idx segment index of the stop pose
 * @param stop_point stop point
 * @param points_with_twist output points of trajectory, path, ... (with velocity)
 * @param overlap_threshold distance threshold, used to check if the inserted point is between start
 * and end of nominated segment to be added in.
 * @return index of stop point
 */
template <class T>
std::optional<size_t> insertStopPoint(
  const size_t stop_seg_idx, const geometry_msgs::msg::Point & stop_point, T & points_with_twist,
  const double overlap_threshold = 1e-3)
{
  const auto insert_idx = autoware::motion_utils::insertTargetPoint(
    stop_seg_idx, stop_point, points_with_twist, overlap_threshold);

  if (!insert_idx) {
    return std::nullopt;
  }

  for (size_t i = insert_idx.value(); i < points_with_twist.size(); ++i) {
    autoware::universe_utils::setLongitudinalVelocity(0.0, points_with_twist.at(i));
  }

  return insert_idx;
}

extern template std::optional<size_t>
insertStopPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const size_t stop_seg_idx, const geometry_msgs::msg::Point & stop_point,
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);

/**
 * @brief Insert deceleration point from the source pose
 * @param src_point source point
 * @param distance_to_decel_point  distance to deceleration point from the src point
 * @param velocity velocity of stop point
 * @param points_with_twist output points of trajectory, path, ... (with velocity)
 */
template <class T>
std::optional<size_t> insertDecelPoint(
  const geometry_msgs::msg::Point & src_point, const double distance_to_decel_point,
  const double velocity, T & points_with_twist)
{
  const auto decel_point =
    calcLongitudinalOffsetPoint(points_with_twist, src_point, distance_to_decel_point);

  if (!decel_point) {
    return {};
  }

  const auto seg_idx = findNearestSegmentIndex(points_with_twist, decel_point.value());
  const auto insert_idx = insertTargetPoint(seg_idx, decel_point.value(), points_with_twist);

  if (!insert_idx) {
    return {};
  }

  for (size_t i = insert_idx.value(); i < points_with_twist.size(); ++i) {
    const auto & original_velocity =
      autoware::universe_utils::getLongitudinalVelocity(points_with_twist.at(i));
    autoware::universe_utils::setLongitudinalVelocity(
      std::min(original_velocity, velocity), points_with_twist.at(i));
  }

  return insert_idx;
}

extern template std::optional<size_t>
insertDecelPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const geometry_msgs::msg::Point & src_point, const double distance_to_decel_point,
  const double velocity,
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist);

/**
 * @brief Insert orientation to each point in points container (trajectory, path, ...)
 * @param points points of trajectory, path, ... (input / output)
 * @param is_driving_forward  flag indicating the order of points is forward or backward
 */
template <class T>
void insertOrientation(T & points, const bool is_driving_forward)
{
  if (is_driving_forward) {
    for (size_t i = 0; i < points.size() - 1; ++i) {
      const auto & src_point = autoware::universe_utils::getPoint(points.at(i));
      const auto & dst_point = autoware::universe_utils::getPoint(points.at(i + 1));
      const double pitch = autoware::universe_utils::calcElevationAngle(src_point, dst_point);
      const double yaw = autoware::universe_utils::calcAzimuthAngle(src_point, dst_point);
      autoware::universe_utils::setOrientation(
        autoware::universe_utils::createQuaternionFromRPY(0.0, pitch, yaw), points.at(i));
      if (i == points.size() - 2) {
        // Terminal orientation is same as the point before it
        autoware::universe_utils::setOrientation(
          autoware::universe_utils::getPose(points.at(i)).orientation, points.at(i + 1));
      }
    }
  } else {
    for (size_t i = points.size() - 1; i >= 1; --i) {
      const auto & src_point = autoware::universe_utils::getPoint(points.at(i));
      const auto & dst_point = autoware::universe_utils::getPoint(points.at(i - 1));
      const double pitch = autoware::universe_utils::calcElevationAngle(src_point, dst_point);
      const double yaw = autoware::universe_utils::calcAzimuthAngle(src_point, dst_point);
      autoware::universe_utils::setOrientation(
        autoware::universe_utils::createQuaternionFromRPY(0.0, pitch, yaw), points.at(i));
    }
    // Initial orientation is same as the point after it
    autoware::universe_utils::setOrientation(
      autoware::universe_utils::getPose(points.at(1)).orientation, points.at(0));
  }
}

extern template void insertOrientation<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  std::vector<autoware_planning_msgs::msg::PathPoint> & points, const bool is_driving_forward);
extern template void insertOrientation<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const bool is_driving_forward);
extern template void insertOrientation<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const bool is_driving_forward);

/**
 * @brief Remove points with invalid orientation differences from a given points container
 * (trajectory, path, ...). Check the difference between the angles of two points and the difference
 * between the azimuth angle between the two points and the angle of the next point.
 * @param points Points of trajectory, path, or other point container (input / output)
 * @param max_yaw_diff Maximum acceptable yaw angle difference between two consecutive points in
 * radians (default: M_PI_2)
 */
template <class T>
void removeFirstInvalidOrientationPoints(T & points, const double max_yaw_diff = M_PI_2)
{
  for (auto itr = points.begin(); std::next(itr) != points.end();) {
    const auto p1 = autoware::universe_utils::getPose(*itr);
    const auto p2 = autoware::universe_utils::getPose(*std::next(itr));
    const double yaw1 = tf2::getYaw(p1.orientation);
    const double yaw2 = tf2::getYaw(p2.orientation);

    if (
      max_yaw_diff < std::abs(autoware::universe_utils::normalizeRadian(yaw1 - yaw2)) ||
      !autoware::universe_utils::isDrivingForward(p1, p2)) {
      points.erase(std::next(itr));
      return;
    } else {
      itr++;
    }
  }
}

/**
 * @brief calculate length of 2D distance between two points, specified by start point and end
 * point with their segment indices in points container
 * @param points points of trajectory, path, ...
 * @param src_point start point
 * @param src_seg_idx index of start point segment
 * @param dst_point end point
 * @param dst_seg_idx index of end point segment
 * @return length of distance between two points.
 * Length is positive if destination point is greater that source point (i.e. after it in
 * trajectory, path, ...) and negative otherwise.
 */
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx)
{
  validateNonEmpty(points);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_seg_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);
  const double signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj - signed_length_src_offset + signed_length_dst_offset;
}

extern template double calcSignedArcLength<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
extern template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
extern template double
calcSignedArcLength<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);

/**
 * @brief calculate length of 2D distance between two points, specified by start point and its
 * segment index in points container and end point index in points container
 * @param points points of trajectory, path, ...
 * @param src_point start point
 * @param src_seg_idx index of start point segment
 * @param dst_idx index of end point
 * @return length of distance between two points
 * Length is positive if destination point associated to dst_idx is greater that source point (i.e.
 * after it in trajectory, path, ...) and negative otherwise.
 */
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const size_t dst_idx)
{
  validateNonEmpty(points);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return signed_length_on_traj - signed_length_src_offset;
}

extern template double calcSignedArcLength<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);
extern template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);
extern template double
calcSignedArcLength<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);

/**
 * @brief calculate length of 2D distance between two points, specified by start point index in
 * points container and end point and its segment index in points container
 * @param points points of trajectory, path, ...
 * @param src_idx index of start point start point
 * @param dst_point end point
 * @param dst_seg_idx index of end point segment
 * @return length of distance between two points
 * Length is positive if destination point is greater that source point associated to src_idx (i.e.
 * after it in trajectory, path, ...) and negative otherwise.
 */
template <class T>
double calcSignedArcLength(
  const T & points, const size_t src_idx, const geometry_msgs::msg::Point & dst_point,
  const size_t dst_seg_idx)
{
  validateNonEmpty(points);

  const double signed_length_on_traj = calcSignedArcLength(points, src_idx, dst_seg_idx);
  const double signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj + signed_length_dst_offset;
}

extern template double calcSignedArcLength<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
extern template double
calcSignedArcLength<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points, const size_t src_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
extern template double
calcSignedArcLength<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const size_t src_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);

/**
 * @brief find first nearest point index through points container for a given pose with soft
 * distance and yaw constraints. Finding nearest point is determined by looping through the points
 * container, and finding the nearest point to the given pose in terms of squared 2D distance and
 * yaw deviation. The index of the point with minimum distance and yaw deviation comparing to the
 * given pose will be returned.
 * @param points points of trajectory, path, ...
 * @param pose given pose
 * @param dist_threshold distance threshold used for searching for first nearest index to given pose
 * @param yaw_threshold yaw threshold used for searching for first nearest index to given pose
 * @return index of nearest point (index or none if not found)
 */
template <class T>
size_t findFirstNearestIndexWithSoftConstraints(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  validateNonEmpty(points);

  {  // with dist and yaw thresholds
    const double squared_dist_threshold = dist_threshold * dist_threshold;
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto squared_dist =
        autoware::universe_utils::calcSquaredDistance2d(points.at(i), pose.position);
      const auto yaw = autoware::universe_utils::calcYawDeviation(
        autoware::universe_utils::getPose(points.at(i)), pose);

      if (squared_dist_threshold < squared_dist || yaw_threshold < std::abs(yaw)) {
        if (is_within_constraints) {
          break;
        }
        continue;
      }

      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      return min_idx;
    }
  }

  {  // with dist threshold
    const double squared_dist_threshold = dist_threshold * dist_threshold;
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto squared_dist =
        autoware::universe_utils::calcSquaredDistance2d(points.at(i), pose.position);

      if (squared_dist_threshold < squared_dist) {
        if (is_within_constraints) {
          break;
        }
        continue;
      }

      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      return min_idx;
    }
  }

  // without any threshold
  return findNearestIndex(points, pose.position);
}

extern template size_t
findFirstNearestIndexWithSoftConstraints<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());
extern template size_t findFirstNearestIndexWithSoftConstraints<
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());
extern template size_t
findFirstNearestIndexWithSoftConstraints<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());

/**
 * @brief find nearest segment index to pose with soft constraints
 * Segment is straight path between two continuous points of trajectory
 * When pose is on a trajectory point whose index is nearest_idx, return nearest_idx - 1
 * @param points points of trajectory, path, ..
 * @param pose pose to which to find nearest segment index
 * @param dist_threshold distance threshold used for searching for first nearest index to given pose
 * @param yaw_threshold yaw threshold used for searching for first nearest index to given pose
 * @return nearest index
 */
template <class T>
size_t findFirstNearestSegmentIndexWithSoftConstraints(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  // find first nearest index with soft constraints (not segment index)
  const size_t nearest_idx =
    findFirstNearestIndexWithSoftConstraints(points, pose, dist_threshold, yaw_threshold);

  // calculate segment index
  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, pose.position);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

extern template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());
extern template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());
extern template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());

/**
 * @brief calculate the point offset from source point along the trajectory (or path)
 * @brief calculate length of 2D distance between given pose and first point in container with zero
 * longitudinal velocity
 * @param points_with_twist points of trajectory, path, ... (with velocity)
 * @param pose given pose to start the distance calculation from
 * @param max_dist max distance, used to search for nearest segment index in points container to the
 * given pose
 * @param max_yaw max yaw, used to search for nearest segment index in points container to the given
 * pose
 * @return Length of 2D distance between given pose and first point in container with zero
 * longitudinal velocity
 */
template <class T>
std::optional<double> calcDistanceToForwardStopPoint(
  const T & points_with_twist, const geometry_msgs::msg::Pose & pose,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())
{
  try {
    validateNonEmpty(points_with_twist);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "Failed to calculate stop distance %s", e.what());
    return {};
  }

  const auto nearest_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(points_with_twist, pose, max_dist, max_yaw);

  if (!nearest_segment_idx) {
    return std::nullopt;
  }

  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(
    points_with_twist, *nearest_segment_idx + 1, points_with_twist.size());

  if (!stop_idx) {
    return std::nullopt;
  }

  const auto closest_stop_dist =
    calcSignedArcLength(points_with_twist, pose.position, *nearest_segment_idx, *stop_idx);

  return std::max(0.0, closest_stop_dist);
}

extern template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points_with_twist,
  const geometry_msgs::msg::Pose & pose, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max());
extern template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points_with_twist,
  const geometry_msgs::msg::Pose & pose, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max());
extern template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const geometry_msgs::msg::Pose & pose, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max());

// NOTE: Points after forward length from the point will be cropped
//       forward_length is assumed to be positive.
template <typename T>
T cropForwardPoints(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length)
{
  if (points.empty()) {
    return T{};
  }

  double sum_length =
    -autoware::motion_utils::calcLongitudinalOffsetToSegment(points, target_seg_idx, target_pos);
  for (size_t i = target_seg_idx + 1; i < points.size(); ++i) {
    sum_length += autoware::universe_utils::calcDistance2d(points.at(i), points.at(i - 1));
    if (forward_length < sum_length) {
      const size_t end_idx = i;
      return T{points.begin(), points.begin() + end_idx};
    }
  }

  return points;
}

extern template std::vector<autoware_planning_msgs::msg::PathPoint>
cropForwardPoints<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length);
extern template std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
cropForwardPoints<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length);
extern template std::vector<autoware_planning_msgs::msg::TrajectoryPoint>
cropForwardPoints<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length);

// NOTE: Points before backward length from the point will be cropped
//       backward_length is assumed to be positive.
template <typename T>
T cropBackwardPoints(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double backward_length)
{
  if (points.empty()) {
    return T{};
  }

  double sum_length =
    -autoware::motion_utils::calcLongitudinalOffsetToSegment(points, target_seg_idx, target_pos);
  for (int i = target_seg_idx; 0 < i; --i) {
    sum_length -= autoware::universe_utils::calcDistance2d(points.at(i), points.at(i - 1));
    if (sum_length < -backward_length) {
      const size_t begin_idx = i;
      return T{points.begin() + begin_idx, points.end()};
    }
  }

  return points;
}

extern template std::vector<autoware_planning_msgs::msg::PathPoint>
cropBackwardPoints<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double backward_length);
extern template std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
cropBackwardPoints<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double backward_length);
extern template std::vector<autoware_planning_msgs::msg::TrajectoryPoint>
cropBackwardPoints<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double backward_length);

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

  if (cropped_points.size() < 2) {
    RCLCPP_DEBUG(get_logger(), "Return original points since cropped_points size is less than 2.");
    return points;
  }

  return cropped_points;
}

extern template std::vector<autoware_planning_msgs::msg::PathPoint>
cropPoints<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
extern template std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
cropPoints<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
extern template std::vector<autoware_planning_msgs::msg::TrajectoryPoint>
cropPoints<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);

/**
 * @brief Calculate the angle of the input pose with respect to the nearest trajectory segment.
 * The function gets the nearest segment index between the points of the trajectory and the given
 * pose's position, then calculates the azimuth angle of that segment and compares it to the yaw of
 * the input pose. The segment is a straight path between two continuous points of the trajectory.
 * @param points Points of the trajectory, path, ...
 * @param pose Input pose with position and orientation (yaw)
 * @param throw_exception Flag to enable/disable exception throwing
 * @return Angle with respect to the trajectory segment (signed) in radians
 */
template <class T>
double calcYawDeviation(
  const T & points, const geometry_msgs::msg::Pose & pose, const bool throw_exception = false)
{
  const auto overlap_removed_points = removeOverlapPoints(points, 0);

  if (throw_exception) {
    validateNonEmpty(overlap_removed_points);
  } else {
    try {
      validateNonEmpty(overlap_removed_points);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(get_logger(), "%s", e.what());
      return 0.0;
    }
  }

  if (overlap_removed_points.size() <= 1) {
    const std::string error_message(
      "[autoware_motion_utils] " + std::string(__func__) +
      " Given points size is less than 2. Failed to calculate yaw deviation.");
    autoware::universe_utils::print_backtrace();
    if (throw_exception) {
      throw std::runtime_error(error_message);
    }
    RCLCPP_DEBUG(
      get_logger(),
      "%s Return 0 since no_throw option is enabled. The maintainer must check the code.",
      error_message.c_str());
    return 0.0;
  }

  const size_t seg_idx = findNearestSegmentIndex(overlap_removed_points, pose.position);

  const double path_yaw = autoware::universe_utils::calcAzimuthAngle(
    autoware::universe_utils::getPoint(overlap_removed_points.at(seg_idx)),
    autoware::universe_utils::getPoint(overlap_removed_points.at(seg_idx + 1)));
  const double pose_yaw = tf2::getYaw(pose.orientation);

  return autoware::universe_utils::normalizeRadian(pose_yaw - path_yaw);
}

extern template double calcYawDeviation<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose, const bool throw_exception = false);
extern template double calcYawDeviation<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const bool throw_exception = false);
extern template double calcYawDeviation<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const bool throw_exception = false);

/**
 * @brief Check if the given target point is in front of the based pose from the trajectory.
 * if the points is empty, the function returns false
 * @param points Points of the trajectory, path, ...
 * @param base_point Base point
 * @param target_point Target point
 * @param threshold threshold for judging front point
 * @return true if the target pose is in front of the base pose
 */
template <class T>
bool isTargetPointFront(
  const T & points, const geometry_msgs::msg::Point & base_point,
  const geometry_msgs::msg::Point & target_point, const double threshold = 0.0)
{
  if (points.empty()) {
    return false;
  }

  const double s_base = calcSignedArcLength(points, 0, base_point);
  const double s_target = calcSignedArcLength(points, 0, target_point);

  return s_target - s_base > threshold;
}

extern template bool isTargetPointFront<std::vector<autoware_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & base_point, const geometry_msgs::msg::Point & target_point,
  const double threshold = 0.0);
extern template bool isTargetPointFront<std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & base_point, const geometry_msgs::msg::Point & target_point,
  const double threshold = 0.0);
extern template bool isTargetPointFront<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & base_point, const geometry_msgs::msg::Point & target_point,
  const double threshold = 0.0);

/// @brief calculate the time_from_start fields of the given trajectory points
/// @details this function assumes constant longitudinal velocity between points
/// @param trajectory trajectory for which to calculate the time_from_start
/// @param current_ego_point current ego position
/// @param min_velocity minimum velocity used for a trajectory point
void calculate_time_from_start(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & current_ego_point, const float min_velocity = 1.0f);

}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY__TRAJECTORY_HPP_
