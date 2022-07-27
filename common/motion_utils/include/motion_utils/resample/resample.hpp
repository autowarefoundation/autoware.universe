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

#ifndef MOTION_UTILS__RESAMPLE__RESAMPLE_HPP_
#define MOTION_UTILS__RESAMPLE__RESAMPLE_HPP_

#include "interpolation/interpolation_utils.hpp"
#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "interpolation/zero_order_hold.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/pose_deviation.hpp"
#include "tier4_autoware_utils/math/constants.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace motion_utils
{
/**
 * @brief A resampling function for a path(poses). Note that in a default setting, position xy are
 * resampled by spline interpolation, position z are resampled by linear interpolation, and
 * orientation of the resampled path are calculated by a forward difference method
 * based on the interpolated position x and y.
 * @param input_path input path(poses) to resample
 * @param resampled_arclength arclength that contains length of each resampling points from initial
 * point
 * @param use_lerp_for_xy If this is true, it uses linear interpolation to resample position x and
 * y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If this is true, it uses linear interpolation to resample position z.
 * Otherwise, it uses spline interpolation
 * @return resampled path(poses)
 */
inline std::vector<geometry_msgs::msg::Pose> resamplePath(
  const std::vector<geometry_msgs::msg::Pose> & points,
  const std::vector<double> & resampled_arclength, const bool use_lerp_for_xy = false,
  const bool use_lerp_for_z = true)
{
  // Check vector size and if out_arclength have the end point of the path
  const double input_path_len = motion_utils::calcArcLength(points);
  if (
    points.size() < 2 || resampled_arclength.size() < 2 ||
    input_path_len < resampled_arclength.back()) {
    std::cerr
      << "[motion_utils]: input points size, input points length or resampled arclength is wrong"
      << std::endl;
    return points;
  }

  // Input Path Information
  std::vector<double> input_arclength(points.size());
  std::vector<double> x(points.size());
  std::vector<double> y(points.size());
  std::vector<double> z(points.size());

  input_arclength.front() = 0.0;
  x.front() = points.front().position.x;
  y.front() = points.front().position.y;
  z.front() = points.front().position.z;
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & prev_pt = points.at(i - 1);
    const auto & curr_pt = points.at(i);
    const double ds = tier4_autoware_utils::calcDistance2d(prev_pt.position, curr_pt.position);
    input_arclength.at(i) = ds + input_arclength.at(i - 1);
    x.at(i) = curr_pt.position.x;
    y.at(i) = curr_pt.position.y;
    z.at(i) = curr_pt.position.z;
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_arclength, input, resampled_arclength);
  };
  const auto slerp = [&](const auto & input) {
    return interpolation::slerp(input_arclength, input, resampled_arclength);
  };

  const auto interpolated_x = use_lerp_for_xy ? lerp(x) : slerp(x);
  const auto interpolated_y = use_lerp_for_xy ? lerp(y) : slerp(y);
  const auto interpolated_z = use_lerp_for_z ? lerp(z) : slerp(z);

  std::vector<geometry_msgs::msg::Pose> interpolated_points;
  interpolated_points.resize(interpolated_x.size());

  // Insert Position, Velocity and Heading Rate
  for (size_t i = 0; i < interpolated_points.size(); ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = interpolated_x.at(i);
    pose.position.y = interpolated_y.at(i);
    pose.position.z = interpolated_z.at(i);
    interpolated_points.at(i) = pose;
  }

  // Insert Orientation
  for (size_t i = 0; i < interpolated_points.size() - 1; ++i) {
    const auto & src_point = interpolated_points.at(i).position;
    const auto & dst_point = interpolated_points.at(i + 1).position;
    const double pitch = tier4_autoware_utils::calcElevationAngle(src_point, dst_point);
    const double yaw = tier4_autoware_utils::calcAzimuthAngle(src_point, dst_point);
    interpolated_points.at(i).orientation =
      tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw);
    if (i == interpolated_points.size() - 2) {
      // Terminal Orientation is same as the point before it
      interpolated_points.at(i + 1).orientation = interpolated_points.at(i).orientation;
    }
  }

  return interpolated_points;
}

/**
 * @brief A resampling function for a path. Note that in a default setting, position xy are
 * resampled by spline interpolation, position z are resampled by linear interpolation, longitudinal
 * and lateral velocity are resampled by zero_order_hold, and heading rate is resampled by linear
 * interpolation. Orientation of the resampled path are calculated by a forward difference method
 * based on the interpolated position x and y.
 * @param input_path input path to resample
 * @param resampled_arclength arclength that contains length of each resampling points from initial
 * point
 * @param use_lerp_for_xy If this is true, it uses linear interpolation to resample position x and
 * y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If this is true, it uses linear interpolation to resample position z.
 * Otherwise, it uses spline interpolation
 * @param use_zero_order_hold_for_v If this is true, it uses zero_order_hold to resample
 * longitudinal and lateral velocity. Otherwise, it uses linear interpolation
 * @return resampled path
 */
inline autoware_auto_planning_msgs::msg::Path resamplePath(
  const autoware_auto_planning_msgs::msg::Path & input_path,
  const std::vector<double> & resampled_arclength, const bool use_lerp_for_xy = false,
  const bool use_lerp_for_z = true, const bool use_zero_order_hold_for_v = true)
{
  // Check vector size and if out_arclength have the end point of the path
  const double input_path_len = motion_utils::calcArcLength(input_path.points);
  if (
    input_path.points.size() < 2 || resampled_arclength.size() < 2 ||
    input_path_len < resampled_arclength.back()) {
    std::cerr
      << "[motion_utils]: input path size, input path length or resampled arclength is wrong"
      << std::endl;
    return input_path;
  }

  // Input Path Information
  std::vector<double> input_arclength(input_path.points.size());
  std::vector<geometry_msgs::msg::Pose> input_pose(input_path.points.size());
  std::vector<double> v_lon(input_path.points.size());
  std::vector<double> v_lat(input_path.points.size());
  std::vector<double> heading_rate(input_path.points.size());

  input_arclength.front() = 0.0;
  input_pose.front() = input_path.points.front().pose;
  v_lon.front() = input_path.points.front().longitudinal_velocity_mps;
  v_lat.front() = input_path.points.front().lateral_velocity_mps;
  heading_rate.front() = input_path.points.front().heading_rate_rps;
  for (size_t i = 1; i < input_path.points.size(); ++i) {
    const auto & prev_pt = input_path.points.at(i - 1);
    const auto & curr_pt = input_path.points.at(i);
    const double ds =
      tier4_autoware_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);
    input_arclength.at(i) = ds + input_arclength.at(i - 1);
    input_pose.at(i) = curr_pt.pose;
    v_lon.at(i) = curr_pt.longitudinal_velocity_mps;
    v_lat.at(i) = curr_pt.lateral_velocity_mps;
    heading_rate.at(i) = curr_pt.heading_rate_rps;
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_arclength, input, resampled_arclength);
  };
  const auto zoh = [&](const auto & input) {
    return interpolation::zero_order_hold(input_arclength, input, resampled_arclength);
  };

  const auto interpolated_pose =
    resamplePath(input_pose, resampled_arclength, use_lerp_for_xy, use_lerp_for_z);
  const auto interpolated_v_lon = use_zero_order_hold_for_v ? zoh(v_lon) : lerp(v_lon);
  const auto interpolated_v_lat = use_zero_order_hold_for_v ? zoh(v_lat) : lerp(v_lat);
  const auto interpolated_heading_rate = lerp(heading_rate);

  if (interpolated_pose.size() != resampled_arclength.size()) {
    std::cerr << "[motion_utils]: Resampled pose size is different from resampled arclength"
              << std::endl;
    return input_path;
  }

  autoware_auto_planning_msgs::msg::Path resampled_path;
  resampled_path.header = input_path.header;
  resampled_path.drivable_area = input_path.drivable_area;
  resampled_path.points.resize(interpolated_pose.size());

  // Insert Position, Velocity and Heading Rate
  for (size_t i = 0; i < resampled_path.points.size(); ++i) {
    autoware_auto_planning_msgs::msg::PathPoint path_point;
    path_point.pose = interpolated_pose.at(i);
    path_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    path_point.lateral_velocity_mps = interpolated_v_lat.at(i);
    path_point.heading_rate_rps = interpolated_heading_rate.at(i);
    resampled_path.points.at(i) = path_point;
  }

  return resampled_path;
}

inline autoware_auto_planning_msgs::msg::Trajectory resampleTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & input_trajectory,
  const std::vector<double> & resampled_arclength, const bool use_lerp_for_xy = false,
  const bool use_lerp_for_z = true, const bool use_zero_order_hold_for_twist = true)
{
  // Check vector size and if out_arclength have the end point of the trajectory
  const double input_trajectory_len = motion_utils::calcArcLength(input_trajectory.points);
  if (
    input_trajectory.points.size() < 2 || resampled_arclength.size() < 2 ||
    input_trajectory_len < resampled_arclength.back()) {
    std::cerr << "[motion_utils]: input trajectory size, input trajectory length or resampled "
                 "arclength is wrong"
              << std::endl;
    return input_trajectory;
  }

  // Input Path Information
  std::vector<double> input_arclength(input_trajectory.points.size());
  std::vector<geometry_msgs::msg::Pose> input_pose(input_trajectory.points.size());
  std::vector<double> v_lon(input_trajectory.points.size());
  std::vector<double> v_lat(input_trajectory.points.size());
  std::vector<double> heading_rate(input_trajectory.points.size());
  std::vector<double> acceleration(input_trajectory.points.size());
  std::vector<double> front_wheel_angle(input_trajectory.points.size());
  std::vector<double> rear_wheel_angle(input_trajectory.points.size());
  std::vector<double> time_from_start(input_trajectory.points.size());

  input_arclength.front() = 0.0;
  input_pose.front() = input_trajectory.points.front().pose;
  v_lon.front() = input_trajectory.points.front().longitudinal_velocity_mps;
  v_lat.front() = input_trajectory.points.front().lateral_velocity_mps;
  heading_rate.front() = input_trajectory.points.front().heading_rate_rps;
  acceleration.front() = input_trajectory.points.front().acceleration_mps2;
  front_wheel_angle.front() = input_trajectory.points.front().front_wheel_angle_rad;
  rear_wheel_angle.front() = input_trajectory.points.front().rear_wheel_angle_rad;
  time_from_start.front() =
    rclcpp::Duration(input_trajectory.points.front().time_from_start).seconds();
  for (size_t i = 1; i < input_trajectory.points.size(); ++i) {
    const auto & prev_pt = input_trajectory.points.at(i - 1);
    const auto & curr_pt = input_trajectory.points.at(i);
    const double ds =
      tier4_autoware_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);
    input_arclength.at(i) = ds + input_arclength.at(i - 1);
    input_pose.at(i) = curr_pt.pose;
    v_lon.at(i) = curr_pt.longitudinal_velocity_mps;
    v_lat.at(i) = curr_pt.lateral_velocity_mps;
    heading_rate.at(i) = curr_pt.heading_rate_rps;
    acceleration.at(i) = curr_pt.acceleration_mps2;
    front_wheel_angle.at(i) = curr_pt.front_wheel_angle_rad;
    rear_wheel_angle.at(i) = curr_pt.rear_wheel_angle_rad;
    time_from_start.at(i) = rclcpp::Duration(curr_pt.time_from_start).seconds();
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_arclength, input, resampled_arclength);
  };
  const auto zoh = [&](const auto & input) {
    return interpolation::zero_order_hold(input_arclength, input, resampled_arclength);
  };

  const auto interpolated_pose =
    resamplePath(input_pose, resampled_arclength, use_lerp_for_xy, use_lerp_for_z);
  const auto interpolated_v_lon = use_zero_order_hold_for_twist ? zoh(v_lon) : lerp(v_lon);
  const auto interpolated_v_lat = use_zero_order_hold_for_twist ? zoh(v_lat) : lerp(v_lat);
  const auto interpolated_heading_rate = lerp(heading_rate);
  const auto interpolated_acceleration =
    use_zero_order_hold_for_twist ? zoh(acceleration) : lerp(acceleration);
  const auto interpolated_front_wheel_angle = lerp(front_wheel_angle);
  const auto interpolated_rear_wheel_angle = lerp(rear_wheel_angle);
  const auto interpolated_time_from_start = lerp(time_from_start);

  if (interpolated_pose.size() != resampled_arclength.size()) {
    std::cerr << "[motion_utils]: Resampled pose size is different from resampled arclength"
              << std::endl;
    return input_trajectory;
  }

  autoware_auto_planning_msgs::msg::Trajectory resampled_trajectory;
  resampled_trajectory.header = input_trajectory.header;
  resampled_trajectory.points.resize(interpolated_pose.size());

  // Insert Position, Velocity and Heading Rate
  for (size_t i = 0; i < resampled_trajectory.points.size(); ++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose = interpolated_pose.at(i);
    traj_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    traj_point.lateral_velocity_mps = interpolated_v_lat.at(i);
    traj_point.heading_rate_rps = interpolated_heading_rate.at(i);
    traj_point.acceleration_mps2 = interpolated_acceleration.at(i);
    traj_point.front_wheel_angle_rad = interpolated_front_wheel_angle.at(i);
    traj_point.rear_wheel_angle_rad = interpolated_rear_wheel_angle.at(i);
    traj_point.time_from_start = rclcpp::Duration::from_seconds(interpolated_time_from_start.at(i));
    resampled_trajectory.points.at(i) = traj_point;
  }

  return resampled_trajectory;
}
}  // namespace motion_utils

#endif  // MOTION_UTILS__RESAMPLE__RESAMPLE_HPP_
