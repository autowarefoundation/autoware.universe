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

#ifndef MOTION_UTILS__RESAMPLE__TRAJECTORY_HPP_
#define MOTION_UTILS__RESAMPLE__TRAJECTORY_HPP_

#include "interpolation/interpolation_utils.hpp"
#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "interpolation/zero_order_hold.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/pose_deviation.hpp"
#include "tier4_autoware_utils/math/constants.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

inline boost::optional<autoware_auto_planning_msgs::msg::Path> resamplePath(
  const autoware_auto_planning_msgs::msg::Path & input_path,
  const std::vector<double> & resampled_arclength, const bool use_lerp_for_xy = false,
  const bool use_lerp_for_z = true, const bool use_zero_order_hold_for_v = true)
{
  // Check vector size and if out_arclength have the end point of the path
  const double input_path_len = motion_utils::calcArcLength(input_path.points);
  if (
    input_path.points.size() < 2 || resampled_arclength.size() < 2 ||
    input_path_len < resampled_arclength.back()) {
    return {};
  }

  // Input Path Information
  std::vector<double> input_arclength(input_path.points.size());
  std::vector<double> x(input_path.points.size());
  std::vector<double> y(input_path.points.size());
  std::vector<double> z(input_path.points.size());
  std::vector<double> v_lon(input_path.points.size());
  std::vector<double> v_lat(input_path.points.size());
  std::vector<double> heading_rate(input_path.points.size());
  for (size_t i = 0; i < input_path.points.size(); ++i) {
    const double ds = motion_utils::calcSignedArcLength(input_path.points, 0, i);
    input_arclength.at(i) = ds;
    x.at(i) = input_path.points.at(i).pose.position.x;
    y.at(i) = input_path.points.at(i).pose.position.y;
    z.at(i) = input_path.points.at(i).pose.position.z;
    v_lon.at(i) = input_path.points.at(i).longitudinal_velocity_mps;
    v_lat.at(i) = input_path.points.at(i).lateral_velocity_mps;
    heading_rate.at(i) = input_path.points.at(i).heading_rate_rps;
  }

  // Interpolate
  const auto interpolated_x = use_lerp_for_xy
                                ? interpolation::lerp(input_arclength, x, resampled_arclength)
                                : interpolation::slerp(input_arclength, x, resampled_arclength);
  const auto interpolated_y = use_lerp_for_xy
                                ? interpolation::lerp(input_arclength, y, resampled_arclength)
                                : interpolation::slerp(input_arclength, y, resampled_arclength);
  const auto interpolated_z = use_lerp_for_z
                                ? interpolation::lerp(input_arclength, z, resampled_arclength)
                                : interpolation::slerp(input_arclength, z, resampled_arclength);
  const auto interpolated_v_lon =
    use_zero_order_hold_for_v
      ? interpolation::zero_order_hold(input_arclength, v_lon, resampled_arclength)
      : interpolation::lerp(input_arclength, v_lon, resampled_arclength);
  const auto interpolated_v_lat =
    use_zero_order_hold_for_v
      ? interpolation::zero_order_hold(input_arclength, v_lat, resampled_arclength)
      : interpolation::lerp(input_arclength, v_lat, resampled_arclength);
  const auto interpolated_heading_rate =
    interpolation::lerp(input_arclength, heading_rate, resampled_arclength);

  autoware_auto_planning_msgs::msg::Path interpolated_path;
  interpolated_path.header = input_path.header;
  interpolated_path.drivable_area = input_path.drivable_area;
  interpolated_path.points.resize(interpolated_x.size());

  // Insert Position, Velocity and Heading Rate
  for (size_t i = 0; i < interpolated_path.points.size(); ++i) {
    autoware_auto_planning_msgs::msg::PathPoint path_point;
    path_point.pose.position.x = interpolated_x.at(i);
    path_point.pose.position.y = interpolated_y.at(i);
    path_point.pose.position.z = interpolated_z.at(i);
    path_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    path_point.lateral_velocity_mps = interpolated_v_lat.at(i);
    path_point.heading_rate_rps = interpolated_heading_rate.at(i);
    interpolated_path.points.at(i) = path_point;
  }

  // Insert Orientation
  for (size_t i = 0; i < interpolated_path.points.size() - 1; ++i) {
    const auto & src_point = interpolated_path.points.at(i).pose.position;
    const auto & dst_point = interpolated_path.points.at(i + 1).pose.position;
    const double pitch = tier4_autoware_utils::calcElevationAngle(src_point, dst_point);
    const double yaw = tier4_autoware_utils::calcAzimuthAngle(src_point, dst_point);
    interpolated_path.points.at(i).pose.orientation =
      tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw);
  }

  return interpolated_path;
}

#endif  // MOTION_UTILS__RESAMPLE__TRAJECTORY_HPP_
