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

#include "autoware/motion_utils/resample/resample.hpp"

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/interpolation/zero_order_hold.hpp"
#include "autoware/motion_utils/resample/resample_utils.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace autoware::motion_utils
{
std::vector<geometry_msgs::msg::Point> resamplePointVector(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy,
  const bool use_lerp_for_z)
{
  // validate arguments
  if (!resample_utils::validate_arguments(points, resampled_arclength)) {
    return points;
  }

  // Input Path Information
  std::vector<double> input_arclength;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  input_arclength.reserve(points.size());
  x.reserve(points.size());
  y.reserve(points.size());
  z.reserve(points.size());

  input_arclength.push_back(0.0);
  x.push_back(points.front().x);
  y.push_back(points.front().y);
  z.push_back(points.front().z);
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & prev_pt = points.at(i - 1);
    const auto & curr_pt = points.at(i);
    const double ds = autoware::universe_utils::calcDistance2d(prev_pt, curr_pt);
    input_arclength.push_back(ds + input_arclength.back());
    x.push_back(curr_pt.x);
    y.push_back(curr_pt.y);
    z.push_back(curr_pt.z);
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return autoware::interpolation::lerp(input_arclength, input, resampled_arclength);
  };
  const auto spline = [&](const auto & input) {
    return autoware::interpolation::spline(input_arclength, input, resampled_arclength);
  };
  const auto spline_by_akima = [&](const auto & input) {
    return autoware::interpolation::splineByAkima(input_arclength, input, resampled_arclength);
  };

  const auto interpolated_x = use_akima_spline_for_xy ? lerp(x) : spline_by_akima(x);
  const auto interpolated_y = use_akima_spline_for_xy ? lerp(y) : spline_by_akima(y);
  const auto interpolated_z = use_lerp_for_z ? lerp(z) : spline(z);

  std::vector<geometry_msgs::msg::Point> resampled_points;
  resampled_points.resize(interpolated_x.size());

  // Insert Position
  for (size_t i = 0; i < resampled_points.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = interpolated_x.at(i);
    point.y = interpolated_y.at(i);
    point.z = interpolated_z.at(i);
    resampled_points.at(i) = point;
  }

  return resampled_points;
}

std::vector<geometry_msgs::msg::Point> resamplePointVector(
  const std::vector<geometry_msgs::msg::Point> & points, const double resample_interval,
  const bool use_akima_spline_for_xy, const bool use_lerp_for_z)
{
  const double input_length = autoware::motion_utils::calcArcLength(points);

  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < input_length; s += resample_interval) {
    resampling_arclength.push_back(s);
  }
  if (resampling_arclength.empty()) {
    std::cerr << "[autoware_motion_utils]: resampling arclength is empty" << std::endl;
    return points;
  }

  // Insert terminal point
  if (input_length - resampling_arclength.back() < autoware::motion_utils::overlap_threshold) {
    resampling_arclength.back() = input_length;
  } else {
    resampling_arclength.push_back(input_length);
  }

  return resamplePointVector(points, resampling_arclength, use_akima_spline_for_xy, use_lerp_for_z);
}

std::vector<geometry_msgs::msg::Pose> resamplePoseVector(
  const std::vector<geometry_msgs::msg::Pose> & points_raw,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy,
  const bool use_lerp_for_z)
{
  // Remove overlap points for resampling
  const auto points = autoware::motion_utils::removeOverlapPoints(points_raw);

  // validate arguments
  if (!resample_utils::validate_arguments(points, resampled_arclength)) {
    return points_raw;
  }

  std::vector<geometry_msgs::msg::Point> position(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    position.at(i) = points.at(i).position;
  }
  const auto resampled_position =
    resamplePointVector(position, resampled_arclength, use_akima_spline_for_xy, use_lerp_for_z);

  std::vector<geometry_msgs::msg::Pose> resampled_points(resampled_position.size());

  // Insert Position
  for (size_t i = 0; i < resampled_position.size(); ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = resampled_position.at(i).x;
    pose.position.y = resampled_position.at(i).y;
    pose.position.z = resampled_position.at(i).z;
    resampled_points.at(i) = pose;
  }

  const bool is_driving_forward =
    autoware::universe_utils::isDrivingForward(points.at(0), points.at(1));
  autoware::motion_utils::insertOrientation(resampled_points, is_driving_forward);

  // Initial orientation is depend on the initial value of the resampled_arclength
  // when backward driving
  if (!is_driving_forward && resampled_arclength.front() < 1e-3) {
    resampled_points.at(0).orientation = points.at(0).orientation;
  }

  return resampled_points;
}

std::vector<geometry_msgs::msg::Pose> resamplePoseVector(
  const std::vector<geometry_msgs::msg::Pose> & points, const double resample_interval,
  const bool use_akima_spline_for_xy, const bool use_lerp_for_z)
{
  const double input_length = autoware::motion_utils::calcArcLength(points);

  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < input_length; s += resample_interval) {
    resampling_arclength.push_back(s);
  }
  if (resampling_arclength.empty()) {
    std::cerr << "[autoware_motion_utils]: resampling arclength is empty" << std::endl;
    return points;
  }

  // Insert terminal point
  if (input_length - resampling_arclength.back() < autoware::motion_utils::overlap_threshold) {
    resampling_arclength.back() = input_length;
  } else {
    resampling_arclength.push_back(input_length);
  }

  return resamplePoseVector(points, resampling_arclength, use_akima_spline_for_xy, use_lerp_for_z);
}

tier4_planning_msgs::msg::PathWithLaneId resamplePath(
  const tier4_planning_msgs::msg::PathWithLaneId & input_path,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy,
  const bool use_lerp_for_z, const bool use_zero_order_hold_for_v)
{
  auto resampling_arclength = resampled_arclength;

  // Add resampling_arclength to insert input points which have multiple lane_ids
  for (size_t i = 0; i < input_path.points.size(); ++i) {
    if (input_path.points.at(i).lane_ids.size() < 2) {
      continue;
    }

    const double distance_to_resampling_point = calcSignedArcLength(input_path.points, 0, i);
    for (size_t j = 1; j < resampling_arclength.size(); ++j) {
      if (
        resampling_arclength.at(j - 1) <= distance_to_resampling_point &&
        distance_to_resampling_point < resampling_arclength.at(j)) {
        const double dist_to_prev_point =
          std::fabs(distance_to_resampling_point - resampling_arclength.at(j - 1));
        const double dist_to_following_point =
          std::fabs(resampling_arclength.at(j) - distance_to_resampling_point);
        if (dist_to_prev_point < autoware::motion_utils::overlap_threshold) {
          resampling_arclength.at(j - 1) = distance_to_resampling_point;
        } else if (dist_to_following_point < autoware::motion_utils::overlap_threshold) {
          resampling_arclength.at(j) = distance_to_resampling_point;
        } else {
          resampling_arclength.insert(
            resampling_arclength.begin() + j, distance_to_resampling_point);
        }
        break;
      }
    }
  }

  // validate arguments
  if (!resample_utils::validate_arguments(input_path.points, resampling_arclength)) {
    return input_path;
  }

  // For LaneIds, is_final
  //
  // ------|----|----|----|----|----|----|-------> resampled
  //      [0]  [1]  [2]  [3]  [4]  [5]  [6]
  //
  // ------|----------------|----------|---------> base
  //      [0]             [1]        [2]
  //
  // resampled[0~3] = base[0]
  // resampled[4~5] = base[1]
  // resampled[6] = base[2]

  // Input Path Information
  std::vector<double> input_arclength;
  std::vector<geometry_msgs::msg::Pose> input_pose;
  std::vector<double> v_lon;
  std::vector<double> v_lat;
  std::vector<double> heading_rate;
  std::vector<bool> is_final;
  std::vector<std::vector<int64_t>> lane_ids;
  input_arclength.reserve(input_path.points.size());
  input_pose.reserve(input_path.points.size());
  v_lon.reserve(input_path.points.size());
  v_lat.reserve(input_path.points.size());
  heading_rate.reserve(input_path.points.size());
  is_final.reserve(input_path.points.size());
  lane_ids.reserve(input_path.points.size());

  input_arclength.push_back(0.0);
  input_pose.push_back(input_path.points.front().point.pose);
  v_lon.push_back(input_path.points.front().point.longitudinal_velocity_mps);
  v_lat.push_back(input_path.points.front().point.lateral_velocity_mps);
  heading_rate.push_back(input_path.points.front().point.heading_rate_rps);
  is_final.push_back(input_path.points.front().point.is_final);
  lane_ids.push_back(input_path.points.front().lane_ids);
  for (size_t i = 1; i < input_path.points.size(); ++i) {
    const auto & prev_pt = input_path.points.at(i - 1).point;
    const auto & curr_pt = input_path.points.at(i).point;
    const double ds =
      autoware::universe_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);
    input_arclength.push_back(ds + input_arclength.back());
    input_pose.push_back(curr_pt.pose);
    v_lon.push_back(curr_pt.longitudinal_velocity_mps);
    v_lat.push_back(curr_pt.lateral_velocity_mps);
    heading_rate.push_back(curr_pt.heading_rate_rps);
    is_final.push_back(curr_pt.is_final);
    lane_ids.push_back(input_path.points.at(i).lane_ids);
  }

  if (input_arclength.back() < resampling_arclength.back()) {
    std::cerr << "[autoware_motion_utils]: resampled path length is longer than input path length"
              << std::endl;
    return input_path;
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return autoware::interpolation::lerp(input_arclength, input, resampling_arclength);
  };

  auto closest_segment_indices =
    autoware::interpolation::calc_closest_segment_indices(input_arclength, resampling_arclength);

  const auto zoh = [&](const auto & input) {
    return autoware::interpolation::zero_order_hold(
      input_arclength, input, closest_segment_indices);
  };

  const auto interpolated_pose =
    resamplePoseVector(input_pose, resampling_arclength, use_akima_spline_for_xy, use_lerp_for_z);
  const auto interpolated_v_lon = use_zero_order_hold_for_v ? zoh(v_lon) : lerp(v_lon);
  const auto interpolated_v_lat = use_zero_order_hold_for_v ? zoh(v_lat) : lerp(v_lat);
  const auto interpolated_heading_rate = lerp(heading_rate);
  const auto interpolated_is_final = zoh(is_final);

  // interpolate lane_ids
  std::vector<std::vector<int64_t>> interpolated_lane_ids;
  interpolated_lane_ids.resize(resampling_arclength.size());
  constexpr double epsilon = 1e-6;
  for (size_t i = 0; i < resampling_arclength.size(); ++i) {
    const size_t seg_idx = std::min(closest_segment_indices.at(i), input_path.points.size() - 2);
    const auto & prev_lane_ids = input_path.points.at(seg_idx).lane_ids;
    const auto & next_lane_ids = input_path.points.at(seg_idx + 1).lane_ids;

    if (std::abs(input_arclength.at(seg_idx) - resampling_arclength.at(i)) <= epsilon) {
      interpolated_lane_ids.at(i).insert(
        interpolated_lane_ids.at(i).end(), prev_lane_ids.begin(), prev_lane_ids.end());
    } else if (std::abs(input_arclength.at(seg_idx + 1) - resampling_arclength.at(i)) <= epsilon) {
      interpolated_lane_ids.at(i).insert(
        interpolated_lane_ids.at(i).end(), next_lane_ids.begin(), next_lane_ids.end());
    } else {
      // extract lane_ids those prev_lane_ids and next_lane_ids have in common
      for (const auto target_lane_id : prev_lane_ids) {
        if (
          std::find(next_lane_ids.begin(), next_lane_ids.end(), target_lane_id) !=
          next_lane_ids.end()) {
          interpolated_lane_ids.at(i).push_back(target_lane_id);
        }
      }
      // If there are no common lane_ids, the prev_lane_ids is assigned.
      if (interpolated_lane_ids.at(i).empty()) {
        interpolated_lane_ids.at(i).insert(
          interpolated_lane_ids.at(i).end(), prev_lane_ids.begin(), prev_lane_ids.end());
      }
    }
  }

  if (interpolated_pose.size() != resampling_arclength.size()) {
    std::cerr
      << "[autoware_motion_utils]: Resampled pose size is different from resampled arclength"
      << std::endl;
    return input_path;
  }

  tier4_planning_msgs::msg::PathWithLaneId resampled_path;
  resampled_path.header = input_path.header;
  resampled_path.left_bound = input_path.left_bound;
  resampled_path.right_bound = input_path.right_bound;
  resampled_path.points.resize(interpolated_pose.size());
  for (size_t i = 0; i < resampled_path.points.size(); ++i) {
    autoware_planning_msgs::msg::PathPoint path_point;
    path_point.pose = interpolated_pose.at(i);
    path_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    path_point.lateral_velocity_mps = interpolated_v_lat.at(i);
    path_point.heading_rate_rps = interpolated_heading_rate.at(i);
    path_point.is_final = interpolated_is_final.at(i);
    resampled_path.points.at(i).point = path_point;
    resampled_path.points.at(i).lane_ids = interpolated_lane_ids.at(i);
  }

  return resampled_path;
}

tier4_planning_msgs::msg::PathWithLaneId resamplePath(
  const tier4_planning_msgs::msg::PathWithLaneId & input_path, const double resample_interval,
  const bool use_akima_spline_for_xy, const bool use_lerp_for_z,
  const bool use_zero_order_hold_for_v, const bool resample_input_path_stop_point)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_path.points, resample_interval)) {
    return input_path;
  }

  // transform input_path
  std::vector<autoware_planning_msgs::msg::PathPoint> transformed_input_path(
    input_path.points.size());
  for (size_t i = 0; i < input_path.points.size(); ++i) {
    transformed_input_path.at(i) = input_path.points.at(i).point;
  }
  // compute path length
  const double input_path_len = autoware::motion_utils::calcArcLength(transformed_input_path);

  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < input_path_len; s += resample_interval) {
    resampling_arclength.push_back(s);
  }
  if (resampling_arclength.empty()) {
    std::cerr << "[autoware_motion_utils]: resampling arclength is empty" << std::endl;
    return input_path;
  }

  // Insert terminal point
  if (input_path_len - resampling_arclength.back() < autoware::motion_utils::overlap_threshold) {
    resampling_arclength.back() = input_path_len;
  } else {
    resampling_arclength.push_back(input_path_len);
  }

  // Insert stop point
  if (resample_input_path_stop_point) {
    const auto distance_to_stop_point =
      autoware::motion_utils::calcDistanceToForwardStopPoint(transformed_input_path, 0);
    if (distance_to_stop_point && !resampling_arclength.empty()) {
      for (size_t i = 1; i < resampling_arclength.size(); ++i) {
        if (
          resampling_arclength.at(i - 1) <= *distance_to_stop_point &&
          *distance_to_stop_point < resampling_arclength.at(i)) {
          const double dist_to_prev_point =
            std::fabs(*distance_to_stop_point - resampling_arclength.at(i - 1));
          const double dist_to_following_point =
            std::fabs(resampling_arclength.at(i) - *distance_to_stop_point);
          if (dist_to_prev_point < autoware::motion_utils::overlap_threshold) {
            resampling_arclength.at(i - 1) = *distance_to_stop_point;
          } else if (dist_to_following_point < autoware::motion_utils::overlap_threshold) {
            resampling_arclength.at(i) = *distance_to_stop_point;
          } else {
            resampling_arclength.insert(resampling_arclength.begin() + i, *distance_to_stop_point);
          }
          break;
        }
      }
    }
  }

  return resamplePath(
    input_path, resampling_arclength, use_akima_spline_for_xy, use_lerp_for_z,
    use_zero_order_hold_for_v);
}

autoware_planning_msgs::msg::Path resamplePath(
  const autoware_planning_msgs::msg::Path & input_path,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy,
  const bool use_lerp_for_z, const bool use_zero_order_hold_for_v)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_path.points, resampled_arclength)) {
    return input_path;
  }

  // Input Path Information
  std::vector<double> input_arclength;
  std::vector<geometry_msgs::msg::Pose> input_pose;
  std::vector<double> v_lon;
  std::vector<double> v_lat;
  std::vector<double> heading_rate;
  input_arclength.reserve(input_path.points.size());
  input_pose.reserve(input_path.points.size());
  v_lon.reserve(input_path.points.size());
  v_lat.reserve(input_path.points.size());
  heading_rate.reserve(input_path.points.size());

  input_arclength.push_back(0.0);
  input_pose.push_back(input_path.points.front().pose);
  v_lon.push_back(input_path.points.front().longitudinal_velocity_mps);
  v_lat.push_back(input_path.points.front().lateral_velocity_mps);
  heading_rate.push_back(input_path.points.front().heading_rate_rps);
  for (size_t i = 1; i < input_path.points.size(); ++i) {
    const auto & prev_pt = input_path.points.at(i - 1);
    const auto & curr_pt = input_path.points.at(i);
    const double ds =
      autoware::universe_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);
    input_arclength.push_back(ds + input_arclength.back());
    input_pose.push_back(curr_pt.pose);
    v_lon.push_back(curr_pt.longitudinal_velocity_mps);
    v_lat.push_back(curr_pt.lateral_velocity_mps);
    heading_rate.push_back(curr_pt.heading_rate_rps);
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return autoware::interpolation::lerp(input_arclength, input, resampled_arclength);
  };

  std::vector<size_t> closest_segment_indices;
  if (use_zero_order_hold_for_v) {
    closest_segment_indices =
      autoware::interpolation::calc_closest_segment_indices(input_arclength, resampled_arclength);
  }
  const auto zoh = [&](const auto & input) {
    return autoware::interpolation::zero_order_hold(
      input_arclength, input, closest_segment_indices);
  };

  const auto interpolated_pose =
    resamplePoseVector(input_pose, resampled_arclength, use_akima_spline_for_xy, use_lerp_for_z);
  const auto interpolated_v_lon = use_zero_order_hold_for_v ? zoh(v_lon) : lerp(v_lon);
  const auto interpolated_v_lat = use_zero_order_hold_for_v ? zoh(v_lat) : lerp(v_lat);
  const auto interpolated_heading_rate = lerp(heading_rate);

  if (interpolated_pose.size() != resampled_arclength.size()) {
    std::cerr
      << "[autoware_motion_utils]: Resampled pose size is different from resampled arclength"
      << std::endl;
    return input_path;
  }

  autoware_planning_msgs::msg::Path resampled_path;
  resampled_path.header = input_path.header;
  resampled_path.left_bound = input_path.left_bound;
  resampled_path.right_bound = input_path.right_bound;
  resampled_path.points.resize(interpolated_pose.size());
  for (size_t i = 0; i < resampled_path.points.size(); ++i) {
    autoware_planning_msgs::msg::PathPoint path_point;
    path_point.pose = interpolated_pose.at(i);
    path_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    path_point.lateral_velocity_mps = interpolated_v_lat.at(i);
    path_point.heading_rate_rps = interpolated_heading_rate.at(i);
    resampled_path.points.at(i) = path_point;
  }

  return resampled_path;
}

autoware_planning_msgs::msg::Path resamplePath(
  const autoware_planning_msgs::msg::Path & input_path, const double resample_interval,
  const bool use_akima_spline_for_xy, const bool use_lerp_for_z,
  const bool use_zero_order_hold_for_twist, const bool resample_input_path_stop_point)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_path.points, resample_interval)) {
    return input_path;
  }

  const double input_path_len = autoware::motion_utils::calcArcLength(input_path.points);

  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < input_path_len; s += resample_interval) {
    resampling_arclength.push_back(s);
  }
  if (resampling_arclength.empty()) {
    std::cerr << "[autoware_motion_utils]: resampling arclength is empty" << std::endl;
    return input_path;
  }

  // Insert terminal point
  if (input_path_len - resampling_arclength.back() < autoware::motion_utils::overlap_threshold) {
    resampling_arclength.back() = input_path_len;
  } else {
    resampling_arclength.push_back(input_path_len);
  }

  // Insert stop point
  if (resample_input_path_stop_point) {
    const auto distance_to_stop_point =
      autoware::motion_utils::calcDistanceToForwardStopPoint(input_path.points, 0);
    if (distance_to_stop_point && !resampling_arclength.empty()) {
      for (size_t i = 1; i < resampling_arclength.size(); ++i) {
        if (
          resampling_arclength.at(i - 1) <= *distance_to_stop_point &&
          *distance_to_stop_point < resampling_arclength.at(i)) {
          const double dist_to_prev_point =
            std::fabs(*distance_to_stop_point - resampling_arclength.at(i - 1));
          const double dist_to_following_point =
            std::fabs(resampling_arclength.at(i) - *distance_to_stop_point);
          if (dist_to_prev_point < autoware::motion_utils::overlap_threshold) {
            resampling_arclength.at(i - 1) = *distance_to_stop_point;
          } else if (dist_to_following_point < autoware::motion_utils::overlap_threshold) {
            resampling_arclength.at(i) = *distance_to_stop_point;
          } else {
            resampling_arclength.insert(resampling_arclength.begin() + i, *distance_to_stop_point);
          }
          break;
        }
      }
    }
  }

  return resamplePath(
    input_path, resampling_arclength, use_akima_spline_for_xy, use_lerp_for_z,
    use_zero_order_hold_for_twist);
}

autoware_planning_msgs::msg::Trajectory resampleTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy,
  const bool use_lerp_for_z, const bool use_zero_order_hold_for_twist)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_trajectory.points, resampled_arclength)) {
    return input_trajectory;
  }

  // Input Trajectory Information
  std::vector<double> input_arclength;
  std::vector<geometry_msgs::msg::Pose> input_pose;
  std::vector<double> v_lon;
  std::vector<double> v_lat;
  std::vector<double> heading_rate;
  std::vector<double> acceleration;
  std::vector<double> front_wheel_angle;
  std::vector<double> rear_wheel_angle;
  std::vector<double> time_from_start;
  input_arclength.reserve(input_trajectory.points.size());
  input_pose.reserve(input_trajectory.points.size());
  v_lon.reserve(input_trajectory.points.size());
  v_lat.reserve(input_trajectory.points.size());
  heading_rate.reserve(input_trajectory.points.size());
  acceleration.reserve(input_trajectory.points.size());
  front_wheel_angle.reserve(input_trajectory.points.size());
  rear_wheel_angle.reserve(input_trajectory.points.size());
  time_from_start.reserve(input_trajectory.points.size());

  input_arclength.push_back(0.0);
  input_pose.push_back(input_trajectory.points.front().pose);
  v_lon.push_back(input_trajectory.points.front().longitudinal_velocity_mps);
  v_lat.push_back(input_trajectory.points.front().lateral_velocity_mps);
  heading_rate.push_back(input_trajectory.points.front().heading_rate_rps);
  acceleration.push_back(input_trajectory.points.front().acceleration_mps2);
  front_wheel_angle.push_back(input_trajectory.points.front().front_wheel_angle_rad);
  rear_wheel_angle.push_back(input_trajectory.points.front().rear_wheel_angle_rad);
  time_from_start.push_back(
    rclcpp::Duration(input_trajectory.points.front().time_from_start).seconds());

  for (size_t i = 1; i < input_trajectory.points.size(); ++i) {
    const auto & prev_pt = input_trajectory.points.at(i - 1);
    const auto & curr_pt = input_trajectory.points.at(i);
    const double ds =
      autoware::universe_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);

    input_arclength.push_back(ds + input_arclength.back());
    input_pose.push_back(curr_pt.pose);
    v_lon.push_back(curr_pt.longitudinal_velocity_mps);
    v_lat.push_back(curr_pt.lateral_velocity_mps);
    heading_rate.push_back(curr_pt.heading_rate_rps);
    acceleration.push_back(curr_pt.acceleration_mps2);
    front_wheel_angle.push_back(curr_pt.front_wheel_angle_rad);
    rear_wheel_angle.push_back(curr_pt.rear_wheel_angle_rad);
    time_from_start.push_back(rclcpp::Duration(curr_pt.time_from_start).seconds());
  }

  // Set Zero Velocity After Stop Point
  // If the longitudinal velocity is zero, set the velocity to zero after that point.
  bool stop_point_found_in_v_lon = false;
  constexpr double epsilon = 1e-4;
  for (size_t i = 0; i < v_lon.size(); ++i) {
    if (std::abs(v_lon.at(i)) < epsilon) {
      stop_point_found_in_v_lon = true;
    }
    if (stop_point_found_in_v_lon) {
      v_lon.at(i) = 0.0;
    }
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return autoware::interpolation::lerp(input_arclength, input, resampled_arclength);
  };

  std::vector<size_t> closest_segment_indices;
  if (use_zero_order_hold_for_twist) {
    closest_segment_indices =
      autoware::interpolation::calc_closest_segment_indices(input_arclength, resampled_arclength);
  }
  const auto zoh = [&](const auto & input) {
    return autoware::interpolation::zero_order_hold(
      input_arclength, input, closest_segment_indices);
  };

  const auto interpolated_pose =
    resamplePoseVector(input_pose, resampled_arclength, use_akima_spline_for_xy, use_lerp_for_z);
  const auto interpolated_v_lon = use_zero_order_hold_for_twist ? zoh(v_lon) : lerp(v_lon);
  const auto interpolated_v_lat = use_zero_order_hold_for_twist ? zoh(v_lat) : lerp(v_lat);
  const auto interpolated_heading_rate = lerp(heading_rate);
  const auto interpolated_acceleration =
    use_zero_order_hold_for_twist ? zoh(acceleration) : lerp(acceleration);
  const auto interpolated_front_wheel_angle = lerp(front_wheel_angle);
  const auto interpolated_rear_wheel_angle = lerp(rear_wheel_angle);
  const auto interpolated_time_from_start = lerp(time_from_start);

  if (interpolated_pose.size() != resampled_arclength.size()) {
    std::cerr
      << "[autoware_motion_utils]: Resampled pose size is different from resampled arclength"
      << std::endl;
    return input_trajectory;
  }

  autoware_planning_msgs::msg::Trajectory resampled_trajectory;
  resampled_trajectory.header = input_trajectory.header;
  resampled_trajectory.points.resize(interpolated_pose.size());
  for (size_t i = 0; i < resampled_trajectory.points.size(); ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint traj_point;
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

autoware_planning_msgs::msg::Trajectory resampleTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double resample_interval,
  const bool use_akima_spline_for_xy, const bool use_lerp_for_z,
  const bool use_zero_order_hold_for_twist, const bool resample_input_trajectory_stop_point)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_trajectory.points, resample_interval)) {
    return input_trajectory;
  }

  const double input_trajectory_len =
    autoware::motion_utils::calcArcLength(input_trajectory.points);

  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < input_trajectory_len; s += resample_interval) {
    resampling_arclength.push_back(s);
  }
  if (resampling_arclength.empty()) {
    std::cerr << "[autoware_motion_utils]: resampling arclength is empty" << std::endl;
    return input_trajectory;
  }

  // Insert terminal point
  if (
    input_trajectory_len - resampling_arclength.back() <
    autoware::motion_utils::overlap_threshold) {
    resampling_arclength.back() = input_trajectory_len;
  } else {
    resampling_arclength.push_back(input_trajectory_len);
  }

  // Insert stop point
  if (resample_input_trajectory_stop_point) {
    const auto distance_to_stop_point =
      autoware::motion_utils::calcDistanceToForwardStopPoint(input_trajectory.points, 0);
    if (distance_to_stop_point && !resampling_arclength.empty()) {
      for (size_t i = 1; i < resampling_arclength.size(); ++i) {
        if (
          resampling_arclength.at(i - 1) <= *distance_to_stop_point &&
          *distance_to_stop_point < resampling_arclength.at(i)) {
          const double dist_to_prev_point =
            std::fabs(*distance_to_stop_point - resampling_arclength.at(i - 1));
          const double dist_to_following_point =
            std::fabs(resampling_arclength.at(i) - *distance_to_stop_point);
          if (dist_to_prev_point < autoware::motion_utils::overlap_threshold) {
            resampling_arclength.at(i - 1) = *distance_to_stop_point;
          } else if (dist_to_following_point < autoware::motion_utils::overlap_threshold) {
            resampling_arclength.at(i) = *distance_to_stop_point;
          } else {
            resampling_arclength.insert(resampling_arclength.begin() + i, *distance_to_stop_point);
          }
          break;
        }
      }
    }
  }

  return resampleTrajectory(
    input_trajectory, resampling_arclength, use_akima_spline_for_xy, use_lerp_for_z,
    use_zero_order_hold_for_twist);
}

}  // namespace autoware::motion_utils
