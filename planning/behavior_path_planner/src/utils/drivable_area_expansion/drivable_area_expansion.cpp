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

#include "behavior_path_planner/utils/drivable_area_expansion/drivable_area_expansion.hpp"

#include "behavior_path_planner/utils/drivable_area_expansion/expansion.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/footprints.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/map_utils.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/path_projection.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"

#include <Eigen/Geometry>
#include <interpolation/linear_interpolation.hpp>
#include <motion_utils/resample/resample.hpp>
#include <motion_utils/trajectory/interpolation.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/geometry.hpp>

#include <limits>

namespace drivable_area_expansion
{
void reuse_previous_points(
  const PathWithLaneId & path, std::vector<Pose> & prev_poses,
  std::vector<double> & prev_curvatures, const Point & ego_point,
  const DrivableAreaExpansionParameters & params)
{
  std::vector<Pose> cropped_poses;
  std::vector<double> cropped_curvatures;
  const auto ego_is_behind =
    motion_utils::calcLongitudinalOffsetToSegment(prev_poses, 0, ego_point) < 0.0;
  const auto ego_is_far = !prev_poses.empty() &&
                          tier4_autoware_utils::calcDistance2d(ego_point, prev_poses.front()) < 0.0;
  if (!ego_is_behind && !ego_is_far && prev_poses.size() > 1) {
    const auto first_idx =
      motion_utils::findNearestSegmentIndex(prev_poses, path.points.front().point.pose);
    const auto deviation =
      motion_utils::calcLateralOffset(prev_poses, path.points.front().point.pose.position);
    if (first_idx && deviation < params.max_reuse_deviation) {
      for (auto idx = *first_idx; idx < prev_poses.size(); ++idx) {
        if (
          motion_utils::calcLateralOffset(path.points, prev_poses[idx].position) >
          params.max_reuse_deviation)
          break;
        cropped_poses.push_back(prev_poses[idx]);
        cropped_curvatures.push_back(prev_curvatures[idx]);
      }
    }
  }
  const auto resampled_path_points =
    motion_utils::resamplePath(path, params.resample_interval, true, true, false).points;
  auto first_path_idx =
    cropped_poses.empty()
      ? 0LU
      : *motion_utils::findNearestSegmentIndex(resampled_path_points, cropped_poses.back()) + 1;
  // make sure we do not had the same last point or segment
  if (
    !cropped_poses.empty() && resampled_path_points.size() > first_path_idx + 1 &&
    tier4_autoware_utils::calcDistance2d(
      resampled_path_points[first_path_idx + 1], cropped_poses.back()) < 1e-1)
    first_path_idx += 2;
  else if (
    !cropped_poses.empty() && tier4_autoware_utils::calcDistance2d(
                                resampled_path_points[first_path_idx], cropped_poses.back()) < 1e-1)
    ++first_path_idx;
  for (auto idx = first_path_idx; idx < resampled_path_points.size(); ++idx)
    cropped_poses.push_back(resampled_path_points[idx].point.pose);
  cropped_poses = motion_utils::cropForwardPoints(
    cropped_poses, cropped_poses.front().position, 0LU, params.max_path_arc_length);

  prev_poses = cropped_poses;
  prev_curvatures = cropped_curvatures;
}

Point2d convert_point(const Point & p)
{
  return Point2d{p.x, p.y};
}

double calculate_minimum_lane_width(
  const double curvature_radius, const DrivableAreaExpansionParameters & params)
{
  const auto k = curvature_radius;
  const auto a = params.vehicle_info.front_overhang_m + params.extra_front_overhang;
  const auto w = params.vehicle_info.vehicle_width_m + params.extra_width;
  const auto l = params.vehicle_info.wheel_base_m + params.extra_wheelbase;
  return (a * a + 2 * a * l + 2 * k * w + l * l + w * w) / (2 * k + w);
}

std::vector<BoundExpansion> calculate_minimum_expansions(
  const std::vector<Pose> & path_poses, const std::vector<Point> bound,
  const std::vector<double> curvatures, const Side side,
  const DrivableAreaExpansionParameters & params)
{
  std::vector<BoundExpansion> bound_expansions(bound.size());
  size_t lb_idx = 0;
  for (auto path_idx = 0UL; path_idx < path_poses.size(); ++path_idx) {
    const auto & path_pose = path_poses[path_idx];
    if (curvatures[path_idx] == 0.0) continue;
    const auto curvature_radius = 1 / curvatures[path_idx];
    const auto min_lane_width = calculate_minimum_lane_width(curvature_radius, params);
    const auto side_distance = min_lane_width / 2.0 * (side == LEFT ? 1.0 : -1.0);
    const auto offset_point =
      tier4_autoware_utils::calcOffsetPose(path_pose, 0.0, side_distance, 0.0).position;
    for (auto bound_idx = lb_idx; bound_idx + 1 < bound.size(); ++bound_idx) {
      const auto & prev_p = bound[bound_idx];
      const auto & next_p = bound[bound_idx + 1];
      const auto intersection_point =
        tier4_autoware_utils::intersect(offset_point, path_pose.position, prev_p, next_p);
      if (intersection_point) {
        lb_idx = bound_idx;
        const auto dist = tier4_autoware_utils::calcDistance2d(*intersection_point, offset_point);
        if (dist > bound_expansions[bound_idx].expansion_distance) {
          bound_expansions[bound_idx].expansion_distance = dist;
          bound_expansions[bound_idx].expansion_point = offset_point;
          bound_expansions[bound_idx].path_idx = path_idx;
          bound_expansions[bound_idx].bound_segment_idx = bound_idx;
        }
        if (dist > bound_expansions[bound_idx + 1].expansion_distance) {
          bound_expansions[bound_idx + 1].expansion_distance = dist;
          bound_expansions[bound_idx + 1].expansion_point = offset_point;
          bound_expansions[bound_idx + 1].path_idx = path_idx;
          bound_expansions[bound_idx + 1].bound_segment_idx = bound_idx;
        }
        break;
      }
    }
  }
  return bound_expansions;
}

void apply_bound_velocity_limit(
  std::vector<BoundExpansion> & expansions, const std::vector<Point> & bound_vector,
  const double max_velocity)
{
  if (expansions.empty()) return;
  const auto apply_max_vel = [&](auto & exp, const auto from, const auto to) {
    if (exp[from].expansion_distance > exp[to].expansion_distance) {
      const auto arc_length =
        tier4_autoware_utils::calcDistance2d(bound_vector[from], bound_vector[to]);
      const auto smoothed_dist = exp[from].expansion_distance - arc_length * max_velocity;
      exp[to].expansion_distance = std::max(exp[to].expansion_distance, smoothed_dist);
    }
  };
  for (auto idx = 0LU; idx + 1 < expansions.size(); ++idx) apply_max_vel(expansions, idx, idx + 1);
  for (auto idx = expansions.size() - 1; idx > 0; --idx) apply_max_vel(expansions, idx, idx - 1);
}

std::vector<double> calculate_maximum_distance(
  const std::vector<Pose> & path_poses, const std::vector<Point> bound,
  const std::vector<LineString2d> & uncrossable_lines,
  const std::vector<Polygon2d> & uncrossable_polygons,
  const DrivableAreaExpansionParameters & params)
{
  // TODO(Maxime): improve performances (dont use bg::distance ? use rtree ?)
  std::vector<double> maximum_distances(bound.size(), std::numeric_limits<double>::max());
  LineString2d path_ls;
  LineString2d bound_ls;
  for (const auto & p : bound) bound_ls.push_back(convert_point(p));
  for (const auto & p : path_poses) path_ls.push_back(convert_point(p.position));
  for (auto i = 0UL; i + 1 < bound_ls.size(); ++i) {
    const LineString2d segment_ls = {bound_ls[i], bound_ls[i + 1]};
    for (const auto & uncrossable_line : uncrossable_lines) {
      const auto bound_to_line_dist = boost::geometry::distance(segment_ls, uncrossable_line);
      const auto dist_limit = std::max(0.0, bound_to_line_dist - params.avoid_linestring_dist);
      maximum_distances[i] = std::min(maximum_distances[i], dist_limit);
      maximum_distances[i + 1] = std::min(maximum_distances[i + 1], dist_limit);
    }
    for (const auto & uncrossable_poly : uncrossable_polygons) {
      const auto bound_to_poly_dist = boost::geometry::distance(segment_ls, uncrossable_poly);
      maximum_distances[i] = std::min(maximum_distances[i], bound_to_poly_dist);
      maximum_distances[i + 1] = std::min(maximum_distances[i + 1], bound_to_poly_dist);
    }
  }
  if (params.max_expansion_distance > 0.0)
    for (auto & d : maximum_distances) d = std::min(params.max_expansion_distance, d);
  return maximum_distances;
}

void expand_bound(
  std::vector<Point> & bound, const std::vector<Pose> & path_poses,
  const std::vector<BoundExpansion> & expansions)
{
  LineString2d path_ls;
  for (const auto & p : path_poses) path_ls.push_back(convert_point(p.position));
  for (auto idx = 0LU; idx < bound.size(); ++idx) {
    const auto bound_p = convert_point(bound[idx]);
    const auto projection = point_to_linestring_projection(bound_p, path_ls);
    const auto expansion_ratio =
      (expansions[idx].expansion_distance + std::abs(projection.distance)) /
      std::abs(projection.distance);
    if (expansion_ratio > 1.0) {
      const auto & path_p = projection.projected_point;
      const auto expanded_p = lerp_point(path_p, bound_p, expansion_ratio);
      bound[idx].x = expanded_p.x();
      bound[idx].y = expanded_p.y();
    }
  }

  // remove loops TODO(Maxime): move to separate function
  std::vector<Point> no_loop_bound = {bound.front()};
  for (auto idx = 1LU; idx < bound.size(); ++idx) {
    bool is_intersecting = false;
    for (auto succ_idx = idx + 1; succ_idx < bound.size(); ++succ_idx) {
      const auto intersection = tier4_autoware_utils::intersect(
        bound[idx - 1], bound[idx], bound[succ_idx - 1], bound[succ_idx]);
      if (
        intersection &&
        tier4_autoware_utils::calcDistance2d(*intersection, bound[idx - 1]) < 1e-3 &&
        tier4_autoware_utils::calcDistance2d(*intersection, bound[idx]) < 1e-3) {
        idx = succ_idx;
        is_intersecting = true;
      }
    }
    if (!is_intersecting) no_loop_bound.push_back(bound[idx]);
  }
  bound = no_loop_bound;
}

std::vector<double> calculate_smoothed_curvatures(
  const std::vector<Pose> & poses, const size_t smoothing_window_size)
{
  const auto curvatures = motion_utils::calcCurvature(poses);
  std::vector<double> smoothed_curvatures(curvatures.size());
  for (auto i = 0UL; i < curvatures.size(); ++i) {
    auto sum = 0.0;
    const auto from_idx = (i >= smoothing_window_size ? i - smoothing_window_size : 0);
    const auto to_idx = std::min(i + smoothing_window_size, curvatures.size() - 1);
    for (auto j = from_idx; j <= to_idx; ++j) sum += std::abs(curvatures[j]);
    smoothed_curvatures[i] = sum / static_cast<double>(to_idx - from_idx + 1);
  }
  return smoothed_curvatures;
}

void expand_drivable_area(
  PathWithLaneId & path,
  const std::shared_ptr<const behavior_path_planner::PlannerData> planner_data)
{
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("overall");
  stop_watch.tic("preprocessing");
  // crop first/last non deviating path_poses
  const auto & params = planner_data->drivable_area_expansion_parameters;
  const auto & route_handler = *planner_data->route_handler;
  const auto uncrossable_lines = extract_uncrossable_lines(
    *route_handler.getLaneletMapPtr(), planner_data->self_odometry->pose.pose.position, params);
  const auto uncrossable_polygons = create_object_footprints(*planner_data->dynamic_object, params);
  const auto preprocessing_ms = stop_watch.toc("preprocessing");

  stop_watch.tic("crop");
  std::vector<Pose> path_poses = planner_data->drivable_area_expansion_prev_path_poses;
  std::vector<double> curvatures = planner_data->drivable_area_expansion_prev_curvatures;
  reuse_previous_points(
    path, path_poses, curvatures, planner_data->self_odometry->pose.pose.position, params);
  const auto crop_ms = stop_watch.toc("crop");

  stop_watch.tic("curvatures_expansion");
  // Only add curvatures for the new points. Curvatures of reused path points are not updated.
  const auto new_curvatures =
    calculate_smoothed_curvatures(path_poses, params.curvature_average_window);
  const auto first_new_point_idx = curvatures.size();
  curvatures.insert(
    curvatures.end(), new_curvatures.begin() + first_new_point_idx, new_curvatures.end());
  auto left_expansions =
    calculate_minimum_expansions(path_poses, path.left_bound, curvatures, LEFT, params);
  auto right_expansions =
    calculate_minimum_expansions(path_poses, path.right_bound, curvatures, RIGHT, params);
  const auto curv_expansion_ms = stop_watch.toc("curvatures_expansion");

  stop_watch.tic("max_dist");
  const auto max_left_expansions = calculate_maximum_distance(
    path_poses, path.left_bound, uncrossable_lines, uncrossable_polygons, params);
  const auto max_right_expansions = calculate_maximum_distance(
    path_poses, path.right_bound, uncrossable_lines, uncrossable_polygons, params);
  for (auto i = 0LU; i < left_expansions.size(); ++i)
    left_expansions[i].expansion_distance =
      std::min(left_expansions[i].expansion_distance, max_left_expansions[i]);
  for (auto i = 0LU; i < right_expansions.size(); ++i)
    right_expansions[i].expansion_distance =
      std::min(right_expansions[i].expansion_distance, max_right_expansions[i]);
  const auto max_dist_ms = stop_watch.toc("max_dist");

  stop_watch.tic("smooth");
  apply_bound_velocity_limit(left_expansions, path.left_bound, params.max_bound_rate);
  apply_bound_velocity_limit(right_expansions, path.right_bound, params.max_bound_rate);
  const auto smooth_ms = stop_watch.toc("smooth");
  // TODO(Maxime): add an arc length shift or margin ?
  // TODO(Maxime): limit the distances based on the total width (left + right < min_lane_width)
  stop_watch.tic("expand");
  expand_bound(path.left_bound, path_poses, left_expansions);
  expand_bound(path.right_bound, path_poses, right_expansions);
  const auto expand_ms = stop_watch.toc("expand");

  const auto total_ms = stop_watch.toc("overall");
  std::printf(
    "Total runtime(ms): %2.2f\n\tPreprocessing: %2.2f\n\tCrop: %2.2f\n\tCurvature expansion: "
    "%2.2f\n\tMaximum expansion: %2.2f\n\tSmoothing: %2.2f\n\tExpansion: %2.2f\n\n",
    total_ms, preprocessing_ms, crop_ms, curv_expansion_ms, max_dist_ms, smooth_ms, expand_ms);

  planner_data->drivable_area_expansion_prev_path_poses = path_poses;
  planner_data->drivable_area_expansion_prev_curvatures = curvatures;
}
}  // namespace drivable_area_expansion
