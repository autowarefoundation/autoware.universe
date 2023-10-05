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

#include <boost/geometry.hpp>

// for writing the svg file
#include <fstream>
#include <iostream>
// for the geometry types
#include <tier4_autoware_utils/geometry/geometry.hpp>
// for the svg mapper
#include <boost/geometry/io/svg/svg_mapper.hpp>
#include <boost/geometry/io/svg/write.hpp>

namespace drivable_area_expansion
{

std::vector<PathPointWithLaneId> crop_and_resample(
  const std::vector<PathPointWithLaneId> & points,
  const std::shared_ptr<const behavior_path_planner::PlannerData> planner_data,
  const double resample_interval)
{
  auto lon_offset = 0.0;
  auto crop_pose = *planner_data->drivable_area_expansion_prev_crop_pose;
  // reuse or update the previous crop point
  if (planner_data->drivable_area_expansion_prev_crop_pose) {
    const auto lon_offset = motion_utils::calcSignedArcLength(
      points, points.front().point.pose.position, crop_pose.position);
    if (lon_offset < 0.0) {
      planner_data->drivable_area_expansion_prev_crop_pose.reset();
    } else {
      const auto is_behind_ego =
        motion_utils::calcSignedArcLength(
          points, crop_pose.position, planner_data->self_odometry->pose.pose.position) > 0.0;
      const auto is_too_far = motion_utils::calcLateralOffset(points, crop_pose.position) > 0.1;
      if (!is_behind_ego || is_too_far)
        planner_data->drivable_area_expansion_prev_crop_pose.reset();
    }
  }
  if (!planner_data->drivable_area_expansion_prev_crop_pose) {
    crop_pose = planner_data->drivable_area_expansion_prev_crop_pose.value_or(
      motion_utils::calcInterpolatedPose(points, resample_interval - lon_offset));
  }
  // crop
  const auto crop_seg_idx = motion_utils::findNearestSegmentIndex(points, crop_pose.position);
  const auto cropped_points = motion_utils::cropPoints(
    points, crop_pose.position, crop_seg_idx,
    planner_data->drivable_area_expansion_parameters.max_path_arc_length, 0.0);
  planner_data->drivable_area_expansion_prev_crop_pose = crop_pose;
  // resample
  PathWithLaneId cropped_path;
  if (tier4_autoware_utils::calcDistance2d(crop_pose, cropped_points.front()) > 1e-3) {
    PathPointWithLaneId crop_path_point;
    crop_path_point.point.pose = crop_pose;
    cropped_path.points.push_back(crop_path_point);
  }
  cropped_path.points.insert(
    cropped_path.points.end(), cropped_points.begin(), cropped_points.end());
  const auto resampled_path =
    motion_utils::resamplePath(cropped_path, resample_interval, true, true, false);

  return resampled_path.points;
}

point_t convert_point(const Point & p)
{
  return point_t{p.x, p.y};
}

double calculate_minimum_lane_width(
  const double curvature_radius, const DrivableAreaExpansionParameters & params)
{
  const auto k = curvature_radius;
  const auto a = params.vehicle_info.front_overhang_m + params.ego_extra_front_offset;
  // TODO(Maxime): update param
  const auto w = params.vehicle_info.vehicle_width_m + params.ego_extra_left_offset;
  const auto l = params.vehicle_info.wheel_base_m;
  return (a * a + 2 * a * l + 2 * k * w + l * l + w * w) / (2 * k + w);
}

std::vector<double> calculate_minimum_lateral_distances(
  const std::vector<PathPointWithLaneId> & points, const std::vector<Point> bound,
  const std::vector<double> curvatures, const Side side,
  const DrivableAreaExpansionParameters & params)
{
  size_t lb_idx = 0;
  std::vector<double> min_lateral_distances(bound.size(), 0.0);
  for (auto path_idx = 0UL; path_idx < points.size(); ++path_idx) {
    const auto & path_p = points[path_idx].point.pose;
    if (curvatures[path_idx] == 0.0) continue;
    const auto curvature_radius = 1 / curvatures[path_idx];
    const auto min_lane_width = calculate_minimum_lane_width(curvature_radius, params);
    const auto side_distance = min_lane_width / 2.0 * (side == LEFT ? -1.0 : 1.0);
    const auto offset_pose =
      tier4_autoware_utils::calcOffsetPose(path_p, 0.0, side_distance, 0.0).position;
    for (auto bound_idx = lb_idx; bound_idx + 1 < bound.size(); ++bound_idx) {
      const auto & prev_p = bound[bound_idx];
      const auto & next_p = bound[bound_idx + 1];
      const auto intersection_point =
        tier4_autoware_utils::intersect(offset_pose, path_p.position, prev_p, next_p);
      if (intersection_point) {
        lb_idx = bound_idx;
        const auto dist = tier4_autoware_utils::calcDistance2d(*intersection_point, offset_pose);
        if (dist > min_lateral_distances[bound_idx]) min_lateral_distances[bound_idx] = dist;
        if (dist > min_lateral_distances[bound_idx + 1])
          min_lateral_distances[bound_idx + 1] = dist;
        break;  // TODO(Maxime): should we rm this break to handle multiple segments intersect ?
      }
    }
  }
  return min_lateral_distances;
}

void apply_velocity_limit(
  std::vector<double> & distance_vector, const std::vector<Point> & bound_vector,
  const double max_velocity)
{
  const auto apply_max_vel = [&](auto & dist_vector, const auto from, const auto to) {
    if (dist_vector[from] > dist_vector[to]) {
      const auto arc_length =
        tier4_autoware_utils::calcDistance2d(bound_vector[from], bound_vector[to]);
      const auto smoothed_dist = dist_vector[from] - arc_length * max_velocity;
      dist_vector[to] = std::max(dist_vector[to], smoothed_dist);
    }
  };
  for (auto idx = 0LU; idx + 1 < distance_vector.size(); ++idx)
    apply_max_vel(distance_vector, idx, idx + 1);
  for (auto idx = distance_vector.size() - 1; idx > 0; --idx)
    apply_max_vel(distance_vector, idx, idx - 1);
}

void expand_bound(
  std::vector<Point> & bound, const std::vector<PathPointWithLaneId> & path_points,
  const std::vector<double> new_distances)
{
  linestring_t path_ls;
  for (const auto & p : path_points) path_ls.push_back(convert_point(p.point.pose.position));
  for (auto idx = 0LU; idx < bound.size(); ++idx) {
    const auto bound_p = convert_point(bound[idx]);
    const auto projection = point_to_linestring_projection(bound_p, path_ls);
    const auto expansion_ratio =
      (new_distances[idx] + std::abs(projection.distance)) / std::abs(projection.distance);
    if (expansion_ratio > 1.0) {
      const auto & path_p = projection.projected_point;
      const auto expanded_p = lerp_point(path_p, bound_p, expansion_ratio);
      bound[idx].x = expanded_p.x();
      bound[idx].y = expanded_p.y();
    }
  }
}

void expandDrivableArea(
  PathWithLaneId & path,
  const std::shared_ptr<const behavior_path_planner::PlannerData> planner_data)
{
  // Declare a stream and an SVG mapper
  std::ofstream svg("/home/mclement/Pictures/da.svg");  // /!\ CHANGE PATH
  boost::geometry::svg_mapper<tier4_autoware_utils::Point2d> mapper(svg, 400, 400);
  linestring_t left_ls, right_ls;
  for (const auto & p : path.left_bound) left_ls.push_back(convert_point(p));
  for (const auto & p : path.right_bound) right_ls.push_back(convert_point(p));
  mapper.add(left_ls);
  mapper.map(left_ls, "fill-opacity:0.3;fill:blue;stroke:blue;stroke-width:2");
  mapper.add(right_ls);
  mapper.map(right_ls, "fill-opacity:0.3;fill:blue;stroke:blue;stroke-width:2");

  const auto & params = planner_data->drivable_area_expansion_parameters;
  const auto & route_handler = *planner_data->route_handler;
  const auto uncrossable_lines =
    extractUncrossableLines(*route_handler.getLaneletMapPtr(), params.avoid_linestring_types);
  multi_linestring_t uncrossable_lines_in_range;
  const auto & p = path.points.front().point.pose.position;
  for (const auto & line : uncrossable_lines)
    if (boost::geometry::distance(line, point_t{p.x, p.y}) < params.max_path_arc_length)
      uncrossable_lines_in_range.push_back(line);

  const auto points = crop_and_resample(path.points, planner_data, params.resample_interval);
  const auto predicted_paths = createObjectFootprints(*planner_data->dynamic_object, params);

  const auto curvatures = motion_utils::calcCurvature(points);
  std::vector<double> smoothed_curvatures(curvatures.size());
  constexpr auto move_avg_window = 3;
  for (auto i = 0UL; i < curvatures.size(); ++i) {
    auto sum = 0.0;
    const auto from_idx = (i >= move_avg_window ? i - move_avg_window : 0);
    const auto to_idx = std::min(i + move_avg_window, curvatures.size() - 1);
    for (auto j = from_idx; j <= to_idx; ++j) sum += std::abs(curvatures[j]);
    smoothed_curvatures[i] = sum / static_cast<double>(to_idx - from_idx + 1);
  }

  auto new_left_distances =
    calculate_minimum_lateral_distances(points, path.left_bound, smoothed_curvatures, LEFT, params);
  auto new_right_distances = calculate_minimum_lateral_distances(
    points, path.right_bound, smoothed_curvatures, RIGHT, params);

  // smooth the distances
  constexpr auto max_bound_vel = 0.5;  // TODO(Maxime): param
  apply_velocity_limit(new_left_distances, path.left_bound, max_bound_vel);
  apply_velocity_limit(new_right_distances, path.right_bound, max_bound_vel);
  // limit the distances based on the uncrossable lines
  // limit the distances based on the total width (left + right < min_lane_width)

  // update the points based on the distances
  // TODO(Maxime): add an arc length offset / margin ?
  expand_bound(path.left_bound, points, new_left_distances);
  expand_bound(path.right_bound, points, new_right_distances);
}
}  // namespace drivable_area_expansion
