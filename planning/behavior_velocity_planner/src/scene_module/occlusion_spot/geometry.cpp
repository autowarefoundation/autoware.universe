// Copyright 2021 Tier IV, Inc.
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

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/occlusion_spot/geometry.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{
using lanelet::BasicLineString2d;
using lanelet::BasicPoint2d;
using lanelet::BasicPolygon2d;
namespace bg = boost::geometry;
namespace lg = lanelet::geometry;

void createOffsetLineString(
  const BasicLineString2d & in, const double offset, BasicLineString2d & offset_line_string)
{
  for (size_t i = 0; i < in.size() - 1; i++) {
    const auto & p0 = in.at(i);
    const auto & p1 = in.at(i + 1);
    // translation
    const double dy = p1[1] - p0[1];
    const double dx = p1[0] - p0[0];
    // rotation (use inverse matrix of rotation)
    const double yaw = std::atan2(dy, dx);
    // translation
    const double offset_x = p0[0] - std::sin(yaw) * offset;
    const double offset_y = p0[1] + std::cos(yaw) * offset;
    offset_line_string.emplace_back(BasicPoint2d{offset_x, offset_y});
    //! insert final offset linestring using prev vertical direction
    if (i == in.size() - 2) {
      const double offset_x = p1[0] - std::sin(yaw) * offset;
      const double offset_y = p1[1] + std::cos(yaw) * offset;
      offset_line_string.emplace_back(BasicPoint2d{offset_x, offset_y});
    }
  }
  return;
}

void buildSlices(
  std::vector<Slice> & slices, const lanelet::ConstLanelet & path_lanelet, const SliceRange & range,
  const double slice_length, const double resolution)
{
  /**
   * @brief bounds
   * +---------- outer bounds
   * |   +------ inner bounds(original path)
   * |   |
   */
  BasicLineString2d inner_bounds = path_lanelet.centerline2d().basicLineString();
  BasicLineString2d outer_bounds;
  if (inner_bounds.size() < 2) return;
  createOffsetLineString(inner_bounds, range.max_distance, outer_bounds);
  const double ratio_dist_start = std::abs(range.min_distance / range.max_distance);
  double next_ratio_dist = ratio_dist_start;
  lanelet::BasicPolygon2d poly;
  const int num_step = static_cast<int>(slice_length / resolution);
  //! max index is the last index of path point
  const int max_index = static_cast<int>(inner_bounds.size() - 1);
  for (int s = 0; s < max_index - 1; s += num_step) {
    const double length = s * slice_length;
    const double next_length = (s + num_step) * resolution;
    Slice slice;
    BasicLineString2d inner_polygons;
    BasicLineString2d outer_polygons;
    // build interpolated polygon for lateral
    for (int i = 0; i <= num_step; i++) {
      if (s + i >= max_index) continue;
      next_ratio_dist = static_cast<double>(s + i) / static_cast<double>(max_index);
      if (ratio_dist_start > next_ratio_dist) continue;
      inner_polygons.emplace_back(
        lerp(inner_bounds.at(s + i), outer_bounds.at(s + i), ratio_dist_start));
      outer_polygons.emplace_back(
        lerp(inner_bounds.at(s + i), outer_bounds.at(s + i), next_ratio_dist));
    }
    if (inner_polygons.empty()) continue;
    //  Build polygon
    inner_polygons.insert(inner_polygons.end(), outer_polygons.rbegin(), outer_polygons.rend());
    slice.polygon = lanelet::BasicPolygon2d(inner_polygons);
    // add range info
    slice.range.min_length = length;
    slice.range.max_length = next_length;
    slice.range.min_distance = ratio_dist_start * range.max_distance;
    slice.range.max_distance = next_ratio_dist * range.max_distance;
    slices.emplace_back(slice);
  }
}

void buildDetectionAreaPolygon(
  std::vector<Slice> & slices, const lanelet::ConstLanelet & path_lanelet, const double offset,
  const PlannerParam & param)
{
  std::vector<Slice> left_slices;
  std::vector<Slice> right_slices;
  const double d_min = param.half_vehicle_width;
  const double d_max = param.detection_area.max_lateral_distance;
  SliceRange left_slice_range = {
    offset + param.baselink_to_front, param.detection_area_length, d_min, d_max};
  // in most case lateral distance is much more effective for velocity planning
  const double slice_length = param.detection_area.slice_length;
  const double resolution = 1.0;  // interpolation interval TODO parametrize this
  buildSlices(left_slices, path_lanelet, left_slice_range, slice_length, resolution);
  SliceRange right_slice_range = {
    offset + param.baselink_to_front, param.detection_area_length, -d_min, -d_max};
  buildSlices(right_slices, path_lanelet, right_slice_range, slice_length, resolution);
  // Properly order lanelets from closest to furthest
  slices = left_slices;
  slices.insert(slices.end(), right_slices.begin(), right_slices.end());
  return;
}
}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
