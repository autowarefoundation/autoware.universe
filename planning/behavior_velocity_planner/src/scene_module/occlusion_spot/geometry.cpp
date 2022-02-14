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
#include <scene_module/occlusion_spot/risk_predictive_braking.hpp>

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

BasicPoint2d calculateOffsetPoint(
  const BasicPoint2d & p0, const BasicPoint2d & p1, const double offset)
{
  // translation
  const double dy = p1[1] - p0[1];
  const double dx = p1[0] - p0[0];
  // rotation (use inverse matrix of rotation)
  const double yaw = std::atan2(dy, dx);
  const double offset_x = p1[0] - std::sin(yaw) * offset;
  const double offset_y = p1[1] + std::cos(yaw) * offset;
  return BasicPoint2d{offset_x, offset_y};
}

void buildSlices(
  std::vector<Slice> & slices, const lanelet::ConstLanelet & path_lanelet, const SliceRange & range,
  const PlannerParam & param)
{
  /**
   * @brief bounds
   * +---------- outer bounds
   * |   +------ inner bounds(original path)
   * |   |
   */
  BasicLineString2d center_line = path_lanelet.centerline2d().basicLineString();
  if (center_line.size() < 2) return;
  const double slice_length = param.detection_area.slice_length;
  const bool is_on_right = range.min_distance < 0;
  const int num_step = static_cast<int>(slice_length);
  //! max index is the last index of path point
  const int max_index = static_cast<int>(center_line.size() - 2);
  int idx = 0;
  for (int s = 0; s < max_index; s += num_step) {
    const double length = s * slice_length;
    const double next_length = static_cast<double>(s + num_step);
    Slice slice;
    BasicLineString2d inner_polygons;
    BasicLineString2d outer_polygons;
    // build interpolated polygon for lateral
    for (int i = 0; i <= num_step; i++) {
      idx = s + i;
      const double arc_length_from_ego = static_cast<double>(idx) - range.min_length;
      if (arc_length_from_ego < 0) continue;
      if (idx >= max_index) continue;
      const auto & c0 = center_line.at(idx);
      const auto & c1 = center_line.at(idx + 1);
      const BasicPoint2d inner_point = calculateOffsetPoint(c0, c1, range.min_distance);
      double lateral_distance = calculateLateralDistanceFromTTC(arc_length_from_ego, param);
      if (is_on_right) lateral_distance *= -1;
      const BasicPoint2d outer_point = calculateOffsetPoint(c0, c1, lateral_distance);
      inner_polygons.emplace_back(inner_point);
      outer_polygons.emplace_back(outer_point);
    }
    if (inner_polygons.empty()) continue;
    //  Build polygon
    inner_polygons.insert(inner_polygons.end(), outer_polygons.rbegin(), outer_polygons.rend());
    slice.polygon = lanelet::BasicPolygon2d(inner_polygons);
    // add range info
    slice.range.min_length = length;
    slice.range.max_length = next_length;
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
  SliceRange left_slice_range = {offset, param.detection_area_length, d_min, d_max};
  // in most case lateral distance is much more effective for velocity planning
  buildSlices(left_slices, path_lanelet, left_slice_range, param);
  SliceRange right_slice_range = {offset, param.detection_area_length, -d_min, -d_max};
  buildSlices(right_slices, path_lanelet, right_slice_range, param);
  // Properly order lanelets from closest to furthest
  slices = left_slices;
  slices.insert(slices.end(), right_slices.begin(), right_slices.end());
  return;
}
}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
