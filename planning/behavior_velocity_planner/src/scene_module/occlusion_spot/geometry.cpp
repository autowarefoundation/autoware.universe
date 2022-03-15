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
#include <utilization/boost_geometry_helper.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{
void createDetectionAreaPolygons(
  Polygons2d & da_polys, const PathWithLaneId & path, const DetectionRange da_range,
  const double obstacle_vel_mps)
{
  using planning_utils::calculateLateralOffsetPoint2d;
  /**
   * @brief relationships for interpolated polygon
   *
   * +(min_length,max_distance)-+ - +---+(max_length,max_distance) = outer_polygons
   * |                                  |
   * +--------------------------+ - +---+(max_length,min_distance) = inner_polygons
   */
  const double min_len = da_range.min_longitudinal_distance;
  const double max_len = da_range.max_longitudinal_distance;
  const double min_dst = da_range.min_lateral_distance;
  const double max_dst = da_range.max_lateral_distance;
  const double interval = da_range.interval;
  //! max index is the last index of path point
  const int max_index = static_cast<int>(path.points.size() - 1);
  double dist_sum = -min_len;
  double ttc = 0.0;
  double length = dist_sum;
  auto p0 = path.points.at(0).point;
  // initial condition
  LineString2d left_inner_bound;
  LineString2d left_outer_bound;
  LineString2d right_inner_bound;
  LineString2d right_outer_bound;
  for (int s = 0; s <= max_index; s++) {
    const auto p1 = path.points.at(s).point;
    const double ds = tier4_autoware_utils::calcDistance2d(p0, p1);
    dist_sum += ds;
    length += ds;
    if (dist_sum <= 0) {
      p0 = p1;
      continue;
    }
    // calculate the distance that obstacles can move until ego reach the trajectory point
    const double v_average = 0.5 * (p0.longitudinal_velocity_mps + p1.longitudinal_velocity_mps);
    const double v = std::max(v_average, 0.3);
    const double dt = ds / v;
    ttc += dt;
    //! avoid bug with same point polygon
    const double eps = 1e-3;
    // for offset calculation
    const double max_lateral_distance = std::min(max_dst, min_dst + ttc * obstacle_vel_mps + eps);
    // left bound
    if (da_range.use_left) {
      left_inner_bound.emplace_back(calculateLateralOffsetPoint2d(p0.pose, min_dst));
      left_outer_bound.emplace_back(calculateLateralOffsetPoint2d(p0.pose, max_lateral_distance));
    }
    // right bound
    if (da_range.use_right) {
      right_inner_bound.emplace_back(calculateLateralOffsetPoint2d(p0.pose, -min_dst));
      right_outer_bound.emplace_back(calculateLateralOffsetPoint2d(p0.pose, -max_lateral_distance));
    }
    // replace previous point with next point
    p0 = p1;
    // separate detection area polygon with fixed interval or at the end of detection max length
    if (length > interval || max_len < dist_sum || s == max_index) {
      if (left_inner_bound.size() > 1)
        da_polys.emplace_back(lines2polygon(left_inner_bound, left_outer_bound));
      if (right_inner_bound.size() > 1)
        da_polys.emplace_back(lines2polygon(right_outer_bound, right_inner_bound));
      left_inner_bound = {left_inner_bound.back()};
      left_outer_bound = {left_outer_bound.back()};
      right_inner_bound = {right_inner_bound.back()};
      right_outer_bound = {right_outer_bound.back()};
      length = 0;
      continue;
    }
  }
}

PathWithLaneId applyVelocityToPath(const PathWithLaneId & path, const double v0)
{
  PathWithLaneId out;
  for (size_t i = 0; i < path.points.size(); i++) {
    PathPointWithLaneId p = path.points.at(i);
    p.point.longitudinal_velocity_mps = std::max(v0, 1.0);
    out.points.emplace_back(p);
  }
  return out;
}

void buildDetectionAreaPolygon(
  Polygons2d & slices, const PathWithLaneId & path, const double offset, const PlannerParam & param)
{
  const auto & p = param;
  [[maybe_unused]] const double min_len = offset + p.baselink_to_front;
  [[maybe_unused]] const double max_len = 50;
  //    std::min(p.detection_area_max_length, p.detection_area_length);
  [[maybe_unused]] const double min_dst = p.half_vehicle_width;
  [[maybe_unused]] const double max_dst = p.detection_area.max_lateral_distance;
  DetectionRange da_range = {p.detection_area.slice_length, min_len, max_len, min_dst, max_dst};
  PathWithLaneId applied_path = applyVelocityToPath(path, param.v.v_ego);
  // in most case lateral distance is much more effective for velocity planning
  createDetectionAreaPolygons(slices, path, da_range, p.pedestrian_vel);
  return;
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
