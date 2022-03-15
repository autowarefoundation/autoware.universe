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

#include <rclcpp/rclcpp.hpp>
#include <scene_module/occlusion_spot/geometry.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{

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
  DetectionRange da_range;
  da_range.interval = p.detection_area.slice_length;
  da_range.min_longitudinal_distance = offset + p.baselink_to_front;
  da_range.max_longitudinal_distance =
    std::min(p.detection_area_max_length, p.detection_area_length);
  da_range.min_lateral_distance = p.half_vehicle_width;
  da_range.max_lateral_distance = p.detection_area.max_lateral_distance;
  PathWithLaneId applied_path = applyVelocityToPath(path, param.v.v_ego);
  // in most case lateral distance is much more effective for velocity planning
  planning_utils::createDetectionAreaPolygons(slices, path, da_range, p.pedestrian_vel);
  return;
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
