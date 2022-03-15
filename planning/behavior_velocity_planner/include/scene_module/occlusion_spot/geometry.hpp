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

#ifndef SCENE_MODULE__OCCLUSION_SPOT__GEOMETRY_HPP_
#define SCENE_MODULE__OCCLUSION_SPOT__GEOMETRY_HPP_

#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>

#include <vector>

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{
//!< @brief wrapper for detection area polygon generation
void buildDetectionAreaPolygon(
  Polygons2d & slices, const PathWithLaneId & path, const double offset,
  const PlannerParam & param);
//!< @brief calculate interpolation between a and b at distance ratio t
template <typename T>
T lerp(T a, T b, double t)
{
  return a + t * (b - a);
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__GEOMETRY_HPP_
