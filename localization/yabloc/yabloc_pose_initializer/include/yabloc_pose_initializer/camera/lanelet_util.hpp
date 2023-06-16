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

#ifndef YABLOC_POSE_INITIALIZER__CAMERA__LANELET_UTIL_HPP_
#define YABLOC_POSE_INITIALIZER__CAMERA__LANELET_UTIL_HPP_

#include <lanelet2_core/LaneletMap.h>

#include <optional>

// TODO(KYabuuchi) Reuse autoware_common/lanelet2_extension instead of this
namespace lanelet
{
std::optional<double> get_current_direction(
  const ConstLanelets & lanelets, const Eigen::Vector3f & query_position);

bool get_current_lanelets(
  const ConstLanelets & lanelets, const Eigen::Vector3f & query_position,
  ConstLanelets * current_lanelets_ptr);

ConstLineString3d get_closest_segment(
  const lanelet::BasicPoint2d & search_pt, const lanelet::ConstLineString3d & linestring);

}  // namespace lanelet

#endif  // YABLOC_POSE_INITIALIZER__CAMERA__LANELET_UTIL_HPP_
