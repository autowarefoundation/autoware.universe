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

#include "drivable_area_expander/path_preprocessing.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace drivable_area_expander
{

size_t calculateStartIndex(const Path & path, const size_t ego_idx, const double backward_length)
{
  auto length = 0.0;
  auto idx = ego_idx;
  while (idx >= 0 && length < backward_length) {
    length += tier4_autoware_utils::calcDistance2d(path.points[idx], path.points[idx + 1]);
    --idx;
  }
  return idx;
}

size_t calculateEndIndex(const Path & path, const size_t start_idx, const double forward_length)
{
  auto length = 0.0;
  auto idx = start_idx;
  while (idx + 1 < path.points.size() && length < forward_length) {
    length += tier4_autoware_utils::calcDistance2d(path.points[idx], path.points[idx + 1]);
    ++idx;
  }
  return idx;
}

Path downsamplePath(
  const Path & path, const size_t start_idx, const size_t end_idx, const int factor)
{
  if (factor < 1) return path;
  Path downsampled_traj;
  downsampled_traj.header = path.header;
  downsampled_traj.points.reserve((end_idx - start_idx) / factor);
  for (size_t i = start_idx; i <= end_idx; i += factor)
    downsampled_traj.points.push_back(path.points[i]);
  return downsampled_traj;
}
}  // namespace drivable_area_expander
