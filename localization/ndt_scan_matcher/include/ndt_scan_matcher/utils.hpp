// Copyright 2022 The Autoware Contributors
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

#ifndef NDT_SCAN_MATCHER__UTILS_HPP_
#define NDT_SCAN_MATCHER__UTILS_HPP_

#include <ndt_scan_matcher/metadata.hpp>

#include <pcl/common/common.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <string>
#include <vector>

namespace loc
{

bool cylinderAndBoxOverlapExists(
  const double center_x, const double center_y, const double radius,
  const pcl::PointXYZ position_min, const pcl::PointXYZ position_max);

void queryContainedSegmentIdx(
  float center_x, float center_y, float radius, const PCDMetadata & m,
  std::list<SegmentIndex> & map_id);
}  // namespace loc

#endif  // NDT_SCAN_MATCHER__UTILS_HPP_
