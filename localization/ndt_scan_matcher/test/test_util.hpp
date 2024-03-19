// Copyright 2024 Autoware Foundation
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

#ifndef TEST_UTIL_HPP_
#define TEST_UTIL_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

inline pcl::PointCloud<pcl::PointXYZ> make_sample_half_cubic_pcd()
{
  constexpr float length = 10;
  constexpr float interval = 0.1;
  constexpr int num_points_per_line = static_cast<int>(length / interval) + 1;
  constexpr int num_points_per_plane = num_points_per_line * num_points_per_line;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 3 * num_points_per_plane;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  for (int i = 0; i < num_points_per_line; ++i) {
    for (int j = 0; j < num_points_per_line; ++j) {
      const float u = interval * static_cast<float>(j);
      const float v = interval * static_cast<float>(i);

      // xy
      cloud.points[num_points_per_plane * 0 + i * num_points_per_line + j].x = u;  // NOLINT
      cloud.points[num_points_per_plane * 0 + i * num_points_per_line + j].y = v;  // NOLINT
      cloud.points[num_points_per_plane * 0 + i * num_points_per_line + j].z = 0;  // NOLINT

      // yz
      cloud.points[num_points_per_plane * 1 + i * num_points_per_line + j].x = 0;  // NOLINT
      cloud.points[num_points_per_plane * 1 + i * num_points_per_line + j].y = u;  // NOLINT
      cloud.points[num_points_per_plane * 1 + i * num_points_per_line + j].z = v;  // NOLINT

      // zx
      cloud.points[num_points_per_plane * 2 + i * num_points_per_line + j].x = u;  // NOLINT
      cloud.points[num_points_per_plane * 2 + i * num_points_per_line + j].y = 0;  // NOLINT
      cloud.points[num_points_per_plane * 2 + i * num_points_per_line + j].z = v;  // NOLINT
    }
  }
  return cloud;
}

#endif  // TEST_UTIL_HPP_
