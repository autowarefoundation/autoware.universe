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

inline pcl::PointCloud<pcl::PointXYZ> make_sample_pcd(
  const float min_xy, const float max_xy, const float interval)
{
  const float range_width = max_xy - min_xy;
  const float center = (max_xy + min_xy) / 2.0f;
  const int num_points_per_line = static_cast<int>(range_width / interval) + 1;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = num_points_per_line * num_points_per_line;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  for (int i = 0; i < num_points_per_line; ++i) {
    for (int j = 0; j < num_points_per_line; ++j) {
      const float x = min_xy + interval * static_cast<float>(j);
      const float y = min_xy + interval * static_cast<float>(i);
      const float z = std::hypot(x - center, y - center) / (range_width / 16);
      cloud.points[i * num_points_per_line + j].x = x;
      cloud.points[i * num_points_per_line + j].y = y;
      cloud.points[i * num_points_per_line + j].z = z;
    }
  }
  return cloud;
}

#endif  // TEST_UTIL_HPP_
