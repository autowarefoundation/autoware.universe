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

#pragma once
#include <sophus/geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::common
{
pcl::PointCloud<pcl::PointXYZLNormal> transform_line_segments(
  const pcl::PointCloud<pcl::PointXYZLNormal> & src, const Sophus::SE3f & transform);

pcl::PointCloud<pcl::PointNormal> transform_line_segments(
  const pcl::PointCloud<pcl::PointNormal> & src, const Sophus::SE3f & transform);
}  // namespace pcdless::common