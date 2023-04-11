// Copyright 2020 Tier IV, Inc.
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

#ifndef TIER4_AUTOWARE_UTILS__TRANSFORM__TRANSFORMS_HPP_
#define TIER4_AUTOWARE_UTILS__TRANSFORM__TRANSFORMS_HPP_

#include <Eigen/Core>

#include <pcl/point_cloud.h>

namespace tier4_autoware_utils
{

template <typename PointT>
void transformPointCloud(
  const pcl::PointCloud<PointT> & cloud_in, pcl::PointCloud<PointT> & cloud_out,
  const Eigen::Matrix<float, 4, 4> & transform)
{
  if (&cloud_in != &cloud_out) {
    cloud_out = cloud_in;
  }

  if (cloud_out.width == 0 || cloud_out.height == 0) return;

  if (cloud_in.is_dense) {
    for (std::size_t i = 0; i < cloud_out.size(); ++i) {
      cloud_out[i].data[0] = static_cast<float>(
        transform(0, 0) * cloud_in[i].data[0] + transform(0, 1) * cloud_in[i].data[1] +
        transform(0, 2) * cloud_in[i].data[2] + transform(0, 3));
      cloud_out[i].data[1] = static_cast<float>(
        transform(1, 0) * cloud_in[i].data[0] + transform(1, 1) * cloud_in[i].data[1] +
        transform(1, 2) * cloud_in[i].data[2] + transform(1, 3));
      cloud_out[i].data[2] = static_cast<float>(
        transform(2, 0) * cloud_in[i].data[0] + transform(2, 1) * cloud_in[i].data[1] +
        transform(2, 2) * cloud_in[i].data[2] + transform(2, 3));
      cloud_out[i].data[3] = 1;
    }
  } else {
    for (std::size_t i = 0; i < cloud_out.size(); ++i) {
      if (
        !std::isfinite(cloud_in[i].x) || !std::isfinite(cloud_in[i].y) ||
        !std::isfinite(cloud_in[i].z))
        continue;

      cloud_out[i].data[0] = static_cast<float>(
        transform(0, 0) * cloud_in[i].data[0] + transform(0, 1) * cloud_in[i].data[1] +
        transform(0, 2) * cloud_in[i].data[2] + transform(0, 3));
      cloud_out[i].data[1] = static_cast<float>(
        transform(1, 0) * cloud_in[i].data[0] + transform(1, 1) * cloud_in[i].data[1] +
        transform(1, 2) * cloud_in[i].data[2] + transform(1, 3));
      cloud_out[i].data[2] = static_cast<float>(
        transform(2, 0) * cloud_in[i].data[0] + transform(2, 1) * cloud_in[i].data[1] +
        transform(2, 2) * cloud_in[i].data[2] + transform(2, 3));
      cloud_out[i].data[3] = 1;
    }
  }
}

template <typename PointT>
void transformPointCloud(
  const pcl::PointCloud<PointT> & cloud_in, pcl::PointCloud<PointT> & cloud_out,
  const Eigen::Affine3f & transform)
{
  tier4_autoware_utils::transformPointCloud(cloud_in, cloud_out, transform.matrix());
}

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__TRANSFORM__TRANSFORMS_HPP_
