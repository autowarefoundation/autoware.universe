// Copyright 2022 Tier IV, Inc.
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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__POINTCLOUD_UTILS_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__POINTCLOUD_UTILS_HPP_

#include "apparent_safe_velocity_limiter/collision_distance.hpp"
#include "apparent_safe_velocity_limiter/types.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <geometry_msgs/msg/transform.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace apparent_safe_velocity_limiter
{

/// @brief return the pointcloud msg transformed and converted to PCL format
/// @param[in] pointcloud_msg pointcloud to transform
/// @param[in] transform transform to use
/// @return PCL pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
  const PointCloud & pointcloud_msg, const geometry_msgs::msg::Transform & transform);

/// @brief filter the pointcloud to keep only relevent points
/// @param[in,out] pointcloud to filter
/// @param[in] polygon_masks polygons to mask from the pointcloud
/// @param[in] envelope polygon where points of the pointcloud must be
void filterPointCloud(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const multipolygon_t & polygon_masks,
  const polygon_t & envelope);

/// @brief returns the pointcloud transformed to the trajectory frame and in PCL format with only
/// points that are inside the envelope and outside of the masks
/// @param[in] pointcloud pointcloud to transform and filter
/// @param[in] polygon_masks polygons to mask from the pointcloud
/// @param[in] envelope polygon where points of the pointcloud must be
/// @param[in] transform_listener used to retrieve the latest transform
/// @param[in] target_frame target_frame used for transformation
pcl::PointCloud<pcl::PointXYZ>::Ptr transformAndFilterPointCloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud, const multipolygon_t & polygon_masks,
  const polygon_t & envelope, tier4_autoware_utils::TransformListener & transform_listener,
  const std::string & target_frame);

/// @brief extract lines around obstacle clusters in the given pointcloud
/// @param[in] pointcloud input pointcloud
/// @param[in] cluster_tolerance minimum distance between two clusters
/// @return linestrings around obstacle clusters
multilinestring_t extractObstacleLines(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const double cluster_tolerance);

}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__POINTCLOUD_UTILS_HPP_
