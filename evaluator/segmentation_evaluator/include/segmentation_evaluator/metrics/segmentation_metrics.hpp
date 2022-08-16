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

#ifndef SEGMENTATION_EVALUATOR__METRICS__SEGMENTATION_METRICS_HPP_
#define SEGMENTATION_EVALUATOR__METRICS__SEGMENTATION_METRICS_HPP_

#include "segmentation_evaluator/stat.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace segmentation_diagnostics
{
namespace metrics
{
using sensor_msgs::msg::PointCloud2;

/**
 * @brief Update statistics
 * @param [in] value New value
 * @param [in] stat_prev Previous statistics
 * @return Calculated statistics
 */
Stat<double> updatePclStats(
  const PointCloud2 & pcl, const PointCloud2 & pcl_gt_ground, const PointCloud2 & pcl_gt_obj,
  const Stat<double> stat_prev, PointCloud2 & pcl_no_ex);

/**
 * @brief Compute confustion matrix for point clouds
 * @param [in] pcl Point cloud
 * @param [in] pcl_gt_ground Ground truth point cloud with ground
 * @param [in] pcl_gt_obj Ground truth point cloud with objects
 * @return calculated statistics
 */
std::vector<int> computeConfusionMatrix(
  const PointCloud2 & pcl, const PointCloud2 & pcl_gt_ground, const PointCloud2 & pcl_gt_obj, PointCloud2 & pcl_no_ex);

}  // namespace metrics
}  // namespace segmentation_diagnostics

#endif  // SEGMENTATION_EVALUATOR__METRICS__SEGMENTATION_METRICS_HPP_
