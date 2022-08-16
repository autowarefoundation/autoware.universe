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

#ifndef SEGMENTATION_EVALUATOR__METRICS_CALCULATOR_HPP_
#define SEGMENTATION_EVALUATOR__METRICS_CALCULATOR_HPP_

#include "segmentation_evaluator/metrics/metric.hpp"
#include "segmentation_evaluator/parameters.hpp"
#include "segmentation_evaluator/stat.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace segmentation_diagnostics
{
using sensor_msgs::msg::PointCloud2;

class MetricsCalculator
{
public:
  Parameters parameters;

  MetricsCalculator() = default;

  /**
   * @brief calculate
   * @param [in] metric Metric enum value
   * @param [in] pcl Point Cloud
   * @param [in] pcl_ref reference Point Cloud
   * @return string describing the requested metric
   */
  Stat<double> calculate(
    const Metric metric, const PointCloud2 pcl, const PointCloud2 pcl_ref) const;

  /**
   * @brief update Metrics
   * @param [in] metric Previous metric enum value
   * @param [in] stat_prev Previous metric
   * @param [in] pcl Point Cloud
   * @param [in] pcl_gt_ground Ground truth point cloud with ground
   * @param [in] pcl_gt_obj Ground truth point cloud with objects
   * @return string describing the requested metric
   */
  Stat<double> updateStat(
    const Stat<double> stat_prev, const Metric metric, const PointCloud2 & pcl,
    const PointCloud2 & pcl_gt_ground, const PointCloud2 & pcl_gt_obj,
    PointCloud2 & pcl_no_ex) const;
};  // class MetricsCalculator

}  // namespace segmentation_diagnostics

#endif  // SEGMENTATION_EVALUATOR__METRICS_CALCULATOR_HPP_
