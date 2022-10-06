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

#include "segmentation_evaluator/metrics_calculator.hpp"

#include "segmentation_evaluator/metrics/segmentation_metrics.hpp"

namespace segmentation_diagnostics
{
Stat<double> MetricsCalculator::updateStat(
  const Stat<double> stat_prev, const Metric metric, const PointCloud2 & pcl,
  const PointCloud2 & pcl_gt_negative_cls, const PointCloud2 & pcl_gt_positive_cls) const
{
  switch (metric) {
    case Metric::segment_stat:
      return metrics::updatePclStats(pcl, pcl_gt_negative_cls, pcl_gt_positive_cls, stat_prev);
    default:
      throw std::runtime_error(
        "[MetricsCalculator][calculate()] unknown Metric " +
        std::to_string(static_cast<int>(metric)));
  }
}

}  // namespace segmentation_diagnostics
