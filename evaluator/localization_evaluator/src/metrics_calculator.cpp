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

#include "localization_evaluator/metrics_calculator.hpp"

#include "localization_evaluator/metrics/localization_metrics.hpp"

namespace localization_diagnostics
{
Stat<double> MetricsCalculator::updateStat(
  const Metric metric, const geometry_msgs::msg::Point & pos,
  const geometry_msgs::msg::Point & pos_gt, const Stat<double> stat_prev) const
{
  switch (metric) {
    case Metric::lateral_error:
    // TODO: remove dummy + 1 with actual ground truth
      return metrics::updateLateralStats(pos.x, pos_gt.x + 1, stat_prev);
    case Metric::absolute_error:
      return metrics::updateAbsoluteStats(pos, pos_gt, stat_prev);
    default:
      throw std::runtime_error(
        "[MetricsCalculator][calculate()] unknown Metric " +
        std::to_string(static_cast<int>(metric)));
  }
}

}  // namespace localization_diagnostics
