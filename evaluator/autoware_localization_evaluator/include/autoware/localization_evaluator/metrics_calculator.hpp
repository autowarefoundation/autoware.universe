// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__LOCALIZATION_EVALUATOR__METRICS_CALCULATOR_HPP_
#define AUTOWARE__LOCALIZATION_EVALUATOR__METRICS_CALCULATOR_HPP_

#include "autoware/localization_evaluator/metrics/metric.hpp"
#include "autoware/localization_evaluator/parameters.hpp"
#include "autoware/universe_utils/math/accumulator.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::localization_diagnostics
{
using autoware::universe_utils::Accumulator;
class MetricsCalculator
{
public:
  Parameters parameters;

  MetricsCalculator() = default;
  /**
   * @brief update Metrics
   * @param [in] stat_prev Previous statistics
   * @param [in] metric metric enum value
   * @param [in] pos current position
   * @param [in] pos_ref reference position
   * @return string describing the requested metric
   */
  Accumulator<double> updateStat(
    const Accumulator<double> stat_prev, const Metric metric, const geometry_msgs::msg::Point & pos,
    const geometry_msgs::msg::Point & pos_ref) const;
};  // class MetricsCalculator

}  // namespace autoware::localization_diagnostics

#endif  // AUTOWARE__LOCALIZATION_EVALUATOR__METRICS_CALCULATOR_HPP_
