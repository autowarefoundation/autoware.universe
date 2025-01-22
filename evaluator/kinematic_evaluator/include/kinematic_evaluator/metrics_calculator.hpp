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

#ifndef KINEMATIC_EVALUATOR__METRICS_CALCULATOR_HPP_
#define KINEMATIC_EVALUATOR__METRICS_CALCULATOR_HPP_

#include "autoware/universe_utils/math/accumulator.hpp"
#include "kinematic_evaluator/metrics/metric.hpp"
#include "kinematic_evaluator/parameters.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include <nav_msgs/msg/odometry.hpp>

namespace kinematic_diagnostics
{
using autoware::universe_utils::Accumulator;
using nav_msgs::msg::Odometry;

class MetricsCalculator
{
public:
  Parameters parameters;

  MetricsCalculator() = default;

  /**
   * @brief calculate
   * @param [in] metric Metric enum value
   * @param [in] odom Odometry
   * @return string describing the requested metric
   */
  Accumulator<double> calculate(const Metric metric, const Odometry & odom) const;

  /**
   * @brief update Metrics
   * @param [in] metric Previous metric enum value
   * @param [in] odom Odometry
   * @return string describing the requested metric
   */
  Accumulator<double> updateStat(
    const Metric metric, const Odometry & odom, const Accumulator<double> stat_prev) const;
};  // class MetricsCalculator

}  // namespace kinematic_diagnostics

#endif  // KINEMATIC_EVALUATOR__METRICS_CALCULATOR_HPP_
