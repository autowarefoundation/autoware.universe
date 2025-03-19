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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__STEER_ACCUMULATOR_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__STEER_ACCUMULATOR_HPP_

#include "autoware/planning_evaluator/metrics/metric.hpp"
#include "autoware/planning_evaluator/metrics/output_metric.hpp"

#include <autoware_utils/math/accumulator.hpp>
#include <nlohmann/json.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <deque>

namespace planning_diagnostics
{
using autoware_utils::Accumulator;
using autoware_vehicle_msgs::msg::SteeringReport;
using MetricMsg = tier4_metric_msgs::msg::Metric;
using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;
using json = nlohmann::json;

/**
 * @class SteerAccumulator
 * @brief Accumulator to generate steer-related metrics
 */
class SteerAccumulator
{
public:
  struct Parameters
  {
    double window_duration_s = 5.0;  // [s] Duration to count steer_rate changes for publishing
    double steer_rate_margin_radps =
      0.1;       // [rad/s] margin of steer_rate around 0 to count as steering change
  } parameters;  // struct Parameters

  SteerAccumulator() = default;
  ~SteerAccumulator() = default;

  /**
   * @brief update the accumulator with new steering data
   * @param msg new SteeringReport msg to update the accumulator state
   */
  void update(const SteeringReport & msg);

  /**
   * @brief add blinker-related metrics to the metrics_msg for publishing
   * @param metric metric to add to the metrics_msg
   * @param metrics_msg MetricArrayMsg to add the metric to
   */
  bool addMetricMsg(const Metric & metric, MetricArrayMsg & metrics_msg) const;

  /**
   * @brief get the output json data for the OutputMetric
   * @return json data
   */
  json getOutputJson(const OutputMetric & output_metric) const;

private:
  // state and statistics
  enum class SteerRateState { AROUND_ZERO, POSITIVE, NEGATIVE };
  struct SteerState
  {
    double steer_angle;
    double steer_rate;
    double timestamp;
    SteerRateState steer_rate_state;

    SteerState()
    : steer_angle(std::numeric_limits<double>::quiet_NaN()),
      steer_rate(std::numeric_limits<double>::quiet_NaN()),
      timestamp(std::numeric_limits<double>::quiet_NaN()),
      steer_rate_state(SteerRateState::AROUND_ZERO){};
    bool has_steer_rate() const { return std::isfinite(steer_rate); }
    bool has_steer_angle() const { return std::isfinite(steer_angle); }
  } steer_state_;

  double steer_rate_change_count_total_ = 0;
  double steer_rate_change_count_in_window_ = 0;
  std::deque<double> steer_rate_change_window_;
  Accumulator<double> steer_rate_change_count_accumulator_;
};

}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__STEER_ACCUMULATOR_HPP_