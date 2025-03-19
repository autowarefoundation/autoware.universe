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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS_PLANNING_FACTOR_ACCUMULATOR_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS_PLANNING_FACTOR_ACCUMULATOR_HPP_

#include "autoware/planning_evaluator/metrics/metric.hpp"
#include "autoware/planning_evaluator/metrics/output_metric.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/math/accumulator.hpp>
#include <nlohmann/json.hpp>

#include <autoware_internal_planning_msgs/msg/control_point.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <unordered_map>
#include <unordered_set>

namespace planning_diagnostics
{
using autoware_utils::Accumulator;
using MetricMsg = tier4_metric_msgs::msg::Metric;
using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;
using autoware_internal_planning_msgs::msg::ControlPoint;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::PlanningFactorArray;
using json = nlohmann::json;
using nav_msgs::msg::Odometry;

/**
 * @class PlanningFactorAccumulator
 * @brief Accumulator to generate planning factor-related metrics from PlanningFactorArray messages.
 */
class PlanningFactorAccumulator
{
public:
  struct Parameters
  {
    double time_count_threshold_s = 60.0;  // [s] time threshold to count a stop as a new one
    double dist_count_threshold_m = 5.0;   // [m] distance threshold to count a stop as a new one
    double abnormal_deceleration_threshold_mps2 =
      2.0;       // [m/s^2] deceleration threshold for a stop to be considered as abnormal
  } parameters;  // struct Parameters

  PlanningFactorAccumulator() = default;
  ~PlanningFactorAccumulator() = default;

  /**
   * @brief update the status with new planning factor data
   * @param module_name the module name of the planning factor data
   * @param planning_factors the planning factor data
   * @param ego_state the ego odometry data
   */
  void update(
    const std::string & module_name, const PlanningFactorArray & planning_factors,
    const Odometry & ego_state);

  /**
   * @brief add the metric message to the MetricArrayMsg for the given Metric and module name
    * @param metric the metric to add to the MetricArrayMsg
    * @param metrics_msg the MetricArrayMsg to add the metric to
   */
  bool addMetricMsg(const Metric & metric, MetricArrayMsg & metrics_msg) const;

  /**
   * @brief get the output json data for the OutputMetric
   */
  json getOutputJson(const OutputMetric & output_metric);

private:
  // Stop decision's state
  struct StopDecisionState
  {
    double last_stop_time, last_stop_x, last_stop_y, last_stop_z;
    double stop_decision_keep_time, distance_to_stop;
    bool is_stop_decision;
    Accumulator<double> stop_decision_keep_time_accumulator;

    explicit StopDecisionState()
    : last_stop_time(0.0),
      last_stop_x(0.0),
      last_stop_y(0.0),
      last_stop_z(0.0),
      stop_decision_keep_time(std::numeric_limits<double>::quiet_NaN()),
      distance_to_stop(std::numeric_limits<double>::quiet_NaN()),
      is_stop_decision(false),
      stop_decision_keep_time_accumulator()
    {
    }
  };
  std::unordered_map<std::string, StopDecisionState> stop_decision_state_,
    abnormal_stop_decision_state_;
  std::unordered_set<std::string> stop_decision_modules_;
};
}  // namespace planning_diagnostics
#endif  // AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS_PLANNING_FACTOR_ACCUMULATOR_HPP_
