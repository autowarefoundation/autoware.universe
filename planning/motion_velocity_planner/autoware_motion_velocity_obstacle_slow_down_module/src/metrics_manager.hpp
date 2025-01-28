// Copyright 2025 TIER IV, Inc.
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

#ifndef METRICS_MANAGER_HPP_
#define METRICS_MANAGER_HPP_

#include "type_alias.hpp"
#include "types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
class MetricsManager
{
public:
  void init() { metrics_.clear(); }

  void calculate_metrics(const std::string & module_name, const std::string & reason)
  {
    // Create status
    {
      // Decision
      Metric decision_metric;
      decision_metric.name = module_name + "/decision";
      decision_metric.unit = "string";
      decision_metric.value = reason;
      metrics_.push_back(decision_metric);
    }
  }

  MetricArray create_metric_array(const rclcpp::Time & current_time)
  {
    MetricArray metrics_msg;
    metrics_msg.stamp = current_time;
    metrics_msg.metric_array.insert(
      metrics_msg.metric_array.end(), metrics_.begin(), metrics_.end());
    return metrics_msg;
  }

private:
  std::vector<Metric> metrics_;
};

}  // namespace autoware::motion_velocity_planner

#endif  // METRICS_MANAGER_HPP_
