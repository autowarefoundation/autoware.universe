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

#ifndef SEGMENTATION_EVALUATOR__METRICS__METRIC_HPP_
#define SEGMENTATION_EVALUATOR__METRICS__METRIC_HPP_

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace segmentation_diagnostics
{
/**
 * @brief Enumeration of velocity metrics
 */
enum class Metric {
  segment_stat,
  SIZE,
};

static const std::unordered_map<std::string, Metric> str_to_metric = {
  {"segment_stat", Metric::segment_stat}
};
static const std::unordered_map<Metric, std::string> metric_to_str = {
  {Metric::segment_stat, "segment_stat"}
};

// Metrics descriptions
static const std::unordered_map<Metric, std::string> metric_descriptions = {
  {Metric::segment_stat, "segment_stat [percentage]"}
};

namespace details
{
static struct CheckCorrectMaps
{
  CheckCorrectMaps()
  {
    if (
      str_to_metric.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_to_str.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_descriptions.size() != static_cast<size_t>(Metric::SIZE)) {
      std::cerr << "[metrics/metrics.hpp] Maps are not defined for all metrics: ";
      std::cerr << str_to_metric.size() << " " << metric_to_str.size() << " "
                << metric_descriptions.size() << std::endl;
    }
  }
} check;

}  // namespace details
}  // namespace segmentation_diagnostics

#endif  // SEGMENTATION_EVALUATOR__METRICS__METRIC_HPP_
