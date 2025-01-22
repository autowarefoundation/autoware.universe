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

#include "autoware/localization_evaluator/metrics/localization_metrics.hpp"

#include <cmath>

namespace autoware::localization_diagnostics
{
namespace metrics
{

Accumulator<double> updateLateralStats(
  const Accumulator<double> stat_prev, const double & lateral_pos, const double & lateral_ref)
{
  Accumulator<double> stat(stat_prev);
  stat.add(std::abs(lateral_ref - lateral_pos));
  return stat;
}

Accumulator<double> updateAbsoluteStats(
  const Accumulator<double> stat_prev, const geometry_msgs::msg::Point & pos,
  const geometry_msgs::msg::Point & pos_ref)
{
  Accumulator<double> stat(stat_prev);
  double dist = std::sqrt(
    (pos_ref.x - pos.x) * (pos_ref.x - pos.x) + (pos_ref.y - pos.y) * (pos_ref.y - pos.y) +
    (pos_ref.z - pos.z) * (pos_ref.z - pos.z));
  stat.add(dist);

  return stat;
}

}  // namespace metrics
}  // namespace autoware::localization_diagnostics
