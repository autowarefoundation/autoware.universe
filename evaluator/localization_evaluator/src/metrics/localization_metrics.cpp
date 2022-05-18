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

#include "localization_evaluator/metrics/localization_metrics.hpp"

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

namespace localization_diagnostics
{
namespace metrics
{

Stat<double> updateLateralStats(
  const double & lateral_pos, const double & lateral_gt, const Stat<double> stat_prev)
{
  Stat<double> stat(stat_prev);
  stat.add(std::abs(lateral_gt - lateral_pos));

  return stat;
}

Stat<double> updateAbsoluteStats(
  const geometry_msgs::msg::Point & pos, const geometry_msgs::msg::Point & pos_gt,
  const Stat<double> stat_prev)
{
  // TODO: remove dummy with actual ground truth
  geometry_msgs::msg::Point dummy_pos(pos_gt);
  dummy_pos.y += 3;
  dummy_pos.x += 4;

  Stat<double> stat(stat_prev);
  double dist = std::sqrt(
    (dummy_pos.x - pos.x) * (dummy_pos.x - pos.x) + (dummy_pos.y - pos.y) * (dummy_pos.y - pos.y) +
    (dummy_pos.z - pos.z) * (dummy_pos.z - pos.z));
  stat.add(dist);

  return stat;
}

}  // namespace metrics
}  // namespace localization_diagnostics
