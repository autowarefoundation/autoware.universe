// Copyright 2024 TIER IV, Inc.
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

#include "behavior_path_lane_change_module/utils/calculation.hpp"

#include "behavior_path_planner_common/utils/utils.hpp"

namespace behavior_path_planner::utils::lane_change::calculation
{
static rclcpp::Logger logger{rclcpp::get_logger("lane_change.utils")};

double calc_min_lane_change_length(
  const LCParamPtr & lc_params, const std::vector<double> & shift_intervals)
{
  if (shift_intervals.empty()) {
    return 0.0;
  }

  const auto min_vel = lc_params->minimum_lane_changing_velocity;
  const auto lat_acc = lc_params->lane_change_lat_acc_map.find(min_vel);
  const auto max_lat_acc = lat_acc.second;
  const auto lat_jerk = lc_params->lane_changing_lateral_jerk;
  const auto finish_judge_buffer = lc_params->lane_change_finish_judge_buffer;

  const auto calc_sum = [&](double sum, double shift_interval) {
    const auto t = PathShifter::calcShiftTimeFromJerk(shift_interval, lat_jerk, max_lat_acc);
    return sum + (min_vel * t + finish_judge_buffer);
  };

  const auto total_length =
    std::accumulate(shift_intervals.begin(), shift_intervals.end(), 0.0, calc_sum);

  const auto backward_buffer = lc_params->backward_length_buffer_for_end_of_lane;
  return total_length + backward_buffer * (static_cast<double>(shift_intervals.size()) - 1.0);
}

double calc_min_lane_change_length(
  const CommonDataPtr & common_data, const lanelet::ConstLanelets & lanes, Direction direction)
{
  if (lanes.empty()) {
    RCLCPP_DEBUG(logger, "Because the lanes are empty, 0.0 meter is returned.");
    return 0.0;
  }

  const auto & route_handler = common_data->route_handler;
  if (!route_handler) {
    RCLCPP_DEBUG(logger, "Because route hander pointer is null, 0.0 is returned.");
    return 0.0;
  }

  const auto shift_intervals =
    route_handler->getLateralIntervalsToPreferredLane(lanes.back(), direction);

  const auto & lc_params = common_data->lc_params;
  return calc_min_lane_change_length(lc_params, shift_intervals);
}

double calc_ego_remaining_distance_in_current_lanes(const CommonDataPtr & common_data)
{
  const auto & current_lanes = common_data->lanes.current;
  if (current_lanes.empty()) {
    RCLCPP_DEBUG(logger, "Because the lanes are empty, 0.0 is returned.");
    return 0.0;
  }

  const auto & route_handler = common_data->route_handler;
  if (route_handler->isInGoalRouteSection(current_lanes)) {
    const auto & goal_pose = route_handler->getGoalPose();
    return utils::getSignedDistance(common_data->get_ego_pose(), goal_pose, current_lanes);
  }

  return utils::getDistanceToEndOfLane(common_data->get_ego_pose(), current_lanes);
}
}  // namespace behavior_path_planner::utils::lane_change::calculation
