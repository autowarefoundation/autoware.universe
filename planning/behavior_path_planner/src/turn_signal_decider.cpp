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

#include "behavior_path_planner/turn_signal_decider.hpp"

#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/resample/resample.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <limits>
#include <string>
#include <utility>

namespace behavior_path_planner
{
TurnIndicatorsCommand TurnSignalDecider::getTurnSignal(
  const PathWithLaneId & path, const Pose & current_pose, const double current_vel,
  const size_t current_seg_idx, const RouteHandler & route_handler,
  const TurnSignalInfo & turn_signal_info) const
{
  // Guard
  if (path.points.empty()) {
    return turn_signal_info.turn_signal;
  }

  // Get closest intersection turn signal if exists
  const auto intersection_turn_signal_info =
    getIntersectionTurnSignalInfo(path, current_pose, current_vel, current_seg_idx, route_handler);

  /* Resolve the conflict between turn signal info using sections
  if (
    intersection_distance < plan_distance ||
    turn_signal_plan.command == TurnIndicatorsCommand::NO_COMMAND ||
    turn_signal_plan.command == TurnIndicatorsCommand::DISABLE) {
    return intersection_turn_signal.turn_signal;
  }
  */

  return turn_signal_info.turn_signal;
}

boost::optional<TurnSignalInfo> TurnSignalDecider::getIntersectionTurnSignalInfo(
  const PathWithLaneId & path, const Pose & current_pose, const double current_vel,
  const size_t current_seg_idx, const RouteHandler & route_handler) const
{
  if (path.points.size() < 2) {
    return {};
  }

  TurnSignalInfo turn_signal_info{};
  turn_signal_info.turn_signal.command = TurnIndicatorsCommand::DISABLE;

  // search distance
  const double search_distance = 3.0 * current_vel + intersection_search_distance_;

  // front pose
  const auto vehicle_front_pose =
    tier4_autoware_utils::calcOffsetPose(current_pose, base_link2front_, 0.0, 0.0);

  // Get nearest intersection and decide turn signal
  auto lane_attribute = std::string("none");
  for (size_t i = 0; i < path.points.size(); ++i) {
    const double distance_from_vehicle_front =
      motion_utils::calcSignedArcLength(path.points, current_pose.position, current_seg_idx, i) -
      base_link2front_;

    if (
      search_distance < distance_from_vehicle_front &&
      turn_signal_info.turn_signal.command == TurnIndicatorsCommand::DISABLE) {
      // No intersection ahead of search_distance
      return {};
    }

    // TODO(Horibe): Route Handler should be a library.
    const auto lanelets = route_handler.getLaneletsFromIds(path.points.at(i).lane_ids);
    for (const auto & lane : lanelets) {
      // judgement of lighting of turn_signal
      const bool cond1 =
        lane.attributeOr("turn_direction", std::string("none")) != lane_attribute &&
        distance_from_vehicle_front < lane.attributeOr("turn_signal_distance", search_distance);
      const bool cond2 = lane.hasAttribute("turn_direction") && i == current_seg_idx + 1;

      // update lane attribute
      lane_attribute = lane.attributeOr("turn_direction", std::string("none"));

      // lane front and back point
      const geometry_msgs::msg::Point lane_front_point =
        lanelet::utils::conversion::toGeomMsgPt(lane.centerline3d().front());
      const geometry_msgs::msg::Point lane_back_point =
        lanelet::utils::conversion::toGeomMsgPt(lane.centerline3d().back());

      if (distance_from_vehicle_front > 0.0 && cond1) {
        // lanelet with turn direction is within the search distance
        const size_t nearest_seg_idx =
          motion_utils::findNearestSegmentIndex(path.points, lane_front_point);
        const double dist_to_lane_front = motion_utils::calcSignedArcLength(
                                            path.points, current_pose.position, current_seg_idx,
                                            lane_front_point, nearest_seg_idx) -
                                          base_link2front_;

        turn_signal_info.desired_start_point =
          dist_to_lane_front < 0.0 ? lane_front_point : vehicle_front_pose.position;
        turn_signal_info.required_start_point = lane_front_point;
        turn_signal_info.required_end_point = get_required_end_point(lane.centerline3d());
        turn_signal_info.desired_end_point = lane_back_point;

        turn_signal_info.turn_signal.command = signal_map.at(lane_attribute);
      } else if (distance_from_vehicle_front > 0.0 && cond2) {
        // Vehicle is inside the turing lanelet
        turn_signal_info.desired_start_point = lane_front_point;
        turn_signal_info.required_start_point = lane_front_point;
        turn_signal_info.required_end_point = get_required_end_point(lane.centerline3d());
        turn_signal_info.desired_end_point = lane_back_point;

        turn_signal_info.turn_signal.command = signal_map.at(lane_attribute);
      }
    }
  }

  if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::DISABLE) {
    return {};
  }

  return turn_signal_info;
}

geometry_msgs::msg::Point TurnSignalDecider::get_required_end_point(
  const lanelet::ConstLineString3d & centerline) const
{
  std::vector<geometry_msgs::msg::Pose> converted_centerline(centerline.size());
  for (size_t i = 0; i < centerline.size(); ++i) {
    converted_centerline.at(i).position = lanelet::utils::conversion::toGeomMsgPt(centerline[i]);
  }
  motion_utils::insertOrientation(converted_centerline, true);

  const double length = motion_utils::calcArcLength(converted_centerline);

  // Create resampling intervals
  const double resampling_interval = 1.0;
  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < length; s += resampling_interval) {
    resampling_arclength.push_back(s);
  }

  // Insert terminal point
  if (length - resampling_arclength.back() < motion_utils::overlap_threshold) {
    resampling_arclength.back() = length;
  } else {
    resampling_arclength.push_back(length);
  }

  const auto resampled_centerline =
    motion_utils::resamplePath(converted_centerline, resampling_arclength);

  const double terminal_yaw = tf2::getYaw(resampled_centerline.back().orientation);
  for (size_t i = 0; i < resampled_centerline.size(); ++i) {
    const double yaw = tf2::getYaw(resampled_centerline.at(i).orientation);
    const double yaw_diff = tier4_autoware_utils::normalizeRadian(yaw - terminal_yaw);
    if (std::fabs(yaw_diff) < tier4_autoware_utils::deg2rad(15)) {
      return resampled_centerline.at(i).position;
    }
  }

  return resampled_centerline.back().position;
}
}  // namespace behavior_path_planner
