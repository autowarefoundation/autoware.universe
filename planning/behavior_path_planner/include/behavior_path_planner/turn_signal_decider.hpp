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

#ifndef BEHAVIOR_PATH_PLANNER__TURN_SIGNAL_DECIDER_HPP_
#define BEHAVIOR_PATH_PLANNER__TURN_SIGNAL_DECIDER_HPP_

#include <route_handler/route_handler.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <map>
#include <memory>
#include <string>
#include <utility>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::Pose;
using route_handler::RouteHandler;

struct TurnSignalInfo
{
  TurnSignalInfo()
  {
    turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
    hazard_signal.command = HazardLightsCommand::NO_COMMAND;
  }

  // desired turn signal
  TurnIndicatorsCommand turn_signal;
  HazardLightsCommand hazard_signal;

  geometry_msgs::msg::Point desired_start_point;
  geometry_msgs::msg::Point desired_end_point;
  geometry_msgs::msg::Point required_start_point;
  geometry_msgs::msg::Point required_end_point;
};

const std::map<std::string, uint8_t> signal_map = {
  {"left", TurnIndicatorsCommand::ENABLE_LEFT},
  {"right", TurnIndicatorsCommand::ENABLE_RIGHT},
  {"none", TurnIndicatorsCommand::DISABLE}};

class TurnSignalDecider
{
public:
  TurnIndicatorsCommand getTurnSignal(
    const PathWithLaneId & path, const Pose & current_pose, const double current_vel,
    const size_t current_seg_idx, const RouteHandler & route_handler,
    const TurnSignalInfo & turn_signal_info) const;

  void setParameters(const double base_link2front, const double intersection_search_distance)
  {
    base_link2front_ = base_link2front;
    intersection_search_distance_ = intersection_search_distance;
  }

private:
  boost::optional<TurnSignalInfo> getIntersectionTurnSignalInfo(
    const PathWithLaneId & path, const Pose & current_pose, const double current_vel,
    const size_t current_seg_idx, const RouteHandler & route_handler) const;

  geometry_msgs::msg::Point get_required_end_point(
    const lanelet::ConstLineString3d & centerline) const;

  rclcpp::Logger logger_{
    rclcpp::get_logger("behavior_path_planner").get_child("turn_signal_decider")};

  // data
  double intersection_search_distance_{0.0};
  double base_link2front_{0.0};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__TURN_SIGNAL_DECIDER_HPP_
