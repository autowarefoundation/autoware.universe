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

#ifndef AUTOWARE__PATH_GENERATOR__COMMON_STRUCTS_HPP_
#define AUTOWARE__PATH_GENERATOR__COMMON_STRUCTS_HPP_

#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <string>

namespace autoware::path_generator
{
struct PlannerData
{
  lanelet::LaneletMapPtr lanelet_map_ptr{nullptr};
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr{nullptr};
  lanelet::routing::RoutingGraphPtr routing_graph_ptr{nullptr};

  std::string route_frame_id{};
  geometry_msgs::msg::Pose goal_pose{};

  lanelet::ConstLanelets route_lanelets{};
  lanelet::ConstLanelets preferred_lanelets{};
  lanelet::ConstLanelets start_lanelets{};
  lanelet::ConstLanelets goal_lanelets{};
};
}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__COMMON_STRUCTS_HPP_
