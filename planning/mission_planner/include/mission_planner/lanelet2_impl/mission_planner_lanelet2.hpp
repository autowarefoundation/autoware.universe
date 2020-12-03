// Copyright 2019 Autoware Foundation
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

#ifndef MISSION_PLANNER_LANELET2_IMPL_MISSION_PLANNER_LANELET2_H
#define MISSION_PLANNER_LANELET2_IMPL_MISSION_PLANNER_LANELET2_H

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <mission_planner/lanelet2_impl/route_handler.hpp>
#include <mission_planner/mission_planner_base.hpp>

// lanelet
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// others
#include <string>

using RouteSections = std::vector<autoware_planning_msgs::msg::RouteSection>;

namespace mission_planner
{
class MissionPlannerLanelet2 : public MissionPlanner
{
public:
  MissionPlannerLanelet2();

private:
  bool is_graph_ready_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;

  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr map_subscriber_;

  void mapCallback(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg);
  bool isGoalValid() const;

  // virtual functions
  bool isRoutingGraphReady() const;
  autoware_planning_msgs::msg::Route planRoute();
  void visualizeRoute(const autoware_planning_msgs::msg::Route & route) const;

  // routing
  bool planPathBetweenCheckpoints(
    const geometry_msgs::msg::PoseStamped & start_checkpoint,
    const geometry_msgs::msg::PoseStamped & goal_checkpoint,
    lanelet::ConstLanelets * path_lanelets_ptr) const;
  lanelet::ConstLanelets getMainLanelets(
    const lanelet::ConstLanelets & path_lanelets, const RouteHandler & lanelet_sequence_finder);
  RouteSections createRouteSections(
    const lanelet::ConstLanelets & main_path, const RouteHandler & route_handler);
};
}  // namespace mission_planner

#endif  // MISSION_PLANNER_LANELET2_IMPL_MISSION_PLANNER_LANELET2_H
