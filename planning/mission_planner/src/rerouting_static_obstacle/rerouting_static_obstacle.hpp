// Copyright 2023 TIER IV, Inc.
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

#ifndef REROUTING_STATIC_OBSTACLE__REROUTING_STATIC_OBSTACLE_HPP_
#define REROUTING_STATIC_OBSTACLE__REROUTING_STATIC_OBSTACLE_HPP_

#include <component_interface_specs/planning.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_routing/RoutingGraph.h>

#include <vector>

namespace mission_planner
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
class ReroutingStaticObstacle : public rclcpp::Node
{
  using ChangeRoute = planning_interface::ChangeRoute;

public:
  explicit ReroutingStaticObstacle(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_trigger_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<HADMapBin>::SharedPtr map_subscriber_;
  component_interface_utils::Client<ChangeRoute>::SharedPtr cli_change_route_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  route_handler::RouteHandler route_handler_;
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void route_callback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg);
  void map_callback(const HADMapBin::ConstSharedPtr msg);
  void on_trigger(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  bool get_selected_point_lanelets(
    const geometry_msgs::msg::Point & selected_point,
    lanelet::ConstLanelets & selected_point_lanelets) const;
  bool get_remaining_route_lanelets(lanelet::ConstLanelets & remaining_route_lanelets) const;
  bool is_selected_point_in_route(
    const lanelet::ConstLanelets & selected_point_lanelets,
    const lanelet::ConstLanelets & remaining_route_lanelets,
    lanelet::ConstLanelet & selected_point_lanelet) const;
  bool search_alternative_route(
    const lanelet::ConstLanelet & selected_point_lanelet,
    lanelet::routing::LaneletPath & alternative_route_lanelets) const;

  void change_route(const lanelet::routing::LaneletPath & lanelet_path);

  void convert_lanelet_path_to_route_segments(
    const lanelet::routing::LaneletPath & lanelet_path,
    std::vector<autoware_adapi_v1_msgs::msg::RouteSegment> & route_segments) const;
};

}  // namespace mission_planner
#endif  // REROUTING_STATIC_OBSTACLE__REROUTING_STATIC_OBSTACLE_HPP_
