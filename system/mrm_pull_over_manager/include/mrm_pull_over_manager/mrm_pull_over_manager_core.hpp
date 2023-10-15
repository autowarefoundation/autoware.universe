// Copyright 2023 Tier IV, Inc.
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

#ifndef MRM_PULL_OVER_MANAGER__MRM_PULL_OVER_MANAGER_CORE_HPP_
#define MRM_PULL_OVER_MANAGER__MRM_PULL_OVER_MANAGER_CORE_HPP_

// Core
#include <map>
#include <memory>
#include <vector>

// Autoware
#include <route_handler/route_handler.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

// lanelet
#include <lanelet2_extension/regulatory_elements/Forward.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
namespace mrm_pull_over_manager
{
class MrmPullOverManager : public rclcpp::Node
{
public:
  MrmPullOverManager();

private:
  using Odometry = nav_msgs::msg::Odometry;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using PoseLaneIdMap = std::map<lanelet::Id, geometry_msgs::msg::Pose>;

  // Subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  Odometry::ConstSharedPtr odom_;
  route_handler::RouteHandler route_handler_;

  void on_odometry(const Odometry::ConstSharedPtr msg);
  void on_route(const LaneletRoute::ConstSharedPtr msg);
  void on_map(const HADMapBin::ConstSharedPtr msg);

  // Publisher

  // Clients

  // TODO: temporary for debug
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void on_timer();

  // Parameters
  // Param param_;

  PoseLaneIdMap candidate_goals_;

  // Algorithm
  bool is_data_ready();
  std::vector<geometry_msgs::msg::Pose> find_near_goals();
  lanelet::ConstLanelet get_current_lanelet();
  lanelet::ConstLanelets get_all_following_and_left_lanelets(
    const lanelet::ConstLanelet & start_lanelet) const;
  lanelet::ConstLanelets get_all_left_lanelets(const lanelet::ConstLanelet & base_lanelet) const;
  std::vector<geometry_msgs::msg::Pose> find_goals_in_lanelets(
    const lanelet::ConstLanelets & candidate_lanelets) const;
};
}  // namespace mrm_pull_over_manager

#endif  // MRM_PULL_OVER_MANAGER__MRM_PULL_OVER_MANAGER_CORE_HPP_
