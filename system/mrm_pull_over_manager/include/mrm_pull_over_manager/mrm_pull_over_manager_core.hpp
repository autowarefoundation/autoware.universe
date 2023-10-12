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
#include <memory>
#include <string>

// Autoware
#include <route_handler/route_handler.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

// lanelet
#include <lanelet2_extension/regulatory_elements/Forward.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

class MrmPullOverManager : public rclcpp::Node
{
public:
  MrmPullOverManager();

private:
  using Odometry = nav_msgs::msg::Odometry;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;

  // Subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  Odometry::ConstSharedPtr odom_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  lanelet::ConstLanelets shoulder_lanelets_;
  route_handler::RouteHandler route_handler_;

  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onMap(const HADMapBin::ConstSharedPtr msg);

  // Publisher
  // rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
  //   pub_control_command_;

  // void publishMrmState();

  // Clients
  // rclcpp::CallbackGroup::SharedPtr client_mrm_comfortable_stop_group_;
  // rclcpp::Client<tier4_system_msgs::srv::OperateMrm>::SharedPtr client_mrm_comfortable_stop_;

  // Timer
  // rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  // Param param_;

  bool isDataReady();

  // Algorithm
};

#endif  // MRM_PULL_OVER_MANAGER__MRM_PULL_OVER_MANAGER_CORE_HPP_
