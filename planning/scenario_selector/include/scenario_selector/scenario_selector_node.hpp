// Copyright 2020 Tier IV, Inc.
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

#ifndef SCENARIO_SELECTOR__SCENARIO_SELECTOR_NODE_HPP_
#define SCENARIO_SELECTOR__SCENARIO_SELECTOR_NODE_HPP_

#include <deque>
#include <memory>
#include <string>

#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_planning_msgs/msg/route.hpp"
#include "autoware_planning_msgs/msg/scenario.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRules.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct Input
{
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr buf_trajectory;
};

struct Output
{
  rclcpp::Publisher<autoware_planning_msgs::msg::Scenario>::SharedPtr pub_scenario;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory;
};

class ScenarioSelectorNode : public rclcpp::Node
{
public:
  explicit ScenarioSelectorNode(const rclcpp::NodeOptions & node_options);

  void onMap(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg);
  void onRoute(const autoware_planning_msgs::msg::Route::ConstSharedPtr msg);
  void onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

  void onTimer();

  autoware_planning_msgs::msg::Scenario selectScenario();
  std::string selectScenarioByPosition();
  Input getScenarioInput(const std::string & scenario);

private:
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr sub_lanelet_map_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Route>::SharedPtr sub_route_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;

  Input input_lane_driving_;
  Input input_parking_;

  Output output_;

  autoware_planning_msgs::msg::Route::ConstSharedPtr route_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;

  std::string current_scenario_;
  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_buffer_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  // Parameters
  double update_rate_;
  double th_max_message_delay_sec_;
  double th_arrived_distance_m_;
  double th_stopped_time_sec_;
  double th_stopped_velocity_mps_;
};

#endif  // SCENARIO_SELECTOR__SCENARIO_SELECTOR_NODE_HPP_
