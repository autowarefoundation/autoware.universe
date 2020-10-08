/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <deque>
#include <memory>
#include <string>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Scenario.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/TwistStamped.h>

struct Input
{
  ros::Subscriber sub_trajectory;
  autoware_planning_msgs::Trajectory::ConstPtr buf_trajectory;
};

struct Output
{
  ros::Publisher pub_scenario;
  ros::Publisher pub_trajectory;
};

class ScenarioSelectorNode
{
public:
  ScenarioSelectorNode();

  void onMap(const autoware_lanelet2_msgs::MapBin & msg);
  void onRoute(const autoware_planning_msgs::Route::ConstPtr & msg);
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg);

  void onTimer(const ros::TimerEvent & event);

  autoware_planning_msgs::Scenario selectScenario();
  std::string selectScenarioByPosition();
  Input getScenarioInput(const std::string & scenario);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Timer timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_lanelet_map_;
  ros::Subscriber sub_route_;
  ros::Subscriber sub_twist_;

  Input input_lane_driving_;
  Input input_parking_;

  Output output_;

  autoware_planning_msgs::Route::ConstPtr route_;
  geometry_msgs::PoseStamped::ConstPtr current_pose_;
  geometry_msgs::TwistStamped::ConstPtr twist_;

  std::string current_scenario_;
  std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_buffer_;

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
