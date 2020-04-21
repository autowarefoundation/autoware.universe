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

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <autoware_vehicle_msgs/TurnSignal.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace turn_signal_decider
{
class DataManager
{
private:
  // subscribe validation boolean
  bool is_map_ready_;
  bool is_path_ready_;
  bool is_pose_ready_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // subscribed variables
  autoware_planning_msgs::PathWithLaneId path_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets path_lanes_;
  geometry_msgs::PoseStamped vehicle_pose_;

  // condition checks
  bool isPathValid() const;
  bool isPoseValid() const;

public:
  DataManager();

  // callbacks
  void onPathWithLaneId(const autoware_planning_msgs::PathWithLaneId & msg);
  void onLaneletMap(const autoware_lanelet2_msgs::MapBin & map_msg);
  void onVehiclePoseUpdate(const ros::TimerEvent & event);

  // getters
  autoware_planning_msgs::PathWithLaneId getPath() const;
  lanelet::LaneletMapPtr getMapPtr() const;
  lanelet::ConstLanelet getLaneFromId(const lanelet::Id & id) const;
  lanelet::routing::RoutingGraphPtr getRoutingGraphPtr() const;
  geometry_msgs::PoseStamped getVehiclePoseStamped() const;

  // condition checks
  bool isDataReady() const;
};
}  // namespace turn_signal_decider