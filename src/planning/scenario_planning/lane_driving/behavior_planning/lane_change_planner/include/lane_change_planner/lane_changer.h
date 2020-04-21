/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef LANE_CHANGE_PLANNER_LANE_CHANGER_H
#define LANE_CHANGE_PLANNER_LANE_CHANGER_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/Route.h>

#include <lane_change_planner/data_manager.h>
#include <lane_change_planner/route_handler.h>
#include <lane_change_planner/state_machine.h>

// lanelet
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>

namespace lane_change_planner
{
class LaneChanger
{
private:
  ros::Timer timer_;

  ros::Publisher path_publisher_;
  ros::Publisher path_marker_publisher_;
  ros::Publisher drivable_area_publisher_;
  ros::Publisher lane_change_ready_publisher_;
  ros::Publisher lane_change_available_publisher_;

  ros::NodeHandle pnh_;

  ros::Subscriber points_subscriber_;
  ros::Subscriber perception_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber lane_change_approval_subscriber_;
  ros::Subscriber force_lane_change_subscriber_;

  ros::Subscriber vector_map_subscriber_;
  ros::Subscriber route_subscriber_;
  ros::Subscriber route_init_subscriber_;

  std::shared_ptr<DataManager> data_manager_ptr_;
  std::shared_ptr<StateMachine> state_machine_ptr_;
  std::shared_ptr<RouteHandler> route_handler_ptr_;
  // PathExtender path_extender_;

  void run(const ros::TimerEvent & event);
  void publishDebugMarkers();
  void publishDrivableArea(const autoware_planning_msgs::PathWithLaneId & path);
  void waitForData();

public:
  LaneChanger();
  void init();
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_LANE_CHANGER_H
