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

#pragma once

#include <map>
#include <memory>

#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_perception_msgs/TrafficLightStateArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// TODO(Kenji Miyake): Add msg
struct TrafficLightStateStamped
{
  std_msgs::Header header;
  autoware_perception_msgs::TrafficLightState traffic_light_state;
};

struct PlannerData
{
  // tf
  geometry_msgs::PoseStamped current_pose;

  // msgs from callbacks that are used for data-ready
  geometry_msgs::TwistStamped::ConstPtr current_velocity;
  autoware_perception_msgs::DynamicObjectArray::ConstPtr dynamic_objects;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_ground_pointcloud;
  lanelet::LaneletMapPtr lanelet_map;

  // other internal data
  std::map<int, TrafficLightStateStamped> traffic_light_id_map_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
  lanelet::routing::RoutingGraphPtr routing_graph;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs;

  // parameters
  double wheel_base;
  double front_overhang;
  double vehicle_width;
  double base_link2front;

  // additional parameters
  double max_stop_acceleration_threshold_;

  bool isVehicleStopping() const
  {
    if (!current_velocity) return false;
    return current_velocity->twist.linear.x < 0.1;
  }

  std::shared_ptr<TrafficLightStateStamped> getTrafficLightState(const int id) const
  {
    if (traffic_light_id_map_.count(id) == 0) {
      return {};
    }
    return std::make_shared<TrafficLightStateStamped>(traffic_light_id_map_.at(id));
  }
};
