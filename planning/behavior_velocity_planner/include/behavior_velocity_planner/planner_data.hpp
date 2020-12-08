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

#pragma once

#include <map>
#include <memory>

#include "tf2_ros/transform_listener.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_perception_msgs/msg/traffic_light_state_array.hpp"
#include "autoware_perception_msgs/msg/traffic_light_state_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "vehicle_info_util/vehicle_info.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/RoutingGraphContainer.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

struct PlannerData
{
  PlannerData(rclcpp::Node & node)
  : vehicle_info_(vehicle_info_util::VehicleInfo::create(node))
  {
    max_stop_acceleration_threshold_ = node.declare_parameter(
      "max_accel", -5.0);  // TODO read min_acc in velocity_controller_param.yaml?
    delay_response_time_ = node.declare_parameter("delay_response_time", 1.3);
  }
  // tf
  geometry_msgs::msg::PoseStamped current_pose;

  // msgs from callbacks that are used for data-ready
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity;
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr dynamic_objects;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_ground_pointcloud;
  lanelet::LaneletMapPtr lanelet_map;

  // other internal data
  std::map<int, autoware_perception_msgs::msg::TrafficLightStateStamped> traffic_light_id_map_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
  lanelet::routing::RoutingGraphPtr routing_graph;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs;

  // parameters
  vehicle_info_util::VehicleInfo vehicle_info_;

  // additional parameters
  double max_stop_acceleration_threshold_;
  double delay_response_time_;

  bool isVehicleStopping() const
  {
    if (!current_velocity) {return false;}
    return current_velocity->twist.linear.x < 0.1;
  }

  std::shared_ptr<autoware_perception_msgs::msg::TrafficLightStateStamped> getTrafficLightState(
    const int id) const
  {
    if (traffic_light_id_map_.count(id) == 0) {
      return {};
    }
    return std::make_shared<autoware_perception_msgs::msg::TrafficLightStateStamped>(
      traffic_light_id_map_.at(id));
  }
};
