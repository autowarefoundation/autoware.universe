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

#ifndef BEHAVIOR_VELOCITY_PLANNER__PLANNER_DATA_HPP_
#define BEHAVIOR_VELOCITY_PLANNER__PLANNER_DATA_HPP_

#include <boost/optional.hpp>
#include <map>
#include <memory>

#include "tf2_ros/transform_listener.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "autoware_api_msgs/msg/crosswalk_status.hpp"
#include "autoware_api_msgs/msg/intersection_status.hpp"
#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_perception_msgs/msg/traffic_light_state_array.hpp"
#include "autoware_perception_msgs/msg/traffic_light_state_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/RoutingGraphContainer.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

struct PlannerData
{
  explicit PlannerData(rclcpp::Node & node)
  : vehicle_info_(vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo())
  {
    max_stop_acceleration_threshold = node.declare_parameter(
      "max_accel", -5.0);  // TODO(someone): read min_acc in velocity_controller.param.yaml?
    max_stop_jerk_threshold = node.declare_parameter("max_jerk", -5.0);
    delay_response_time = node.declare_parameter("delay_response_time", 0.50);
    yellow_lamp_period = node.declare_parameter("yellow_lamp_period", 2.75);
  }
  // tf
  geometry_msgs::msg::PoseStamped current_pose;

  // msgs from callbacks that are used for data-ready
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr prev_velocity;
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr dynamic_objects;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_ground_pointcloud;
  lanelet::LaneletMapPtr lanelet_map;

  // other internal data
  std::map<int, autoware_perception_msgs::msg::TrafficLightStateStamped> traffic_light_id_map;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
  lanelet::routing::RoutingGraphPtr routing_graph;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs;

  // external data
  std::map<int, autoware_perception_msgs::msg::TrafficLightStateStamped>
    external_traffic_light_id_map;
  boost::optional<autoware_api_msgs::msg::CrosswalkStatus> external_crosswalk_status_input;
  boost::optional<autoware_api_msgs::msg::IntersectionStatus> external_intersection_status_input;

  // parameters
  vehicle_info_util::VehicleInfo vehicle_info_;

  // additional parameters
  double max_stop_acceleration_threshold;
  double max_stop_jerk_threshold;
  double delay_response_time;
  double yellow_lamp_period;

  // calc current acceleration
  double current_accel;
  double prev_accel;
  double accel_lowpass_gain;

  void updateCurrentAcc()
  {
    if (prev_velocity) {
      const double dv = current_velocity->twist.linear.x - prev_velocity->twist.linear.x;
      const auto time_diff =
        rclcpp::Time(current_velocity->header.stamp) - rclcpp::Time(prev_velocity->header.stamp);
      const double dt = std::max(time_diff.seconds(), 1e-03);
      const double accel = dv / dt;
      // apply lowpass filter
      current_accel = accel_lowpass_gain * accel + (1.0 - accel_lowpass_gain) * prev_accel;
    } else {
      current_accel = 0.0;
    }

    prev_velocity = current_velocity;
    prev_accel = current_accel;
  }

  bool isVehicleStopping() const
  {
    if (!current_velocity) {
      return false;
    }
    return current_velocity->twist.linear.x < 0.1;
  }

  std::shared_ptr<autoware_perception_msgs::msg::TrafficLightStateStamped> getTrafficLightState(
    const int id) const
  {
    if (traffic_light_id_map.count(id) == 0) {
      return {};
    }
    return std::make_shared<autoware_perception_msgs::msg::TrafficLightStateStamped>(
      traffic_light_id_map.at(id));
  }

  std::shared_ptr<autoware_perception_msgs::msg::TrafficLightStateStamped>
  getExternalTrafficLightState(const int id) const
  {
    if (external_traffic_light_id_map.count(id) == 0) {
      return {};
    }
    return std::make_shared<autoware_perception_msgs::msg::TrafficLightStateStamped>(
      external_traffic_light_id_map.at(id));
  }
};
#endif  // BEHAVIOR_VELOCITY_PLANNER__PLANNER_DATA_HPP_
