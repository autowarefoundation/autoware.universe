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

#include <memory>
#include <string>

#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "behavior_velocity_planner/planner_data.hpp"
#include "behavior_velocity_planner/planner_manager.hpp"

class BehaviorVelocityPlannerNode : public rclcpp::Node
{
public:
  BehaviorVelocityPlannerNode();

private:
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // subscriber
  rclcpp::Subscription<autoware_planning_msgs::msg::PathWithLaneId>::SharedPtr
    trigger_sub_path_with_lane_id_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    sub_dynamic_objects_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_no_ground_pointcloud_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vehicle_velocity_;
  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr sub_lanelet_map_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficLightStateArray>::SharedPtr
    sub_traffic_light_states_;

  void onTrigger(const autoware_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg);
  void onDynamicObjects(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr msg);
  void onNoGroundPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onVehicleVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void onLaneletMap(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg);
  void onTrafficLightStates(
    const autoware_perception_msgs::msg::TrafficLightStateArray::ConstSharedPtr msg);

  // publisher
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stop_reason_diag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;

  void publishDebugMarker(const autoware_planning_msgs::msg::Path & path);

  //  parameter
  double forward_path_length_;
  double backward_path_length_;

  // member
  PlannerData planner_data_;
  BehaviorVelocityPlannerManager planner_manager_;

  // function
  geometry_msgs::msg::PoseStamped getCurrentPose();
  bool isDataReady();
};
