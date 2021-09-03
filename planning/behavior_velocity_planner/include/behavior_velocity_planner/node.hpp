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

#ifndef BEHAVIOR_VELOCITY_PLANNER__NODE_HPP_
#define BEHAVIOR_VELOCITY_PLANNER__NODE_HPP_

#include <memory>
#include <string>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_api_msgs/msg/crosswalk_status.hpp"
#include "autoware_api_msgs/msg/intersection_status.hpp"
#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "behavior_velocity_planner/planner_data.hpp"
#include "behavior_velocity_planner/planner_manager.hpp"

namespace behavior_velocity_planner
{
class BehaviorVelocityPlannerNode : public rclcpp::Node
{
public:
  explicit BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & node_options);

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
  rclcpp::Subscription<autoware_api_msgs::msg::CrosswalkStatus>::SharedPtr
    sub_external_crosswalk_states_;
  rclcpp::Subscription<autoware_api_msgs::msg::IntersectionStatus>::SharedPtr
    sub_external_intersection_states_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficLightStateArray>::SharedPtr
    sub_external_traffic_light_states_;
  rclcpp::Subscription<autoware_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr
    sub_virtual_traffic_light_states_;

  void onTrigger(const autoware_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg);
  void onDynamicObjects(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr msg);
  void onNoGroundPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onVehicleVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void onLaneletMap(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg);
  void onTrafficLightStates(
    const autoware_perception_msgs::msg::TrafficLightStateArray::ConstSharedPtr msg);
  void onExternalTrafficLightStates(
    const autoware_perception_msgs::msg::TrafficLightStateArray::ConstSharedPtr msg);
  void onExternalCrosswalkStates(const autoware_api_msgs::msg::CrosswalkStatus::ConstSharedPtr msg);
  void onExternalIntersectionStates(
    const autoware_api_msgs::msg::IntersectionStatus::ConstSharedPtr msg);
  void onVirtualTrafficLightStates(
    const autoware_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr msg);

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
}  // namespace behavior_velocity_planner

#endif  // BEHAVIOR_VELOCITY_PLANNER__NODE_HPP_
