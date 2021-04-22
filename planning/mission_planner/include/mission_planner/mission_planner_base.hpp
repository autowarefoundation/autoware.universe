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

#ifndef MISSION_PLANNER_MISSION_PLANNER_BASE_H
#define MISSION_PLANNER_MISSION_PLANNER_BASE_H

// ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

// Autoware
#include "autoware_planning_msgs/msg/route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// others
#include <string>
#include <vector>

namespace mission_planner
{
class MissionPlanner : public rclcpp::Node
{
protected:
  MissionPlanner(const std::string & node_name, const rclcpp::NodeOptions & node_options);

  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped start_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> checkpoints_;

  std::string base_link_frame_;
  std::string map_frame_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  virtual bool isRoutingGraphReady() const = 0;
  virtual autoware_planning_msgs::msg::Route planRoute() = 0;
  virtual void visualizeRoute(const autoware_planning_msgs::msg::Route & route) const = 0;
  virtual void publishRoute(const autoware_planning_msgs::msg::Route & route) const;

private:
  rclcpp::Publisher<autoware_planning_msgs::msg::Route>::SharedPtr route_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr checkpoint_subscriber_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool getEgoVehiclePose(geometry_msgs::msg::PoseStamped * ego_vehicle_pose);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_msg_ptr);
  void checkpointCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr checkpoint_msg_ptr);
  bool transformPose(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped * output_pose, const std::string target_frame);
};

}  // namespace mission_planner
#endif  // MISSION_PLANNER_MISSION_PLANNER_BASE_H
