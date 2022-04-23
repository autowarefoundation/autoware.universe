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

#ifndef MISSION_PLANNER__MISSION_PLANNER_BASE_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_BASE_HPP_

#include <boost/optional.hpp>

#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>

#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

namespace mission_planner
{
class MissionPlanner : public rclcpp::Node
{
protected:
  MissionPlanner(const std::string & node_name, const rclcpp::NodeOptions & node_options);

  tier4_autoware_utils::SelfPoseListener self_pose_listener_{this};

  std::string map_frame_;

  boost::optional<geometry_msgs::msg::PoseStamped> goal_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> checkpoints_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  virtual bool isRoutingGraphReady() const = 0;
  virtual autoware_auto_planning_msgs::msg::HADMapRoute planRoute(
    const std::vector<geometry_msgs::msg::PoseStamped> & pass_points,
    const bool is_looped_route = false) = 0;
  virtual boost::optional<size_t> getClosestRouteSectionIndex(
    const autoware_auto_planning_msgs::msg::HADMapRoute & route,
    const geometry_msgs::msg::PoseStamped & pose, geometry_msgs::msg::Pose & goal_pose) = 0;

  virtual void visualizeRoute(
    const autoware_auto_planning_msgs::msg::HADMapRoute & route) const = 0;
  virtual void publishRoute(const autoware_auto_planning_msgs::msg::HADMapRoute & route) const;

private:
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::HADMapRoute>::SharedPtr route_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr checkpoint_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loop_subscriber_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool is_looped_route_{false};

  autoware_auto_planning_msgs::msg::HADMapRoute route_;
  boost::optional<size_t> loop_idx_;

  rclcpp::TimerBase::SharedPtr timer_;

  boost::optional<geometry_msgs::msg::PoseStamped> transformPose(
    const geometry_msgs::msg::PoseStamped & input_pose, const std::string & target_frame);

  void run();
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_msg_ptr);
  void checkpointCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr checkpoint_msg_ptr);
  void loopCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
};

}  // namespace mission_planner
#endif  // MISSION_PLANNER__MISSION_PLANNER_BASE_HPP_
