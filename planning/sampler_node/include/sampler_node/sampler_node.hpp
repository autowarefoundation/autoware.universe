// Copyright 2022 Tier IV, Inc.
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

#ifndef FRENET_PLANNER__SAMPLER_NODE_HPP_
#define FRENET_PLANNER__SAMPLER_NODE_HPP_

#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/structures.hpp"
#include "sampler_node/plot/debug_window.hpp"

#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>

#include <autoware_auto_perception_msgs/msg/detail/predicted_objects__struct.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <qapplication.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace sampler_node
{
class FrenetPlannerNode : public rclcpp::Node
{
private:
  // Debug visualization
  int argc_ = 1;
  std::vector<char *> argv_ = {std::string("Frenet Debug Visualization").data()};
  QApplication qapplication_;
  std::unique_ptr<QApplication> qt_app_;
  std::unique_ptr<plot::MainWindow> qt_window_;
  plot::MainWindow w_;

  // Parameters

  // Cached data
  std::unique_ptr<geometry_msgs::msg::TwistStamped> current_twist_ptr_ =
    std::make_unique<geometry_msgs::msg::TwistStamped>();
  std::unique_ptr<geometry_msgs::msg::Pose> prev_ego_pose_ptr_;
  std::unique_ptr<autoware_auto_perception_msgs::msg::PredictedObjects> in_objects_ptr_;
  frenet_planner::Trajectory prev_trajectory_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<rclcpp::Time> prev_replanned_time_ptr_;

  // ROS pub / sub
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    objects_sub_;

  rclcpp::TimerBase::SharedPtr gui_process_timer_;

  // callback functions
  void pathCallback(const autoware_auto_planning_msgs::msg::Path::SharedPtr);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr);
  void objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr);

  // other functions
  void publishTrajectory(
    const frenet_planner::Trajectory & trajectory,
    const autoware_auto_planning_msgs::msg::Path::SharedPtr path_msg);
  std::optional<sampler_common::Point> getCurrentEgoPose();
  static std::vector<frenet_planner::Trajectory> generateFrenetTrajectories(
    const frenet_planner::FrenetState & initial_state,
    const frenet_planner::Trajectory & base_trajectory,
    const autoware_auto_planning_msgs::msg::Path & path_msg,
    const sampler_common::transform::Spline2D & path_spline,
    const sampler_common::Constraints & constraints, frenet_planner::Debug & debug);
  static std::optional<frenet_planner::Trajectory> selectBestTrajectory(
    const std::vector<frenet_planner::Trajectory> & trajectories);

public:
  explicit FrenetPlannerNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace sampler_node

#endif  // FRENET_PLANNER__SAMPLER_NODE_HPP_
