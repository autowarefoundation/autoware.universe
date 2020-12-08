// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef FREESPACE_PLANNER__FREESPACE_PLANNER_HPP_
#define FREESPACE_PLANNER__FREESPACE_PLANNER_HPP_

#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "astar_search/astar_search.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "autoware_planning_msgs/msg/route.hpp"
#include "autoware_planning_msgs/msg/scenario.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

struct NodeParam
{
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;
  bool replan_when_obstacle_found;
  bool replan_when_course_out;
};

class AstarNavi : public rclcpp::Node
{
public:
  AstarNavi();

private:
  // ros
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr debug_pose_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr debug_partial_pose_array_pub_;

  rclcpp::Subscription<autoware_planning_msgs::msg::Route>::SharedPtr route_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Scenario>::SharedPtr scenario_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;
  AstarParam astar_param_;

  // variables
  std::unique_ptr<AstarSearch> astar_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  autoware_planning_msgs::msg::Trajectory trajectory_;
  autoware_planning_msgs::msg::Trajectory partial_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_;
  size_t target_index_;
  bool is_completed_ = false;

  autoware_planning_msgs::msg::Route::ConstSharedPtr route_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid_;
  autoware_planning_msgs::msg::Scenario::ConstSharedPtr scenario_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;

  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_buffer_;

  // functions, callback
  void onRoute(const autoware_planning_msgs::msg::Route::ConstSharedPtr msg);
  void onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void onScenario(const autoware_planning_msgs::msg::Scenario::ConstSharedPtr msg);
  void onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

  void onTimer();

  void reset();
  bool isPlanRequired();
  void planTrajectory();
  void updateTargetIndex();

  geometry_msgs::msg::TransformStamped getTransform(
    const std::string & from, const std::string & to);
};

#endif  // FREESPACE_PLANNER__FREESPACE_PLANNER_HPP_
