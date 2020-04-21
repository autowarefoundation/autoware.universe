/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <memory>
#include <vector>

#include <boost/optional.hpp>  // To be replaced by std::optional in C++17

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "pure_pursuit/pure_pursuit.h"

struct Param
{
  // Global Parameters
  double wheel_base;

  // Node Parameters
  double ctrl_period;

  // Algorithm Parameters
  double lookahead_distance_ratio;
  double min_lookahead_distance;
  double reverse_min_lookahead_distance;  // min_lookahead_distance in reverse gear
};

struct TargetValues
{
  double kappa;
  double velocity;
  double acceleration;
};

struct DebugData
{
  geometry_msgs::Point next_target;
};

class PurePursuitNode
{
public:
  PurePursuitNode();

private:
  // Node Handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_trajectory_;
  ros::Subscriber sub_current_velocity_;

  autoware_planning_msgs::Trajectory::ConstPtr trajectory_;
  geometry_msgs::TwistStamped::ConstPtr current_velocity_;

  bool isDataReady() const;

  void onTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg);
  void onCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg);

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  boost::optional<geometry_msgs::PoseStamped> current_pose_;

  // Publisher
  ros::Publisher pub_ctrl_cmd_;

  void publishCommand(const TargetValues & targets) const;

  // Debug Publisher
  ros::Publisher pub_debug_marker_;

  void publishDebugMarker() const;

  // Timer
  ros::Timer timer_;
  void onTimer(const ros::TimerEvent & event);

  // Parameter
  Param param_;

  // Algorithm
  std::unique_ptr<planning_utils::PurePursuit> pure_pursuit_;
  TargetValues target_values_;

  boost::optional<TargetValues> calcTargetValues() const;
  boost::optional<autoware_planning_msgs::TrajectoryPoint> calcTargetPoint() const;

  // Debug
  mutable DebugData debug_data_;
};
