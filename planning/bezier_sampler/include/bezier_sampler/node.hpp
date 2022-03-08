/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include <bezier_sampler/bezier.hpp>
#include <bezier_sampler/bezier_sampling.hpp>
#include <bezier_sampler/constraint_checker.hpp>
#include <bezier_sampler/conversions.hpp>
#include <bezier_sampler/path_splitting.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>

namespace bezier_sampler
{
class PathSmootherNode
{
public:
  PathSmootherNode();

private:
  // ROS
  ros::NodeHandle nh_, pnh_;
  ros::Publisher path_pub_;
  ros::Publisher debug_paths_pub_;
  ros::Subscriber path_sub_;
  SamplingParameters params_;
  ConstraintParameters cc_params_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // callback functions
  void pathCallback(const autoware_auto_planning_msgs::msg::Path & msg);

  std::unique_ptr<geometry_msgs::Pose> getCurrentEgoPose();
  int getCurrentEgoPoseIndex(const autoware_auto_planning_msgs::msg::Path & path_msg);
};
}  // namespace bezier_sampler
