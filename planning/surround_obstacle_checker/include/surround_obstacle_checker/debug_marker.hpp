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

#pragma once
#include <autoware_planning_msgs/StopReasonArray.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

enum class PoseType : int8_t { NoStart = 0 };
enum class PointType : int8_t { NoStart = 0 };
class SurroundObstacleCheckerDebugNode
{
public:
  explicit SurroundObstacleCheckerDebugNode(const double base_link2front);

  bool pushPose(const geometry_msgs::Pose & pose, const PoseType & type);
  bool pushObstaclePoint(const geometry_msgs::Point & obstacle_point, const PointType & type);
  void publish();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher debug_viz_pub_;
  ros::Publisher stop_reason_pub_;
  double base_link2front_;

  visualization_msgs::MarkerArray makeVisualizationMarker();
  autoware_planning_msgs::StopReasonArray makeStopReasonArray();

  std::shared_ptr<geometry_msgs::Point> stop_obstacle_point_ptr_;
  std::shared_ptr<geometry_msgs::Pose> stop_pose_ptr_;
};
