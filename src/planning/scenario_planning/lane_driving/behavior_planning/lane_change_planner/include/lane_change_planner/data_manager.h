/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#ifndef LANE_CHANGE_PLANNER_DATA_MANAGER_H
#define LANE_CHANGE_PLANNER_DATA_MANAGER_H

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/Route.h>
#include <lane_change_planner/parameters.h>

// lanelet
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// other
#include <memory>

namespace lane_change_planner
{
class SelfPoseLinstener
{
public:
  SelfPoseLinstener();
  bool getSelfPose(geometry_msgs::PoseStamped & self_pose);
  bool isSelfPoseReady();

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

struct BoolStamped
{
  explicit BoolStamped(bool in_data) : data(in_data) {}
  bool data = false;
  ros::Time stamp;
};

class DataManager
{
private:
  /*
   * Cache
   */
  autoware_perception_msgs::DynamicObjectArray::ConstPtr perception_ptr_;
  geometry_msgs::TwistStamped::ConstPtr vehicle_velocity_ptr_;
  BoolStamped lane_change_approval_;
  BoolStamped force_lane_change_;
  geometry_msgs::PoseStamped self_pose_;

  // ROS parameters
  LaneChangerParameters parameters_;
  bool is_parameter_set_;

  /*
   * SelfPoseLinstener
   */
  std::shared_ptr<SelfPoseLinstener> self_pose_listener_ptr_;

public:
  DataManager();
  ~DataManager() = default;

  // callbacks
  void perceptionCallback(
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr & input_perception_msg);
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr & input_twist_msg);
  void setLaneChangerParameters(const LaneChangerParameters & parameters);
  void laneChangeApprovalCallback(const std_msgs::Bool & input_approval_msg);
  void forceLaneChangeSignalCallback(const std_msgs::Bool & input_approval_msg);

  // getters
  autoware_perception_msgs::DynamicObjectArray::ConstPtr getDynamicObjects();
  geometry_msgs::PoseStamped getCurrentSelfPose();
  geometry_msgs::TwistStamped::ConstPtr getCurrentSelfVelocity();
  LaneChangerParameters getLaneChangerParameters();
  bool getLaneChangeApproval();
  bool getForceLaneChangeSignal();

  bool isDataReady();
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_DATA_MANAGER_H
