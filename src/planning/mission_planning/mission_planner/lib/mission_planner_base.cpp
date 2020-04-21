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
#include <mission_planner/mission_planner_base.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_routing/Route.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>

namespace mission_planner
{
MissionPlanner::MissionPlanner() : pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");

  goal_subscriber_ = pnh_.subscribe("input/goal_pose", 10, &MissionPlanner::goalPoseCallback, this);
  checkpoint_subscriber_ =
    pnh_.subscribe("input/checkpoint", 10, &MissionPlanner::checkpointCallback, this);

  route_publisher_ = pnh_.advertise<autoware_planning_msgs::Route>("output/route", 1, true);
  marker_publisher_ =
    pnh_.advertise<visualization_msgs::MarkerArray>("debug/route_marker", 1, true);
}

bool MissionPlanner::getEgoVehiclePose(geometry_msgs::PoseStamped * ego_vehicle_pose)
{
  geometry_msgs::PoseStamped base_link_origin;
  base_link_origin.header.frame_id = base_link_frame_;
  base_link_origin.pose.position.x = 0;
  base_link_origin.pose.position.y = 0;
  base_link_origin.pose.position.z = 0;
  base_link_origin.pose.orientation.x = 0;
  base_link_origin.pose.orientation.y = 0;
  base_link_origin.pose.orientation.z = 0;
  base_link_origin.pose.orientation.w = 1;

  //  transform base_link frame origin to map_frame to get vehicle positions
  return transformPose(base_link_origin, ego_vehicle_pose, map_frame_);
}

bool MissionPlanner::transformPose(
  const geometry_msgs::PoseStamped & input_pose, geometry_msgs::PoseStamped * output_pose,
  const std::string target_frame)
{
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(target_frame, input_pose.header.frame_id, ros::Time(0));
    tf2::doTransform(input_pose, *output_pose, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
}

void MissionPlanner::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr & goal_msg_ptr)
{
  // set start pose
  if (!getEgoVehiclePose(&start_pose_)) {
    ROS_ERROR("Failed to get ego vehicle pose in map frame. Aborting mission planning");
    return;
  }
  // set goal pose
  if (!transformPose(*goal_msg_ptr, &goal_pose_, map_frame_)) {
    ROS_ERROR("Failed to get goal pose in map frame. Aborting mission planning");
    return;
  }

  ROS_INFO("New goal pose is set. Reset checkpoints.");
  checkpoints_.clear();
  checkpoints_.push_back(start_pose_);
  checkpoints_.push_back(goal_pose_);

  if (!isRoutingGraphReady()) {
    ROS_ERROR("RoutingGraph is not ready. Aborting mission planning");
    return;
  }

  autoware_planning_msgs::Route route = planRoute();
  publishRoute(route);
}  // namespace mission_planner

void MissionPlanner::checkpointCallback(
  const geometry_msgs::PoseStampedConstPtr & checkpoint_msg_ptr)
{
  if (checkpoints_.size() < 2) {
    ROS_ERROR("You must set start and goal before setting checkpoints. Aborting mission planning");
    return;
  }

  geometry_msgs::PoseStamped transformed_checkpoint;
  if (!transformPose(*checkpoint_msg_ptr, &transformed_checkpoint, map_frame_)) {
    ROS_ERROR("Failed to get checkpoint pose in map frame. Aborting mission planning");
    return;
  }

  // insert checkpoint before goal
  checkpoints_.insert(checkpoints_.end() - 1, transformed_checkpoint);

  autoware_planning_msgs::Route route = planRoute();
  publishRoute(route);
}

void MissionPlanner::publishRoute(const autoware_planning_msgs::Route & route) const
{
  if (!route.route_sections.empty()) {
    ROS_INFO("Route successfuly planned. Publishing...");
    route_publisher_.publish(route);
    visualizeRoute(route);
  } else {
    ROS_ERROR("Calculated route is empty!");
  }
}

}  // namespace mission_planner
