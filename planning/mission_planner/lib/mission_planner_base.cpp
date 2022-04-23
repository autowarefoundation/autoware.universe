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

#include "mission_planner/mission_planner_base.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>

#include <lanelet2_routing/Route.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker_array.h>

#include <string>

namespace mission_planner
{
MissionPlanner::MissionPlanner(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter("map_frame", "map");

  using std::placeholders::_1;

  loop_subscriber_ = create_subscription<std_msgs::msg::Bool>(
    "input/loop", 10, std::bind(&MissionPlanner::loopCallback, this, _1));
  goal_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/goal_pose", 10, std::bind(&MissionPlanner::goalPoseCallback, this, _1));
  checkpoint_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/checkpoint", 10, std::bind(&MissionPlanner::checkpointCallback, this, _1));

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  route_publisher_ =
    create_publisher<autoware_auto_planning_msgs::msg::HADMapRoute>("output/route", durable_qos);
  marker_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("debug/route_marker", durable_qos);

  self_pose_listener_.waitForFirstPose();

  const auto planning_hz = declare_parameter("planning_hz", 1);
  const auto period_ns = rclcpp::Rate(planning_hz).period();
  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&MissionPlanner::run, this));
}

boost::optional<geometry_msgs::msg::PoseStamped> MissionPlanner::transformPose(
  const geometry_msgs::msg::PoseStamped & input_pose, const std::string & target_frame)
{
  geometry_msgs::msg::PoseStamped output_pose;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform =
      tf_buffer_.lookupTransform(target_frame, input_pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input_pose, output_pose, transform);
    return output_pose;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return {};
  }
}

void MissionPlanner::run()
{
  if (!is_looped_route_ || route_.segments.empty()) {
    return;
  }

  const auto start_pose = *self_pose_listener_.getCurrentPose();

  // publish new route if the ego's closest lanelet moves forward
  geometry_msgs::msg::Pose goal_pose;
  const auto final_idx = getClosestRouteSectionIndex(route_, start_pose, goal_pose);
  if (!loop_idx_ || loop_idx_.get() != final_idx.get()) {
    auto route = route_;
    route.segments.clear();
    route.segments.insert(
      route.segments.end(), route_.segments.begin() + final_idx.get(), route_.segments.end());
    route.segments.insert(
      route.segments.end(), route_.segments.begin(),
      route_.segments.end() - (route_.segments.size() - final_idx.get()));
    route.goal_pose = goal_pose;

    publishRoute(route);

    loop_idx_ = final_idx.get();
  }
}

void MissionPlanner::loopCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if (msg->data) {
    RCLCPP_INFO(get_logger(), "Route will be looped.");
  } else {
    RCLCPP_INFO(get_logger(), "Route will not be looped.");
  }
  is_looped_route_ = msg->data;
}

void MissionPlanner::goalPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_msg_ptr)
{
  const auto start_pose = *self_pose_listener_.getCurrentPose();

  // set goal pose
  goal_pose_ = transformPose(*goal_msg_ptr, map_frame_);
  if (!goal_pose_) {
    RCLCPP_ERROR(get_logger(), "Failed to get goal pose in map frame. Aborting mission planning");
    return;
  }
  RCLCPP_INFO(get_logger(), "New goal pose is set.");

  std::vector<geometry_msgs::msg::PoseStamped> pass_points;
  pass_points.push_back(start_pose);
  pass_points.push_back(goal_pose_.get());

  if (!isRoutingGraphReady()) {
    RCLCPP_ERROR(get_logger(), "RoutingGraph is not ready. Aborting mission planning");
    return;
  }

  autoware_auto_planning_msgs::msg::HADMapRoute route = planRoute(pass_points);
  publishRoute(route);
}

void MissionPlanner::checkpointCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr checkpoint_msg_ptr)
{
  const auto transformed_checkpoint = transformPose(*checkpoint_msg_ptr, map_frame_);
  if (!transformed_checkpoint) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get checkpoint pose in map frame. Aborting mission planning");
    return;
  }

  const auto start_pose = *self_pose_listener_.getCurrentPose();

  if (!isRoutingGraphReady()) {
    RCLCPP_ERROR(get_logger(), "RoutingGraph is not ready. Aborting mission planning");
    return;
  }

  if (is_looped_route_) {
    checkpoints_.push_back(transformed_checkpoint.get());

    std::vector<geometry_msgs::msg::PoseStamped> pass_points;
    pass_points.push_back(start_pose);
    pass_points.insert(pass_points.end(), checkpoints_.begin(), checkpoints_.end());
    pass_points.push_back(start_pose);

    route_ = planRoute(pass_points, true);
    // route_.segments.pop_back();
  } else {
    if (!goal_pose_) {
      RCLCPP_ERROR(
        get_logger(),
        "You must set start and goal before setting checkpoints. Aborting mission planning");
      return;
    }

    checkpoints_.push_back(transformed_checkpoint.get());

    std::vector<geometry_msgs::msg::PoseStamped> pass_points;
    pass_points.push_back(start_pose);
    pass_points.insert(pass_points.end(), checkpoints_.begin(), checkpoints_.end());
    pass_points.push_back(goal_pose_.get());

    autoware_auto_planning_msgs::msg::HADMapRoute route = planRoute(pass_points);
    publishRoute(route);
  }
}

void MissionPlanner::publishRoute(const autoware_auto_planning_msgs::msg::HADMapRoute & route) const
{
  if (!route.segments.empty()) {
    RCLCPP_INFO(get_logger(), "Route successfully planned. Publishing...");
    route_publisher_->publish(route);
    visualizeRoute(route);
  } else {
    RCLCPP_ERROR(get_logger(), "Calculated route is empty!");
  }
}

}  // namespace mission_planner
