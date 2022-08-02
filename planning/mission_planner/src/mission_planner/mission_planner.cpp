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

#include "mission_planner.hpp"

#include "type_conversion.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace mission_planner
{

MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options)
: Node("mission_planner", options),
  arrival_checker_(this),
  plugin_loader_("mission_planner", "mission_planner::PlannerPlugin"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter<std::string>("map_frame");
  base_link_frame_ = declare_parameter<std::string>("base_link_frame");

  planner_ = plugin_loader_.createSharedInstance("mission_planner::lanelet2::DefaultPlanner");
  planner_->Initialize(this);

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  pub_had_route_ = create_publisher<HADMapRoute>("output/route", durable_qos);
  pub_marker_ = create_publisher<MarkerArray>("debug/route_marker", durable_qos);

  const auto period = rclcpp::Duration::from_seconds(1.0);
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { OnArrivalCheck(); });

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_pub(pub_state_);
  node.init_pub(pub_api_route_);
  node.init_srv(srv_clear_route_, this, &MissionPlanner::OnClearRoute);
  node.init_srv(srv_set_route_, this, &MissionPlanner::OnSetRoute);
  node.init_srv(srv_set_route_points_, this, &MissionPlanner::OnSetRoutePoints);

  ChangeState(RouteState::Message::UNSET);
}

PoseStamped MissionPlanner::GetEgoVehiclePose()
{
  PoseStamped base_link_origin;
  base_link_origin.header.frame_id = base_link_frame_;
  base_link_origin.pose.position.x = 0;
  base_link_origin.pose.position.y = 0;
  base_link_origin.pose.position.z = 0;
  base_link_origin.pose.orientation.x = 0;
  base_link_origin.pose.orientation.y = 0;
  base_link_origin.pose.orientation.z = 0;
  base_link_origin.pose.orientation.w = 1;

  //  transform base_link frame origin to map_frame to get vehicle positions
  return TransformPose(base_link_origin);
}

PoseStamped MissionPlanner::TransformPose(const PoseStamped & input)
{
  PoseStamped output;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(map_frame_, input.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input, output, transform);
    return output;
  } catch (tf2::TransformException & error) {
    throw component_interface_utils::TransformError(error.what());
  }
}

void MissionPlanner::OnArrivalCheck()
{
  // NOTE: Do not check in the changing state as goal may change.
  if (state_.state == RouteState::Message::SET) {
    if (arrival_checker_.IsArrived(GetEgoVehiclePose())) {
      ChangeState(RouteState::Message::ARRIVED);
    }
  }
}

void MissionPlanner::ChangeRoute()
{
  arrival_checker_.ResetGoal();
  pub_api_route_->publish(conversion::CreateEmptyRoute(now()));
}

void MissionPlanner::ChangeRoute(const HADMapRoute & route)
{
  // TODO(Takagi, Isamu): replace when modified goal is always published
  // arrival_checker_.ResetGoal();
  PoseStamped goal;
  goal.header = route.header;
  goal.pose = route.goal_pose;
  arrival_checker_.ResetGoal(goal);

  pub_api_route_->publish(conversion::ConvertRoute(route));
  pub_had_route_->publish(route);
  pub_marker_->publish(planner_->Visualize(route));
}

void MissionPlanner::ChangeState(RouteState::Message::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

void MissionPlanner::OnClearRoute(
  const ClearRoute::Service::Request::SharedPtr, const ClearRoute::Service::Response::SharedPtr res)
{
  // NOTE: The route services should be mutually exclusive by callback group.
  RCLCPP_INFO_STREAM(get_logger(), "ClearRoute");

  ChangeRoute();
  ChangeState(RouteState::Message::UNSET);
  res->status.success = true;
}

void MissionPlanner::OnSetRoute(
  const SetRoute::Service::Request::SharedPtr req, const SetRoute::Service::Response::SharedPtr res)
{
  // NOTE: The route services should be mutually exclusive by callback group.
  if (state_.state != RouteState::Message::UNSET) {
    throw component_interface_utils::ServiceException(
      SetRoute::Service::Response::ERROR_ROUTE_EXISTS, "The route is already set.");
  }

  // Use common header for transforms
  PoseStamped pose;
  pose.header = req->header;

  // Convert route points.
  autoware_ad_api_msgs::msg::RouteBody body;
  body.segments = req->segments;
  if (req->start.empty()) {
    body.start = GetEgoVehiclePose().pose;
  } else {
    pose.pose = req->start.front();
    body.start = TransformPose(pose).pose;
  }
  pose.pose = req->goal;
  body.goal = TransformPose(pose).pose;

  // Convert route.
  HADMapRoute route = conversion::ConvertRoute(body);
  route.header.stamp = req->header.stamp;
  route.header.frame_id = map_frame_;

  // Update route.
  ChangeRoute(route);
  ChangeState(RouteState::Message::SET);
  res->status.success = true;
}

void MissionPlanner::OnSetRoutePoints(
  const SetRoutePoints::Service::Request::SharedPtr req,
  const SetRoutePoints::Service::Response::SharedPtr res)
{
  // NOTE: The route services should be mutually exclusive by callback group.
  if (state_.state != RouteState::Message::UNSET) {
    throw component_interface_utils::ServiceException(
      SetRoute::Service::Response::ERROR_ROUTE_EXISTS, "The route is already set.");
  }
  if (!planner_->Ready()) {
    throw component_interface_utils::ServiceException(
      SetRoutePoints::Service::Response::ERROR_PLANNER_UNREADY, "The planner is not ready.");
  }

  // Use common header for transforms.
  PoseStamped pose;
  pose.header = req->header;

  // Convert route points.
  PlannerPlugin::RoutePoints points;
  if (req->start.empty()) {
    points.push_back(GetEgoVehiclePose().pose);
  } else {
    pose.pose = req->start.front();
    points.push_back(TransformPose(pose).pose);
  }
  for (const auto & waypoint : req->waypoints) {
    pose.pose = waypoint;
    points.push_back(TransformPose(pose).pose);
  }
  pose.pose = req->goal;
  points.push_back(TransformPose(pose).pose);

  // Plan route.
  HADMapRoute route = planner_->Plan(points);
  if (route.segments.empty()) {
    throw component_interface_utils::ServiceException(
      SetRoutePoints::Service::Response::ERROR_PLANNER_FAILED, "The planned route is empty.");
  }
  route.header.stamp = req->header.stamp;
  route.header.frame_id = map_frame_;

  // Update route.
  ChangeRoute(route);
  ChangeState(RouteState::Message::SET);
  res->status.success = true;
}

}  // namespace mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mission_planner::MissionPlanner)
