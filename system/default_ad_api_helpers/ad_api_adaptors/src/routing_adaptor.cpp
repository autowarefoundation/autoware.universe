// Copyright 2022 TIER IV, Inc.
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

#include "routing_adaptor.hpp"

#include <memory>

namespace ad_api_adaptors
{

RoutingAdaptor::RoutingAdaptor(const rclcpp::NodeOptions & options)
: Node("routing_adaptor", options)
{
  using std::placeholders::_1;

  sub_reroute_ = create_subscription<PoseStamped>(
    "~/input/reroute", 3, std::bind(&RoutingAdaptor::on_reroute, this, _1));

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_cli(cli_reroute_);
  adaptor.init_cli(cli_route_);
  adaptor.init_cli(cli_clear_);
  adaptor.init_sub(
    sub_state_, [this](const RouteState::Message::ConstSharedPtr msg) { state_ = msg->state; });

  const auto rate = rclcpp::Rate(5.0);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rate.period(), std::bind(&RoutingAdaptor::on_timer, this));

  state_ = RouteState::Message::UNKNOWN;
  route_ = std::make_shared<SetRoutePoints::Service::Request>();
}

void RoutingAdaptor::on_timer()
{
  const rclcpp::Time current_time = this->get_clock()->now();
  auto fixed_goal_msg = sub_fixed_goal_.takeData();
  auto rough_goal_msg = sub_rough_goal_.takeData();
  auto waypoint_msg = sub_waypoint_.takeData();
  if (
    fixed_goal_msg && rough_goal_msg &&
    rclcpp::Time(fixed_goal_msg->header.stamp) > previous_timer_time_ &&
    rclcpp::Time(rough_goal_msg->header.stamp) > previous_timer_time_) {
    if (rclcpp::Time(fixed_goal_msg->header.stamp) > rclcpp::Time(rough_goal_msg->header.stamp)) {
      request_timing_control_ = 1;
      set_route_from_goal(fixed_goal_msg, false);
    } else {
      request_timing_control_ = 1;
      set_route_from_goal(rough_goal_msg, true);
    }
  } else if (fixed_goal_msg && rclcpp::Time(fixed_goal_msg->header.stamp) > previous_timer_time_) {
    request_timing_control_ = 1;
    set_route_from_goal(fixed_goal_msg, false);
  } else if (rough_goal_msg && rclcpp::Time(rough_goal_msg->header.stamp) > previous_timer_time_) {
    request_timing_control_ = 1;
    set_route_from_goal(rough_goal_msg, true);
  }
  if (waypoint_msg && rclcpp::Time(waypoint_msg->header.stamp) > previous_timer_time_) {
    if (route_->header.frame_id != waypoint_msg->header.frame_id) {
      RCLCPP_ERROR_STREAM(get_logger(), "The waypoint frame does not match the goal.");
      previous_timer_time_ = current_time;
      return;
    }
    request_timing_control_ = 1;
    route_->waypoints.push_back(waypoint_msg->pose);
  }
  previous_timer_time_ = current_time;

  // Wait a moment to combine consecutive goals and checkpoints into a single request.
  // This value is rate dependent and set the wait time for merging.
  constexpr int delay_count = 3;  // 0.4 seconds (rate * (value - 1))
  if (0 < request_timing_control_ && request_timing_control_ < delay_count) {
    ++request_timing_control_;
  }
  if (request_timing_control_ != delay_count) {
    return;
  }

  if (!calling_service_) {
    if (state_ != RouteState::Message::UNSET) {
      const auto request = std::make_shared<ClearRoute::Service::Request>();
      calling_service_ = true;
      cli_clear_->async_send_request(request, [this](auto) { calling_service_ = false; });
    } else {
      request_timing_control_ = 0;
      calling_service_ = true;
      cli_route_->async_send_request(route_, [this](auto) { calling_service_ = false; });
    }
  }
}

void RoutingAdaptor::set_route_from_goal(
  const PoseStamped::ConstSharedPtr msg, const bool allow_goal_modification)
{
  route_->header = msg->header;
  route_->goal = msg->pose;
  route_->waypoints.clear();
  route_->option.allow_goal_modification = allow_goal_modification;
}

void RoutingAdaptor::on_reroute(const PoseStamped::ConstSharedPtr pose)
{
  const auto route = std::make_shared<SetRoutePoints::Service::Request>();
  route->header = pose->header;
  route->goal = pose->pose;
  cli_reroute_->async_send_request(route);
}

}  // namespace ad_api_adaptors

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ad_api_adaptors::RoutingAdaptor)
