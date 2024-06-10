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

#ifndef ROUTING_ADAPTOR_HPP_
#define ROUTING_ADAPTOR_HPP_

#include <autoware_ad_api_specs/routing.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/polling_subscriber.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace ad_api_adaptors
{

class RoutingAdaptor : public rclcpp::Node
{
public:
  explicit RoutingAdaptor(const rclcpp::NodeOptions & options);

private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using SetRoutePoints = autoware_ad_api::routing::SetRoutePoints;
  using ChangeRoutePoints = autoware_ad_api::routing::ChangeRoutePoints;
  using ClearRoute = autoware_ad_api::routing::ClearRoute;
  using RouteState = autoware_ad_api::routing::RouteState;
  component_interface_utils::Client<ChangeRoutePoints>::SharedPtr cli_reroute_;
  component_interface_utils::Client<SetRoutePoints>::SharedPtr cli_route_;
  component_interface_utils::Client<ClearRoute>::SharedPtr cli_clear_;
  component_interface_utils::Subscription<RouteState>::SharedPtr sub_state_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_reroute_;
  // Subscriber without callback
  tier4_autoware_utils::InterProcessPollingSubscriber<PoseStamped> sub_fixed_goal_{
    this, "~/input/fixed_goal"};
  tier4_autoware_utils::InterProcessPollingSubscriber<PoseStamped> sub_rough_goal_{
    this, "~/input/rough_goal"};
  tier4_autoware_utils::InterProcessPollingSubscriber<PoseStamped> sub_waypoint_{
    this, "~/input/waypoint"};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time previous_timer_time_ = rclcpp::Time(0);

  bool calling_service_ = false;
  int request_timing_control_ = 0;
  SetRoutePoints::Service::Request::SharedPtr route_;
  RouteState::Message::_state_type state_;

  void on_timer();
  void set_route_from_goal(
    const PoseStamped::ConstSharedPtr pose, const bool allow_goal_modification);
  void on_rough_goal(const PoseStamped::ConstSharedPtr pose);
  void on_waypoint(const PoseStamped::ConstSharedPtr pose);
  void on_reroute(const PoseStamped::ConstSharedPtr pose);
};

}  // namespace ad_api_adaptors

#endif  // ROUTING_ADAPTOR_HPP_
