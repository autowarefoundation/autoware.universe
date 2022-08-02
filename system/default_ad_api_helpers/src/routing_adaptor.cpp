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

#include "routing_adaptor.hpp"

#include <memory>

namespace default_ad_api_helpers
{

RoutingAdaptor::RoutingAdaptor() : Node("routing_adaptor")
{
  RCLCPP_INFO_STREAM(get_logger(), "RoutingAdaptor");

  sub_goal_ = create_subscription<PoseStamped>(
    "input/goal", 5, std::bind(&RoutingAdaptor::OnGoal, this, std::placeholders::_1));
  sub_waypoints_ = create_subscription<PoseStamped>(
    "input/checkpoint", 5, std::bind(&RoutingAdaptor::OnWaypoint, this, std::placeholders::_1));

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_cli(cli_route_);
  node.init_cli(cli_clear_);
  route_points_ = std::make_shared<SetRoutePoints::Service::Request>();
}

void RoutingAdaptor::OnGoal(const PoseStamped::ConstSharedPtr pose)
{
  route_points_->header = pose->header;
  route_points_->goal = pose->pose;
  route_points_->waypoints.clear();

  cli_clear_->async_send_request(std::make_shared<ClearRoute::Service::Request>());
  cli_route_->async_send_request(route_points_);
}

void RoutingAdaptor::OnWaypoint(const PoseStamped::ConstSharedPtr pose)
{
  if (route_points_->header.frame_id != pose->header.frame_id) {
    RCLCPP_ERROR_STREAM(get_logger(), "The waypoint frame does not match the goal.");
  }
  route_points_->waypoints.push_back(pose->pose);

  cli_clear_->async_send_request(std::make_shared<ClearRoute::Service::Request>());
  cli_route_->async_send_request(route_points_);
}

}  // namespace default_ad_api_helpers

#include "macros/create_node.hpp"
CREATE_SINGLE_THREAD_NODE(default_ad_api_helpers::RoutingAdaptor)
