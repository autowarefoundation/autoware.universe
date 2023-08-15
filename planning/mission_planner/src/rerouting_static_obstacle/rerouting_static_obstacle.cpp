// Copyright 2023 TIER IV, Inc.
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

#include "rerouting_static_obstacle.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <limits>
#include <memory>

namespace mission_planner
{
ReroutingStaticObstacle::ReroutingStaticObstacle(const rclcpp::NodeOptions & node_options)
: Node("rerouting_static_obstacle", node_options)
{
  using std::placeholders::_1;

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", rclcpp::QoS(1),
    std::bind(&ReroutingStaticObstacle::on_odom, this, _1));

  sub_route_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&ReroutingStaticObstacle::route_callback, this, std::placeholders::_1));

  map_subscriber_ = create_subscription<HADMapBin>(
    "input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&ReroutingStaticObstacle::map_callback, this, std::placeholders::_1));

  sub_trigger_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "input/reroute_point", rclcpp::QoS(1),
    std::bind(&ReroutingStaticObstacle::on_trigger, this, _1));

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_cli(cli_change_route_);
}

void ReroutingStaticObstacle::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
}

void ReroutingStaticObstacle::route_callback(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  goal_pose_ = msg->goal_pose;
}

void ReroutingStaticObstacle::map_callback(const HADMapBin::ConstSharedPtr msg)
{
  route_handler_.setMap(*msg);
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
}

void ReroutingStaticObstacle::on_trigger(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  geometry_msgs::msg::Pose selected_point;
  selected_point.position = msg->point;

  lanelet::ConstLanelet selected_point_lanelet;
  bool selected_point_lanelet_found{false};
  selected_point_lanelet_found = get_selected_point_lanelet(selected_point, selected_point_lanelet);

  lanelet::ConstLanelets remaining_route_lanelets;
  bool remaining_route_lanelets_found{false};
  remaining_route_lanelets_found = get_remaining_route_lanelets(remaining_route_lanelets);

  bool selected_point_in_route{false};
  if (selected_point_lanelet_found && remaining_route_lanelets_found) {
    selected_point_in_route =
      is_selected_point_in_route(selected_point_lanelet, remaining_route_lanelets);
  } else {
    return;
  }

  if (selected_point_in_route) {
    lanelet::routing::LaneletPath alternative_route_lanelets;
    bool alternative_route_found{false};
    alternative_route_found =
      search_alternative_route(selected_point_lanelet, alternative_route_lanelets);

    if (alternative_route_found) {
      change_route(alternative_route_lanelets);
    }
  }
}

bool ReroutingStaticObstacle::get_selected_point_lanelet(
  const geometry_msgs::msg::Pose & selected_point,
  lanelet::ConstLanelet & selected_point_lanelet) const
{
  lanelet::ConstLanelets selected_point_lanelets;
  if (!lanelet::utils::query::getCurrentLanelets(
        road_lanelets_, selected_point, &selected_point_lanelets)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Failed to find selected_point current lanelet.");
    return false;
  }

  if (selected_point_lanelets.size() > 1) {  // This will happen only if we have co-located lanes at
                                             // the point the user selects
    // TODO(AhmedEbrahim) find a way to deduce the selected point orientation
    if (!lanelet::utils::query::getClosestLanelet(
          road_lanelets_, selected_point, &selected_point_lanelet)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to find selected_point closest lanelet.");
      return false;
    }

  } else {
    selected_point_lanelet = selected_point_lanelets.at(0);
  }

  return true;
}

bool ReroutingStaticObstacle::get_remaining_route_lanelets(
  lanelet::ConstLanelets & remaining_route_lanelets) const
{
  return route_handler_.planPathLaneletsBetweenCheckpoints(
    current_pose_, goal_pose_, &remaining_route_lanelets);
}

bool ReroutingStaticObstacle::is_selected_point_in_route(
  const lanelet::ConstLanelet & selected_point_lanelet,
  const lanelet::ConstLanelets & remaining_route_lanelets) const
{
  for (const auto & llt : remaining_route_lanelets) {
    if (llt.id() == selected_point_lanelet.id()) {
      return true;
    }
  }

  return false;
}

bool ReroutingStaticObstacle::search_alternative_route(
  const lanelet::ConstLanelet & selected_point_lanelet,
  lanelet::routing::LaneletPath & alternative_route_lanelets) const
{
  bool alternative_route_found{false};

  lanelet::Optional<lanelet::routing::Route> optional_route;

  lanelet::ConstLanelet current_lanelet;
  double alternative_route_length2d = std::numeric_limits<double>::max();
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets_, current_pose_, &current_lanelet)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Failed to find current_lanelet.");
    return false;
  }

  lanelet::ConstLanelet goal_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets_, goal_pose_, &goal_lanelet)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Failed to find goal_lanelet.");
    return false;
  }
  for (const auto & llt : road_lanelets_) {
    lanelet::ConstLanelets via_lanelet;
    via_lanelet.push_back(llt);
    optional_route = routing_graph_ptr_->getRouteVia(current_lanelet, via_lanelet, goal_lanelet, 0);

    if (
      (optional_route) && (!optional_route->contains(selected_point_lanelet)) &&
      (optional_route->length2d() < alternative_route_length2d)) {
      alternative_route_length2d = optional_route->length2d();
      alternative_route_lanelets = optional_route->shortestPath();
      alternative_route_found = true;
    }
    via_lanelet.clear();
  }

  return alternative_route_found;
}

void ReroutingStaticObstacle::change_route(const lanelet::routing::LaneletPath & lanelet_path)
{
  const auto req = std::make_shared<ChangeRoute::Service::Request>();
  req->header.frame_id = "map";
  req->header.stamp = this->get_clock()->now();
  req->goal = goal_pose_;
  convert_lanelet_path_to_route_segments(lanelet_path, req->segments);
  cli_change_route_->async_send_request(req);
}

void ReroutingStaticObstacle::convert_lanelet_path_to_route_segments(
  const lanelet::routing::LaneletPath & lanelet_path,
  std::vector<autoware_adapi_v1_msgs::msg::RouteSegment> & route_segments) const
{
  for (const auto & ll : lanelet_path) {
    autoware_adapi_v1_msgs::msg::RouteSegment route_segment;
    route_segment.preferred.id = ll.id();
    route_segment.preferred.type = "lane";
    route_segments.push_back(route_segment);
  }
}

}  // namespace mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mission_planner::ReroutingStaticObstacle)
