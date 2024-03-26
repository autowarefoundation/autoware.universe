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
    "input/odometry", rclcpp::QoS(1), std::bind(&ReroutingStaticObstacle::on_odom, this, _1));

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
  adaptor.init_cli(cli_set_lanelet_route_);
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
  geometry_msgs::msg::Point selected_point{msg->point};

  lanelet::ConstLanelets selected_point_lanelets;

  auto selected_point_lanelet_found =
    get_selected_point_lanelets(selected_point, selected_point_lanelets);

  lanelet::ConstLanelets remaining_route_lanelets;
  auto remaining_route_lanelets_found = get_remaining_route_lanelets(remaining_route_lanelets);

  lanelet::ConstLanelet selected_point_lanelet;
  bool selected_point_in_route{false};
  if (selected_point_lanelet_found && remaining_route_lanelets_found) {
    selected_point_in_route = is_selected_point_in_route(
      selected_point_lanelets, remaining_route_lanelets, selected_point_lanelet);
  } else {
    return;
  }

  if (selected_point_in_route) {
    lanelet::routing::LaneletPath alternative_route_lanelets;
    auto alternative_route_found =
      search_alternative_route(selected_point_lanelet, alternative_route_lanelets);

    if (alternative_route_found) {
      set_lanelet_route(alternative_route_lanelets);
    }
  }
}

bool ReroutingStaticObstacle::get_selected_point_lanelets(
  const geometry_msgs::msg::Point & selected_point,
  lanelet::ConstLanelets & selected_point_lanelets) const
{
  if (!lanelet::utils::query::getCurrentLanelets(
        road_lanelets_, selected_point, &selected_point_lanelets)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Failed to find selected_point current lanelet.");
    return false;
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
  const lanelet::ConstLanelets & selected_point_lanelets,
  const lanelet::ConstLanelets & remaining_route_lanelets,
  lanelet::ConstLanelet & selected_point_lanelet) const
{
  for (const auto & slc_pnt_llt : selected_point_lanelets) {
    for (const auto & rmn_rt_llt : remaining_route_lanelets) {
      if (slc_pnt_llt.id() == rmn_rt_llt.id()) {
        selected_point_lanelet = slc_pnt_llt;
        return true;
      }
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

void ReroutingStaticObstacle::set_lanelet_route(const lanelet::routing::LaneletPath & lanelet_path)
{
  const auto req = std::make_shared<SetLaneletRoute::Service::Request>();
  req->header.frame_id = "map";
  req->header.stamp = this->get_clock()->now();
  req->goal_pose = goal_pose_;
  convert_lanelet_path_to_route_segments(lanelet_path, req->segments);
  cli_set_lanelet_route_->async_send_request(req);
}

void ReroutingStaticObstacle::convert_lanelet_path_to_route_segments(
  const lanelet::routing::LaneletPath & lanelet_path,
  std::vector<autoware_planning_msgs::msg::LaneletSegment> & route_segments) const
{
  for (const auto & ll : lanelet_path) {
    autoware_planning_msgs::msg::LaneletSegment route_segment;
    autoware_planning_msgs::msg::LaneletPrimitive lanelet_primitive;
    lanelet_primitive.id = ll.id();
    lanelet_primitive.primitive_type = "lane";
    route_segment.primitives.push_back(lanelet_primitive);
    route_segments.push_back(route_segment);
  }
}

}  // namespace mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mission_planner::ReroutingStaticObstacle)
