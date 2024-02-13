// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef MISSION_PLANNER__ROUTE_SELECTOR_HPP_
#define MISSION_PLANNER__ROUTE_SELECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tier4_planning_msgs/msg/route_state.hpp>
#include <tier4_planning_msgs/srv/clear_route.hpp>
#include <tier4_planning_msgs/srv/set_lanelet_route.hpp>
#include <tier4_planning_msgs/srv/set_waypoint_route.hpp>

#include <variant>

namespace mission_planner
{

using autoware_planning_msgs::msg::LaneletRoute;
using tier4_planning_msgs::msg::RouteState;
using tier4_planning_msgs::srv::ClearRoute;
using tier4_planning_msgs::srv::SetLaneletRoute;
using tier4_planning_msgs::srv::SetWaypointRoute;
using unique_identifier_msgs::msg::UUID;

class RouteInterface
{
public:
  void change_state(const rclcpp::Time & stamp, RouteState::_state_type state);
  void update_state(const RouteState & state);
  void update_route(const LaneletRoute & route);

  rclcpp::Service<ClearRoute>::SharedPtr srv_clear_route;
  rclcpp::Service<SetLaneletRoute>::SharedPtr srv_set_lanelet_route;
  rclcpp::Service<SetWaypointRoute>::SharedPtr srv_set_waypoint_route;
  rclcpp::Publisher<RouteState>::SharedPtr pub_state_;
  rclcpp::Publisher<LaneletRoute>::SharedPtr pub_route_;

private:
  RouteState state_;
  UUID uuid_;
};

class RouteSelector : public rclcpp::Node
{
public:
  explicit RouteSelector(const rclcpp::NodeOptions & options);

private:
  RouteInterface main_;
  RouteInterface mrm_;

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Client<ClearRoute>::SharedPtr cli_clear_route_;
  rclcpp::Client<SetWaypointRoute>::SharedPtr cli_set_waypoint_route_;
  rclcpp::Client<SetLaneletRoute>::SharedPtr cli_set_lanelet_route_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_state_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;

  using WaypointRequest = SetWaypointRoute::Request::SharedPtr;
  using LaneletRequest = SetLaneletRoute::Request::SharedPtr;
  bool initialized_;
  bool mrm_operating_;
  std::variant<std::monostate, WaypointRequest, LaneletRequest> main_request_;

  void on_state(const RouteState::ConstSharedPtr msg);
  void on_route(const LaneletRoute::ConstSharedPtr msg);

  void on_clear_route_main(ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res);
  void on_set_waypoint_route_main(
    SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res);
  void on_set_lanelet_route_main(
    SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res);

  void on_clear_route_mrm(ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res);
  void on_set_waypoint_route_mrm(
    SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res);
  void on_set_lanelet_route_mrm(
    SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res);
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ROUTE_SELECTOR_HPP_
