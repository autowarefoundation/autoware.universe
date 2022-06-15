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

#ifndef TIER4_AUTOWARE_UTILS__ROUTE__ROUTE_CHECKER_HPP_
#define TIER4_AUTOWARE_UTILS__ROUTE__ROUTE_CHECKER_HPP_

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <memory>

namespace tier4_autoware_utils
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::HADMapRoute;

class RouteChecker
{
public:
  explicit RouteChecker(rclcpp::Node * node);

  bool isRouteValid(const HADMapRoute::ConstSharedPtr route);
  rclcpp::Logger getLogger() { return logger_; }
  
private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  void onMap(const HADMapBin::ConstSharedPtr map);
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  bool is_map_msg_ready_{false};
  bool is_handler_ready_{false};
};

} // namespace tier4_autoware_utils

#endif // TIER4_AUTOWARE_UTILS__ROUTE__ROUTE_CHECKER_HPP_