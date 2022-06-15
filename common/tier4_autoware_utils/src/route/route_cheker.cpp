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

#include "tier4_autoware_utils/route/route_checker.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <string>

namespace tier4_autoware_utils
{
RouteChecker::RouteChecker(rclcpp::Node * node)
: clock_(node->get_clock()), logger_(node->get_logger())
{
  using std::placeholders::_1;

  sub_map_ = node->create_subscription<HADMapBin>(
    "/map/vector_map", rclcpp::QoS(1), std::bind(&RouteChecker::onMap, this, _1));
}

void RouteChecker::onMap(const HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
  is_map_msg_ready_ = true;
  is_handler_ready_ = false;
}

bool RouteChecker::isRouteValid(const HADMapRoute::ConstSharedPtr route_msg)
{
  if (!is_map_msg_ready_)
  {
    return false;
  }
  for (const auto & route_section : route_msg->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      try {
        lanelet_map_ptr_->laneletLayer.get(id);
        RCLCPP_INFO(logger_, "Route set");
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          logger_,
          "Error: %s. Maybe the loaded route was created on a different Map from the current one. "
          "Try to load the other Route again.",
          e.what());
      }
    }
  }
  return true;
}

} // namespace tier4_autoware_utils