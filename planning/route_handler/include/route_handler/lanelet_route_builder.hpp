// Copyright 2022 Macnica, Inc.
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

#ifndef ROUTE_HANDLER__LANELET_ROUTE_BUILDER_HPP_
#define ROUTE_HANDLER__LANELET_ROUTE_BUILDER_HPP_

#include "route_handler/forward.hpp"
#include "route_handler/lanelet_route.hpp"

#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/Types.h>

#include <limits>

namespace route_handler
{

using autoware_planning_msgs::msg::LaneletPrimitive;
using LaneletRouteMsg = autoware_planning_msgs::msg::LaneletRoute;
using LaneletSegmentMsg = autoware_planning_msgs::msg::LaneletSegment;

//! @brief An helper class for building LaneletRoute
class LaneletRouteBuilder
{
public:
  //! @brief Initialize the LaneletRouteBuilder object.
  LaneletRouteBuilder(
    const lanelet::LaneletMapPtr lanelet_map_ptr,
    const lanelet::routing::RoutingGraphPtr routing_graph_ptr);

  //! @brief Build a route from start pose to goal pose.
  //! @note Even though lanelet::routing is used in the build process, the route building logic is
  //! not exactly the same, and the resulting route and may not contain exactly the same lanelets
  //! than its lanelet::routing::Route counterpart.
  //! @return the route if built successfully, nothing otherwise.
  [[nodiscard]] LaneletRoutePtr buildFromStartGoalPose(
    const geometry_msgs::msg::Pose & start_pose, 
    const geometry_msgs::msg::Pose & goal_pose) const;

  //! @brief Initialize the route from the message.
  //! @return true if the route was build successfully, false otherwise.
  [[nodiscard]] LaneletRoutePtr buildFromLaneletRouteMsg(
    const LaneletRouteMsg & route_msg) const;

private:

  //! @brief Compute shortest lanelet route from start to goal pose.
  [[nodiscard]] lanelet::Optional<lanelet::routing::Route> getShortestRoute(
    const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose) const;

  //! @brief Initialize start/goal segments and start/goal line
  //! In most cases, start and goal segments contains all the lanelets around the start and goal point
  //! But if the goal is behind the start point on the same or a neighbor lanelet, the sections are shrinked so that they don't overlap.
  //! @return true if successful.
  [[nodiscard]] bool updateStartGoalSegments(
    LaneletRoute * lanelet_route_ptr,
    const LaneletPoint & start_point,
    const LaneletPoint & goal_point
  ) const;

  //! @brief List all reachable lanelets in the route.
  [[nodiscard]] lanelet::ConstLanelets getRouteLanelets(const lanelet::ConstLanelets & path_lanelets) const;

  //! @brief Get main path from start to goal.
  //! @note By construction, the main path starts at the beginning of the first preferred lanelet, and
  //! goes to the goal point on the goal lanelet. The first lanelet of the main path may be
  //! different from the route start lanelet (typically if there is a lane change on the first
  //! lanelet).
  //! @return true if successful.
  [[nodiscard]] bool updateMainPath(
    LaneletRoute * lanelet_route_ptr,
    const LaneletPoint & start_point,
    const LaneletPoint & goal_point
  ) const;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;

  rclcpp::Logger logger_{rclcpp::get_logger("route_handler")};
};

}  // namespace route_handler

#endif  // ROUTE_HANDLER__LANELET_ROUTE_BUILDER_HPP_
