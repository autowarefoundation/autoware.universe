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

#ifndef ROUTE_HANDLER__LANELET_ROUTE_HPP_
#define ROUTE_HANDLER__LANELET_ROUTE_HPP_

#include "route_handler/forward.hpp"
#include "route_handler/lanelet_path.hpp"
#include "route_handler/lanelet_point.hpp"

#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/Types.h>

#include <limits>

namespace route_handler
{

using autoware_planning_msgs::msg::LaneletPrimitive;
using LaneletRouteMsg = autoware_planning_msgs::msg::LaneletRoute;
using LaneletSegmentMsg = autoware_planning_msgs::msg::LaneletSegment;

// TODO(vrichard) eventually, we should be able to replace every lanelet::ConstLanelet by
// LaneletSection. Besides the LaneletRoute building functions, LaneletRoute does not rely much on
// lanelet lib algoritms. In the long term, it should be possible to create generic routes from non
// lanelet vector maps.

//! @brief A lanelet route between a start and end pose.
//! A route is composed of all reachable lane sections around a main path (preferred lanes).
//! LaneletRoute are usually created using the LaneletRouteBuilder class.
//! Loops are not supported, however it is possible for the start and goal
//! lanelets to be the same (if the goal is behind start position). When the goal position is behind
//! the start pose on the same lane (i.e. same lanelet or a reachable neighbor), this first/last
//! lane is split virtually in 2, and everything happens as if the start and goal poses were on
//! different lanes.
class LaneletRoute
{
  friend class LaneletRouteBuilder;

public:
  //! @brief Export main path to LaneletSegmentMsg.
  [[nodiscard]] std::vector<LaneletSegmentMsg> toLaneletSegmentMsgs() const;

  // getters

  [[nodiscard]] const LaneletPath & getMainPath() const { return main_path_; }
  [[nodiscard]] const std::vector<LaneletPoint> & getStartLine() const { return start_line_; }
  [[nodiscard]] const std::vector<LaneletPoint> & getGoalLine() const { return goal_line_; }

  // utils

  //! @brief Get remaining backward distance from given point within the route boundaries.
  //! Max search distance can be set to fasten search. It is typically used if we just need to know
  //! if there is "enough" distance left to the end of the route boundaries.
  //! @return the remaining backward length within the route, 0 if the point is outside the route.
  [[nodiscard]] double getRemainingBackwardLengthWithinRoute(
    const LaneletPoint & lanelet_point,
    const double max_search_distance = std::numeric_limits<double>::infinity()) const;

  //! @brief Get remaining forward distance from given point within the route boundaries.
  //! Max search distance can be set to fasten search. It is typically used if we just need to know
  //! if there is "enough" distance left to the end of the route boundaries.
  //! @return the remaining forward length within the route, 0 if the point is outside the route.
  [[nodiscard]] double getRemainingForwardLengthWithinRoute(
    const LaneletPoint & lanelet_point,
    const double max_search_distance = std::numeric_limits<double>::infinity()) const;

  //! @brief Get arc length of a point on the route main path.
  //! If not on the main path, the point is projected on the preferred neighbor first.
  //! @return the arc length on the main path, nothing if the point is outside the route.
  [[nodiscard]] std::optional<double> getRouteArcLength(const LaneletPoint & lanelet_point) const;

  // path finding

  //! @brief Get a straight path (no lane change) from given lanelet point.
  //! Path extends both forward and backward, and is limited by given backward and forward distance
  //! If within_route is true, the path is also restricted to the route boundaries (from start to
  //! goal, within the route lanelets). If within_route is false, eventual overlapping path sections
  //! will be removed according to chosen strategy
  //! @note There is not necessarily a unique solution. This function just returns one.
  //! @return a path if found, an empty path otherwise.
  [[nodiscard]] LaneletPath getStraightPath(
    const LaneletPoint & lanelet_point, const double backward_distance,
    const double forward_distance, const bool within_route = true,
    const OverlapRemovalStrategy overlap_removal_strategy = OverlapRemovalStrategy::SPLIT) const;

  //! @brief Forward only version of getStraightPath().
  [[nodiscard]] LaneletPath getStraightPathFrom(
    const LaneletPoint & lanelet_point, const double forward_distance,
    const bool within_route = true,
    const OverlapRemovalStrategy overlap_removal_strategy = OverlapRemovalStrategy::SPLIT) const;

  //! @brief Backward only version of getStraightPath().
  [[nodiscard]] LaneletPath getStraightPathUpTo(
    const LaneletPoint & lanelet_point, const double backward_distance,
    const bool within_route = true,
    const OverlapRemovalStrategy overlap_removal_strategy = OverlapRemovalStrategy::SPLIT) const;

  // path conversion

  // TODO(vrichard) temporarily glue code because lanelets are used as path everywhere in the path
  // planning modules
  //! @brief Create a path covering all the input lanelets
  //! @return the path if valid, an empty path otherwise
  [[nodiscard]] LaneletPath getPathFromLanelets(const lanelet::ConstLanelets & lanelets) const;
  // TODO(vrichard) temporarily glue code because lanelets are used as path everywhere in the path
  // planning modules
  //! @brief Create a path from the input lanelets, a start and goal point
  //! start and goal points must be on the lanelets
  //! @return the path if valid, an empty path otherwise
  [[nodiscard]] LaneletPath getPathFromLanelets(
    const lanelet::ConstLanelets & lanelets, const LaneletPoint & start_point,
    const LaneletPoint & goal_point) const;
  // TODO(vrichard) temporarily glue code because lanelets are used as path everywhere in the path
  // planning modules
  //! @brief Create a path from the input lanelets, a start and goal point
  //! @param lanelets the sequence of lanelets composing the path
  //! @param start_pose is projected to the closest lanelet centerline.
  //! @param goal_pose is projected to the closest lanelet centerline.
  //! @return the path if valid, an empty path otherwise
  [[nodiscard]] LaneletPath getPathFromLanelets(
    const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & goal_pose) const;

  // path manipulation

  //! @brief Extend given path forward and backward by given amount.
  //! If within_route is true, extended path will be restricted to the route boundaries (from start
  //! to goal, within route lanelets) If within_route is false, eventual overlapping path sections
  //! will be removed according to chosen strategy
  //! @note This function is similar to getStraightPath(), except it takes a path instead of a point
  //! as input
  //! @return the extended path if successful, an empty path otherwise.
  [[nodiscard]] LaneletPath extendPath(
    const LaneletPath & lanelet_path, const double backward_distance, const double forward_distance,
    const bool within_route,
    const OverlapRemovalStrategy overlap_removal_strategy = OverlapRemovalStrategy::SPLIT) const;

  //! @brief Change lane of the last lanelet of the path
  //! The target lanelet must be reachable from the last point of the path
  //! @return modified path if successful, an empty path otherwise
  [[nodiscard]] LaneletPath changeLastLane(
    const LaneletPath & lanelet_path, const lanelet::ConstLanelet & lane_change_target,
    const bool within_route) const;

  // query within route

  //! @brief Get closest lanelet point to pose within route.
  //! @return closest lanelet point if found, an invalid point otherwise.
  [[nodiscard]] LaneletPoint getClosestLaneletPointWithinRoute(
    const geometry_msgs::msg::Pose pose) const;

  //! @brief Get left lanelet within the route, if any.
  [[nodiscard]] std::optional<lanelet::ConstLanelet> getLeftLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet) const;

  //! @brief Get right lanelet within the route, if any.
  [[nodiscard]] std::optional<lanelet::ConstLanelet> getRightLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet) const;

  //! @brief List all neighbors of given lanelet within route.
  [[nodiscard]] lanelet::ConstLanelets getNeighborsWithinRoute(
    const lanelet::ConstLanelet & lanelet) const;

  //! @brief Whether the current point is in preferred lane section.
  //! @note Also consider points before/after start/goal but on the route lanelets
  [[nodiscard]] bool isInPreferredLane(const LaneletPoint & lanelet_point) const;

  //! @brief Get required number of lane changes to reach preferred lane section
  //! @note Also consider points before/after start/goal but on the route lanelets
  //! @return signed number of lane changes (left > 0, right < 0), 0 if the point is on a preferred
  //! lane, nothing if the point is not on the route lanelets.
  [[nodiscard]] std::optional<int> getNumLaneChangeToPreferredLane(
    const LaneletPoint & lanelet_point) const;

  // lanelet query

  // TODO(vrichard) makes this private when outside code has been fixed (planning functions should
  // not manipulate lanelets directly) planning modules should not use lanelets as path!
  //! @brief Get lanelet following current position (forward only, no lane change).
  //! If within_route is true, search is restricted to route lanelets.
  [[nodiscard]] lanelet::ConstLanelets getFollowingLanelets(
    const LaneletPoint & lanelet_point, const bool within_route = true) const;

  // TODO(vrichard) makes this private when outside code has been fixed (planning functions should
  // not manipulate lanelets directly) planning modules should not use lanelets as path!
  //! @brief List all lanelets before current position (backward only, no lane change).
  //! If within_route is true, search is restricted to route lanelets.
  [[nodiscard]] lanelet::ConstLanelets getPreviousLanelets(
    const LaneletPoint & lanelet_point, const bool within_route = true) const;

  // utils
  // TODO(vrichard) makes this private when outside code has been fixed
  // (99% of the time you just want to use getRemainingBackwardLengthWithinRoute())
  // planning modules should not use lanelets as path!
  //! @brief Whether the point is on the first "segment" of the route.
  //! The first segment is the full first lane in most cases.
  //! But if the goal pose is behind the start pose on the same lane, the full lane is "split" in 2,
  //! and the first segment corresponds to the lane section on the start pose side.
  [[nodiscard]] bool isOnFirstSegment(const LaneletPoint & lanelet_point) const;

  // TODO(vrichard) makes this private when outside code has been fixed
  // (99% of the time you just want to use getRemainingForwardLengthWithinRoute())
  // planning modules should not use lanelets as path!
  //! @brief Whether the point is on the last "segment" of the route.
  //! The last segment is the full last lane in most cases.
  //! But if the goal pose is behind the start pose on the same lane, the full lane is "split" in 2,
  //! and the last segment corresponds to the lane section on the goal pose side.
  [[nodiscard]] bool isOnLastSegment(const LaneletPoint & lanelet_point) const;

private:
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;

  // TODO(vrichard) eventually, we should get rid of lanelet::ConstLanelets and use LaneletSections
  // instead
  lanelet::ConstLanelets route_lanelets_;  //!< all reachable lanelets within the route
  LaneletSections start_segments_;         //!< the starting lanelets sections
  LaneletSections goal_segments_;          //!< the ending lanelets sections
  std::vector<LaneletPoint>
    start_line_;                         //!< start point projected on each of the starting segments
  std::vector<LaneletPoint> goal_line_;  //!< goal point projected on each of the goaling segments
  LaneletPath main_path_;                //!< preferred path from start to goal

  rclcpp::Logger logger_{rclcpp::get_logger("route_handler")};
};

}  // namespace route_handler

#endif  // ROUTE_HANDLER__LANELET_ROUTE_HPP_
