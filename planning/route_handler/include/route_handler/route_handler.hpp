// Copyright 2021 Tier IV, Inc.
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

#ifndef ROUTE_HANDLER__ROUTE_HANDLER_HPP_
#define ROUTE_HANDLER__ROUTE_HANDLER_HPP_

#include "route_handler/lanelet_route.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <motion_utils/motion_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace route_handler
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using LaneletRouteMsg = autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Header;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;

enum class LaneChangeDirection { NONE, LEFT, RIGHT };
enum class PullOverDirection { NONE, LEFT, RIGHT };
enum class PullOutDirection { NONE, LEFT, RIGHT };

class RouteHandler
{
public:
  RouteHandler() = default;
  explicit RouteHandler(const HADMapBin & map_msg);

  // non-const methods (initialization)

  //! @brief load lanelet map from binary message
  void setMap(const HADMapBin & map_msg);
  //! @brief load route from message
  //! requires the lanelet map to have been loaded first with setMap()
  //! @return whether the route was loaded successfully
  bool setRoute(const LaneletRouteMsg & route_msg);
  //! @brief build a new route from start pose to goal pose
  //! requires lanelet map to have been loaded first with setMap()
  //! @return whether the route was built successfully
  bool buildRoute(const Pose & start, const Pose & goal);

  // const methods

  bool isRouteMsgValid(const LaneletRouteMsg & route_msg) const;

  // for route handler status
  bool isHandlerReady() const { return is_handler_ready_; }
  bool isMapMsgReady() const { return is_map_msg_ready_; }
  Header getRouteHeader() const { return route_msg_.header; }

  // for routing
  lanelet::routing::RoutingGraphPtr getRoutingGraphPtr() const { return routing_graph_ptr_; }
  lanelet::traffic_rules::TrafficRulesPtr getTrafficRulesPtr() const { return traffic_rules_ptr_; }
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> getOverallGraphPtr() const
  {
    return overall_graphs_ptr_;
  }
  lanelet::LaneletMapPtr getLaneletMapPtr() const { return lanelet_map_ptr_; }
  LaneletRoutePtr getLaneletRoutePtr() const { return lanelet_route_ptr_; }

  // for lanelet
  lanelet::ConstLanelet getLaneletsFromId(const lanelet::Id id) const;
  lanelet::ConstPolygon3d getIntersectionAreaById(const lanelet::Id id) const;

  // for pull over
  lanelet::ConstLanelets getShoulderLanelets() const { return shoulder_lanelets_; }
  lanelet::ConstLanelets getShoulderLaneletSequence(
    const lanelet::ConstLanelet & lanelet, const Pose & pose,
    const double backward_distance = std::numeric_limits<double>::max(),
    const double forward_distance = std::numeric_limits<double>::max()) const;
  static bool getPullOverTarget(
    const lanelet::ConstLanelets & lanelets, const Pose & goal_pose,
    lanelet::ConstLanelet * target_lanelet);
  static bool getPullOutStartLane(
    const lanelet::ConstLanelets & lanelets, const Pose & pose, const double vehicle_width,
    lanelet::ConstLanelet * target_lanelet);

  // for goal
  Pose getGoalPose() const { return goal_pose_; }
  // bool isInGoalRouteSection(const LaneletPoint & lanelet_point) const;
  // lanelet::Id getGoalLaneId() const;
  // bool getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const;
  // bool getGoalLaneletPoint(LaneletPoint * goal_lanelet_point) const;

  // std::vector<lanelet::ConstLanelet> getLanesBeforePose(
  //   const geometry_msgs::msg::Pose & pose, const double vehicle_length) const;
  // std::vector<lanelet::ConstLanelet> getLanesAfterGoal(const double vehicle_length) const;

  // for lanelet
  // bool getPreviousLaneletsWithinRoute(
  //   const LaneletPoint & lanelet_point, lanelet::ConstLanelets * prev_lanelets) const;
  // bool isDeadEndLanelet(const LaneletPoint & lanelet_point) const;
  // lanelet::ConstLanelets getLaneletsFromPoint(const lanelet::ConstPoint3d & point) const;

  // for lane change
  // int getNumLaneToPreferredLane(const LaneletPoint & lanelet_point) const;
  lanelet::ConstLanelets getCheckTargetLanesFromPath(
    const PathWithLaneId & path, const lanelet::ConstLanelets & target_lanes,
    const double check_length) const;
  [[nodiscard]] bool getNextLaneChangeTarget(
    const LaneletPath & lanelet_path, lanelet::ConstLanelet * target_lanelet) const;

  /**
   * @brief Check if same-direction lane is available at the right side of the lanelet
   * Searches for any lanes regardless of whether it is lane-changeable or not.
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet having same direction if true
   */
  boost::optional<lanelet::ConstLanelet> getRightLanelet(
    const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Check if same-direction lane is available at the left side of the lanelet
   * Searches for any lanes regardless of whether it is lane-changeable or not.
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet having same direction if true
   */
  boost::optional<lanelet::ConstLanelet> getLeftLanelet(
    const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Check if opposite-direction lane is available at the right side of the lanelet
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet with opposite direction if true
   */
  lanelet::Lanelets getRightOppositeLanelets(const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Check if opposite-direction lane is available at the left side of the lanelet
   * Required the linestring to be shared between(same line ID) the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet with opposite direction if true
   */
  lanelet::Lanelets getLeftOppositeLanelets(const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Searches and return all lanelet on the left that shares same linestring
   * @param the lanelet of interest
   * @param (optional) flag to include the lane with opposite direction
   * @param (optional) flag to invert the opposite lanelet
   * @return vector of lanelet that is connected via share linestring
   */
  lanelet::ConstLanelets getAllLeftSharedLinestringLanelets(
    const lanelet::ConstLanelet & lane, const bool & include_opposite,
    const bool & invert_opposite = false) const noexcept;

  /**
   * @brief Searches and return all lanelet on the right that shares same linestring
   * @param the lanelet of interest
   * @param (optional) flag to include the lane with opposite direction
   * @param (optional) flag to invert the opposite lanelet
   * @return vector of lanelet that is connected via share linestring
   */
  lanelet::ConstLanelets getAllRightSharedLinestringLanelets(
    const lanelet::ConstLanelet & lane, const bool & include_opposite,
    const bool & invert_opposite = false) const noexcept;

  /**
   * @brief Searches and return all lanelet (left and right) that shares same linestring
   * @param the lanelet of interest
   * @param (optional) flag to search only right side
   * @param (optional) flag to search only left side
   * @param (optional) flag to include the lane with opposite direction
   * @param (optional) flag to invert the opposite lanelet
   * @return vector of lanelet that is connected via share linestring
   */
  lanelet::ConstLanelets getAllSharedLineStringLanelets(
    const lanelet::ConstLanelet & current_lane, bool is_right = true, bool is_left = true,
    bool is_opposite = true, const bool & invert_opposite = false) const noexcept;

  /**
   * @brief Check if same-direction lane is available at the left side of the lanelet
   * Searches for any lanes regardless of whether it is lane-changeable or not.
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet having same direction if true
   */
  lanelet::ConstLanelet getMostLeftLanelet(const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Searches the furthest linestring to the right side of the lanelet
   * Only lanelet with same direction is considered
   * @param the lanelet of interest
   * @return right most linestring of the lane with same direction
   */
  lanelet::ConstLineString3d getRightMostSameDirectionLinestring(
    const lanelet::ConstLanelet & lanelet) const noexcept;

  /**
   * @brief Searches the furthest linestring to the right side of the lanelet
   * Used to search for road shoulders. Lane direction is ignored
   * @param the lanelet of interest
   * @return right most linestring
   */
  lanelet::ConstLineString3d getRightMostLinestring(
    const lanelet::ConstLanelet & lanelet) const noexcept;

  /**
   * @brief Searches the furthest linestring to the left side of the lanelet
   * Only lanelet with same direction is considered
   * @param the lanelet of interest
   * @return left most linestring of the lane with same direction
   */
  lanelet::ConstLineString3d getLeftMostSameDirectionLinestring(
    const lanelet::ConstLanelet & lanelet) const noexcept;

  /**
   * @brief Searches the furthest linestring to the left side of the lanelet
   * Used to search for road shoulders. Lane direction is ignored
   * @param the lanelet of interest
   * @return left most linestring
   */
  lanelet::ConstLineString3d getLeftMostLinestring(
    const lanelet::ConstLanelet & lanelet) const noexcept;

  /**
   * @brief Return furthest linestring on both side of the lanelet
   * @param the lanelet of interest
   * @param (optional) search furthest right side
   * @param (optional) search furthest left side
   * @param (optional) include opposite lane as well
   * @return right and left linestrings
   */
  lanelet::ConstLineStrings3d getFurthestLinestring(
    const lanelet::ConstLanelet & lanelet, bool is_right = true, bool is_left = true,
    bool is_opposite = true) const noexcept;

  // lanelet::ConstLanelets getNextLanelets(const lanelet::ConstLanelet & lanelet) const;
  // lanelet::ConstLanelets getPreviousLanelets(const lanelet::ConstLanelet & lanelet) const;

  // bool getClosestLaneletWithinRoute(
  //   const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const;
  lanelet::ConstLanelets getLaneletsFromIds(const lanelet::Ids & ids) const;
  // LaneletPath getStraightPath(
  //   const LaneletPoint & lanelet_point,
  //   const double backward_distance = std::numeric_limits<double>::max(),
  //   const double forward_distance = std::numeric_limits<double>::max()) const;
  // lanelet::routing::RelationType getRelation(
  //   const lanelet::ConstLanelet & prev_lane, const lanelet::ConstLanelet & next_lane) const;

  // for path centerline
  PathWithLaneId getCenterLinePath(const LaneletPath & lanelet_path, bool use_exact = true) const;

private:
  // MUST
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets shoulder_lanelets_;
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;
  LaneletRouteMsg route_msg_;

  LaneletRoutePtr lanelet_route_ptr_;

  rclcpp::Logger logger_{rclcpp::get_logger("route_handler")};

  bool is_route_msg_ready_{false};
  bool is_map_msg_ready_{false};
  bool is_handler_ready_{false};

  // non-const methods (initialization)

  //! @brief create route from route_msg_ if available.
  //! @return whether the route was successfully created.
  void buildRouteFromMsg();

  // const methods

  // for lanelet
  // bool isInTargetLane(const PoseStamped & pose, const lanelet::ConstLanelets & target) const;
  // bool isInPreferredLane(const PoseStamped & pose) const;
  // bool getRightLaneletWithinRoute(
  //   const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet) const;
  // bool getLeftLaneletWithinRoute(
  //   const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet) const;
  // lanelet::ConstLanelets getNextLaneSequence(const lanelet::ConstLanelets & lane_sequence) const;

  // for pull over
  bool getFollowingShoulderLanelet(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * following_lanelet) const;
  lanelet::ConstLanelets getShoulderLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
  bool getPreviousShoulderLanelet(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const;
  lanelet::ConstLanelets getShoulderLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
};

}  // namespace route_handler

#endif  // ROUTE_HANDLER__ROUTE_HANDLER_HPP_
