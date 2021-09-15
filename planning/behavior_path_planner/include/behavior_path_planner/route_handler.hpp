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

#ifndef BEHAVIOR_PATH_PLANNER__ROUTE_HANDLER_HPP_
#define BEHAVIOR_PATH_PLANNER__ROUTE_HANDLER_HPP_

#include <limits>
#include <memory>
#include <vector>

#include "lanelet2_routing/Route.h"
#include "lanelet2_routing/RoutingCost.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/RoutingGraphContainer.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

#include "autoware_lanelet2_msgs/msg/map_bin.hpp"
#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_planning_msgs/msg/route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "rclcpp/rclcpp.hpp"

#include "behavior_path_planner/parameters.hpp"

namespace behavior_path_planner
{
using autoware_lanelet2_msgs::msg::MapBin;
using autoware_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::Route;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using std_msgs::msg::Header;

enum class LaneChangeDirection { NONE, LEFT, RIGHT };

struct LaneChangePath
{
  PathWithLaneId path;
  double acceleration{0.0};
  double preparation_length{0.0};
  double lane_change_length{0.0};
};

class RouteHandler
{
public:
  RouteHandler() = default;
  RouteHandler(
    const lanelet::LaneletMapConstPtr & lanelet_map_ptr,
    const lanelet::routing::RoutingGraphPtr & routing_graph, const lanelet::routing::Route & route);

  // non-const methods

  void setMap(const MapBin & map_msg);

  void setRoute(const Route & route_msg);

  // const methods

  // for route handler status

  bool isHandlerReady() const;

  lanelet::ConstPolygon3d getExtraDrivableAreaById(const lanelet::Id id) const;

  Header getRouteHeader() const;

  lanelet::routing::RoutingGraphContainer getOverallGraph() const;

  // for goal
  bool isInGoalRouteSection(const lanelet::ConstLanelet & lanelet) const;

  Pose getGoalPose() const;

  lanelet::Id getGoalLaneId() const;

  bool getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const;

  std::vector<lanelet::ConstLanelet> getLanesAfterGoal(const double vehicle_length) const;

  // for lanelet

  int getNumLaneToPreferredLane(const lanelet::ConstLanelet & lanelet) const;

  bool getClosestLaneletWithinRoute(
    const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const;

  lanelet::ConstLanelet getLaneletsFromId(const lanelet::Id id) const;

  lanelet::ConstLanelets getLaneletsFromIds(const lanelet::Ids ids) const;

  lanelet::ConstLanelets getLaneletSequence(const lanelet::ConstLanelet & lanelet) const;

  lanelet::ConstLanelets getLaneletSequence(
    const lanelet::ConstLanelet & lanelet, const Pose & current_pose,
    const double backward_distance, const double forward_distance) const;

  lanelet::ConstLanelets getCheckTargetLanesFromPath(
    const PathWithLaneId & path,
    const lanelet::ConstLanelets & target_lanes, const double check_length) const;

  lanelet::routing::RelationType getRelation(
    const lanelet::ConstLanelet & prev_lane, const lanelet::ConstLanelet & next_lane) const;

  // for path

  std::vector<LaneChangePath> getLaneChangePaths(
    const lanelet::ConstLanelets & original_lanes, const lanelet::ConstLanelets & target_lanes,
    const Pose & pose, const Twist & twist,
    const BehaviorPathPlannerParameters & parameter) const;

  PathWithLaneId getCenterLinePath(
    const lanelet::ConstLanelets & lanelet_sequence, const Pose & pose,
    const double backward_path_length, const double forward_path_length,
    const BehaviorPathPlannerParameters & parameter) const;

  PathWithLaneId getCenterLinePath(
    const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
    bool use_exact = true) const;

  PathWithLaneId setDecelerationVelocity(
    const PathWithLaneId & input,
    const lanelet::ConstLanelets & lanelet_sequence, const double lane_change_prepare_duration,
    const double lane_change_buffer) const;

  bool getLaneChangeTarget(
    const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const;

  double getLaneChangeableDistance(
    const Pose & current_pose, const LaneChangeDirection & direction) const;

  lanelet::ConstPolygon3d getIntersectionAreaById(const lanelet::Id id) const;

private:
  // MUST
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  // maybe necessary
  lanelet::ConstLanelets route_lanelets_;
  lanelet::ConstLanelets preferred_lanelets_;
  lanelet::ConstLanelets start_lanelets_;
  lanelet::ConstLanelets goal_lanelets_;

  Route route_msg_;

  rclcpp::Logger logger_{rclcpp::get_logger("behavior_path_planner").get_child("route_handler")};

  bool is_route_msg_ready_{false};
  bool is_map_msg_ready_{false};
  bool is_handler_ready_{false};

  // non-const methods

  void setRouteLanelets();

  // const methods

  // for lanelet

  bool isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const;

  bool isInTargetLane(
    const PoseStamped & pose, const lanelet::ConstLanelets & target) const;

  bool isInPreferredLane(const PoseStamped & pose) const;

  bool getPreviousLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const;

  bool getNextLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const;

  bool getRightLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet) const;

  bool getLeftLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet) const;

  lanelet::ConstLanelets getRouteLanelets() const;

  lanelet::ConstLanelets getLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;

  lanelet::ConstLanelets getLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;

  lanelet::ConstLanelets getPreviousLaneletSequence(
    const lanelet::ConstLanelets & lanelet_sequence) const;

  lanelet::ConstLanelets getClosestLaneletSequence(const Pose & pose) const;

  lanelet::ConstLanelets getNeighborsWithinRoute(const lanelet::ConstLanelet & lanelet) const;

  lanelet::ConstLanelets getLaneChangeTargetLanes(const Pose & pose) const;

  // for path

  PathWithLaneId updatePathTwist(
    const PathWithLaneId & path) const;
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_PLANNER__ROUTE_HANDLER_HPP_
