/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LANE_CHANGE_PLANNER_ROUTE_HANDLER_H
#define LANE_CHANGE_PLANNER_ROUTE_HANDLER_H

// Autoware
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/Route.h>
#include <geometry_msgs/PoseStamped.h>
#include <lanelet2_extension/utility/query.h>
// lanelet
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>

#include <vector>

namespace lane_change_planner
{
enum class LaneChangeDirection { NONE, LEFT, RIGHT };

class RouteHandler
{
public:
  RouteHandler();
  ~RouteHandler() = default;

private:
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  autoware_planning_msgs::Route route_msg_;

  bool is_map_msg_ready_;
  bool is_route_msg_ready_;
  bool is_handler_ready_;

  lanelet::ConstLanelets route_lanelets_;
  lanelet::ConstLanelets preferred_lanelets_;
  lanelet::ConstLanelets start_lanelets_;
  lanelet::ConstLanelets goal_lanelets_;

  void setRouteLanelets();

  friend class LaneChanger;

public:
  RouteHandler(
    const lanelet::LaneletMapConstPtr & lanelet_map_ptr,
    const lanelet::routing::RoutingGraphPtr & routing_graph, const lanelet::routing::Route & route);
  bool isHandlerReady();

  void mapCallback(const autoware_lanelet2_msgs::MapBin & map_msg);
  void routeCallback(const autoware_planning_msgs::Route & route_msg);

  bool getPreviousLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const;
  bool getNextLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const;
  bool getRightLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet);
  bool getLeftLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet);

  bool getClosestLaneletWithinRoute(
    const geometry_msgs::Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const;
  bool getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const;
  geometry_msgs::Pose getGoalPose() const;
  lanelet::Id getGoalLaneId() const;
  lanelet::ConstLanelets getLaneletsFromIds(const std::vector<uint64_t> ids) const;

  bool isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const;
  bool isInTargetLane(
    const geometry_msgs::PoseStamped & pose, const lanelet::ConstLanelets & target) const;
  bool isInGoalRouteSection(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getRouteLanelets() const;
  lanelet::ConstLanelets getLaneletSequence(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneletSequence(
    const lanelet::ConstLanelet & lanelet, const geometry_msgs::Pose & current_pose,
    const double backward_distance, const double forward_distance) const;
  lanelet::ConstLanelets getLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
  lanelet::ConstLanelets getLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
  lanelet::ConstLanelets getPreviousLaneletSequence(
    const lanelet::ConstLanelets & lanelet_sequence) const;
  lanelet::ConstLanelets getClosestLaneletSequence(const geometry_msgs::Pose & pose) const;
  lanelet::ConstLanelets getNeighborsWithinRoute(const lanelet::ConstLanelet & lanelet) const;

  int getNumLaneToPreferredLane(const lanelet::ConstLanelet & lanelet) const;
  bool isInPreferredLane(const geometry_msgs::PoseStamped & pose) const;
  autoware_planning_msgs::PathWithLaneId getLaneChangePath(
    const lanelet::ConstLanelets & original_lanes, const lanelet::ConstLanelets & target_lanes,
    const geometry_msgs::Pose & pose, const geometry_msgs::Twist & twist,
    const double backward_path_length, const double forward_path_length,
    const double lane_change_prepare_duration, const double lane_changing_duration,
    const double minimum_lane_change_length) const;
  autoware_planning_msgs::PathWithLaneId getReferencePath(
    const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::Pose & pose,
    const double backward_path_length, const double forward_path_length,
    const double minimum_lane_change_length) const;
  autoware_planning_msgs::PathWithLaneId getReferencePath(
    const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
    bool use_exact = true) const;
  bool getLaneChangeTarget(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * target_lanelet) const;
  lanelet::ConstLanelets getLaneChangeTarget(const geometry_msgs::Pose & pose) const;

  double getLaneChangeableDistance(
    const geometry_msgs::Pose & current_pose, const LaneChangeDirection & direction);
};
}  // namespace lane_change_planner
#endif  // LANE_CHANGE_PLANNER_ROUTE_HANDLER_H
