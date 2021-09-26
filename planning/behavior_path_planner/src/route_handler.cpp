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

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_set>
#include <string>
#include <vector>

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/primitives/LaneletSequence.h"

#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spline_interpolation/spline_interpolation.hpp"
#include "tf2/utils.h"

#include "behavior_path_planner/route_handler.hpp"
#include "behavior_path_planner/utilities.hpp"

namespace
{
using lanelet::utils::to2D;

template<typename T>
bool exists(const std::vector<T> & vectors, const T & item)
{
  for (const auto & i : vectors) {
    if (i == item) {
      return true;
    }
  }
  return false;
}

bool isRouteLooped(const autoware_planning_msgs::msg::Route & route_msg)
{
  const auto & route_sections = route_msg.route_sections;
  for (std::size_t i = 0; i < route_sections.size(); i++) {
    const auto & route_section = route_sections.at(i);
    for (const auto & lane_id : route_section.lane_ids) {
      for (std::size_t j = i + 1; j < route_sections.size(); j++) {
        const auto & future_route_section = route_sections.at(j);
        if (exists(future_route_section.lane_ids, lane_id)) {
          return true;
        }
      }
    }
  }
  return false;
}

lanelet::ConstPoint3d get3DPointFrom2DArcLength(
  const lanelet::ConstLanelets & lanelet_sequence, const double s)
{
  double accumulated_distance2d = 0;
  for (const auto & llt : lanelet_sequence) {
    const auto & centerline = llt.centerline();
    lanelet::ConstPoint3d prev_pt;
    if (!centerline.empty()) {
      prev_pt = centerline.front();
    }
    for (const auto & pt : centerline) {
      double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated_distance2d + distance2d > s) {
        double ratio = (s - accumulated_distance2d) / distance2d;
        auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
        return lanelet::ConstPoint3d(
          lanelet::InvalId, interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z());
      }
      accumulated_distance2d += distance2d;
      prev_pt = pt;
    }
  }
  return lanelet::ConstPoint3d();
}

}  // namespace

namespace behavior_path_planner
{
using autoware_planning_msgs::msg::PathPointWithLaneId;

void RouteHandler::setMap(const MapBin & map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);

  is_map_msg_ready_ = true;
  is_handler_ready_ = false;

  setRouteLanelets();
}

void RouteHandler::setRoute(const Route & route_msg)
{
  if (!isRouteLooped(route_msg)) {
    route_msg_ = route_msg;
    is_route_msg_ready_ = true;
    is_handler_ready_ = false;
    setRouteLanelets();
  } else {
    RCLCPP_ERROR(
      logger_,
      "Loop detected within route! Currently, no loop is allowed for route! Using previous route");
  }
}

bool RouteHandler::isHandlerReady() const {return is_handler_ready_;}

void RouteHandler::setRouteLanelets()
{
  if (!is_route_msg_ready_ || !is_map_msg_ready_) {
    return;
  }
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  for (const auto & route_section : route_msg_.route_sections) {
    for (const auto & id : route_section.lane_ids) {
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      route_lanelets_.push_back(llt);
      if (id == route_section.preferred_lane_id) {
        preferred_lanelets_.push_back(llt);
      }
    }
  }
  goal_lanelets_.clear();
  start_lanelets_.clear();
  if (!route_msg_.route_sections.empty()) {
    for (const auto & id : route_msg_.route_sections.back().lane_ids) {
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      goal_lanelets_.push_back(llt);
    }
    for (const auto & id : route_msg_.route_sections.front().lane_ids) {
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      start_lanelets_.push_back(llt);
    }
  }
  is_handler_ready_ = true;
}

lanelet::ConstPolygon3d RouteHandler::getIntersectionAreaById(const lanelet::Id id) const
{
  return lanelet_map_ptr_->polygonLayer.get(id);
}

Header RouteHandler::getRouteHeader() const {return route_msg_.header;}

std::vector<lanelet::ConstLanelet> RouteHandler::getLanesAfterGoal(
  const double vehicle_length) const
{
  lanelet::ConstLanelet goal_lanelet;
  if (getGoalLanelet(&goal_lanelet)) {
    const double min_succeeding_length = vehicle_length * 2;
    const auto succeeding_lanes_vec = lanelet::utils::query::getSucceedingLaneletSequences(
      routing_graph_ptr_, goal_lanelet, min_succeeding_length);
    if (succeeding_lanes_vec.empty()) {
      return std::vector<lanelet::ConstLanelet>{};
    } else {
      return succeeding_lanes_vec.front();
    }
  } else {
    return std::vector<lanelet::ConstLanelet>{};
  }
}

lanelet::ConstLanelets RouteHandler::getRouteLanelets() const {return route_lanelets_;}

Pose RouteHandler::getGoalPose() const {return route_msg_.goal_pose;}

void RouteHandler::setPullOverGoalPose(
  const lanelet::ConstLanelet target_lane, const double vehicle_width, const double margin)
{
  const auto arc_position_goal =
    lanelet::utils::getArcCoordinates({target_lane}, route_msg_.goal_pose);
  Path centerline_path = util::convertToPathFromPathWithLaneId(
    getCenterLinePath({target_lane}, 0.0, arc_position_goal.length + 10));
  const auto seg_idx =
    autoware_utils::findNearestSegmentIndex(centerline_path.points, route_msg_.goal_pose.position);
  const double d_lat = autoware_utils::calcLongitudinalOffsetToSegment(
    centerline_path.points, seg_idx, route_msg_.goal_pose.position);
  const auto shoulder_point =
    autoware_utils::calcOffsetPose(centerline_path.points.at(seg_idx).pose, d_lat, 0.0, 0.0);
  pull_over_goal_pose_.orientation = shoulder_point.orientation;
  pull_over_goal_pose_.position = shoulder_point.position;

  // distance between shoulder lane's left boundary and shoulder lane center
  double distance_shoulder_to_left_boundary =
    util::getDistanceToShoulderBoundary({target_lane}, shoulder_point);

  // distance between shoulder lane center and target line
  double distance_shoulder_to_target = distance_shoulder_to_left_boundary + vehicle_width / 2 +
    margin;

  // Apply shifting shoulder lane to adjust to target line
  double offset = -distance_shoulder_to_target;

  double yaw = tf2::getYaw(shoulder_point.orientation);
  pull_over_goal_pose_.position.x = shoulder_point.position.x - std::sin(yaw) * offset;
  pull_over_goal_pose_.position.y = shoulder_point.position.y + std::cos(yaw) * offset;
}

Pose RouteHandler::getPullOverGoalPose() const {return pull_over_goal_pose_;}

lanelet::Id RouteHandler::getGoalLaneId() const
{
  if (route_msg_.route_sections.empty()) {
    return lanelet::InvalId;
  } else {
    return route_msg_.route_sections.back().preferred_lane_id;
  }
}

bool RouteHandler::getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const
{
  const lanelet::Id goal_lane_id = getGoalLaneId();
  for (const auto & llt : route_lanelets_) {
    if (llt.id() == goal_lane_id) {
      *goal_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::isInGoalRouteSection(const lanelet::ConstLanelet & lanelet) const
{
  if (route_msg_.route_sections.empty()) {
    return false;
  } else {
    return exists(route_msg_.route_sections.back().lane_ids, lanelet.id());
  }
}

lanelet::ConstLanelets RouteHandler::getLaneletsFromIds(const lanelet::Ids ids) const
{
  lanelet::ConstLanelets lanelets;
  for (const auto & id : ids) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  return lanelets;
}

lanelet::ConstLanelet RouteHandler::getLaneletsFromId(const lanelet::Id id) const
{
  return lanelet_map_ptr_->laneletLayer.get(id);
}

bool RouteHandler::isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & route_section : route_msg_.route_sections) {
    if (exists(route_section.continued_lane_ids, lanelet.id())) {
      return false;
    }
  }
  return true;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length += boost::geometry::length(next_lanelet.centerline().basicLineString());
  }

  return lanelet_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  double length = 0;

  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousLaneletWithinRoute(current_lanelet, &prev_lanelet)) {
      break;
    }
    lanelet_sequence_backward.push_back(prev_lanelet);
    length += boost::geometry::length(prev_lanelet.centerline().basicLineString());
    current_lanelet = prev_lanelet;
  }

  std::reverse(lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(const lanelet::ConstLanelet & lanelet) const
{
  Pose tmp_pose{};
  tmp_pose.orientation.w = 1;
  if (!lanelet.centerline().empty()) {
    tmp_pose.position = lanelet::utils::conversion::toGeomMsgPt(lanelet.centerline().front());
  }
  constexpr double double_max = std::numeric_limits<double>::max();
  return getLaneletSequence(lanelet, tmp_pose, double_max, double_max);
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & current_pose,
  const double backward_distance, const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  lanelet::ConstLanelets lanelet_sequence_backward;
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet_sequence_forward = getLaneletSequenceAfter(lanelet, forward_distance);

  const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
  if (arc_coordinate.length < backward_distance) {
    lanelet_sequence_backward = getLaneletSequenceUpTo(lanelet, backward_distance);
  }

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

bool RouteHandler::getFollowingShoulderLanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * following_lanelet) const
{
  Pose back_pose;
  back_pose.position.x = lanelet.centerline2d().back().x();
  back_pose.position.y = lanelet.centerline2d().back().y();
  back_pose.position.z = 0;

  lanelet::ArcCoordinates arc_coordinates;
  const auto & centerline_2d = to2D(lanelet.centerline());

  for (const auto & shoulder_lanelet : shoulder_lanelets_) {
    Pose front_pose;
    front_pose.position.x = shoulder_lanelet.centerline2d().front().x();
    front_pose.position.y = shoulder_lanelet.centerline2d().front().y();
    front_pose.position.z = 0;
    if (
      std::hypot(
        front_pose.position.x - back_pose.position.x,
        front_pose.position.y - back_pose.position.y) < 5)
    {
      *following_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(shoulder_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getFollowingShoulderLanelet(current_lanelet, &next_lanelet)) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length += boost::geometry::length(next_lanelet.centerline().basicLineString());
  }

  return lanelet_sequence_forward;
}

bool RouteHandler::getPreviousShoulderLanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const
{
  Pose front_pose;
  front_pose.position.x = lanelet.centerline2d().front().x();
  front_pose.position.y = lanelet.centerline2d().front().y();
  front_pose.position.z = 0;

  lanelet::ArcCoordinates arc_coordinates;
  const auto & centerline_2d = to2D(lanelet.centerline());

  for (const auto & shoulder_lanelet : shoulder_lanelets_) {
    Pose back_pose;
    back_pose.position.x = shoulder_lanelet.centerline2d().back().x();
    back_pose.position.y = shoulder_lanelet.centerline2d().back().y();
    back_pose.position.z = 0;
    if (
      std::hypot(
        front_pose.position.x - back_pose.position.x,
        front_pose.position.y - back_pose.position.y) < 5)
    {
      *prev_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!exists(shoulder_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousShoulderLanelet(current_lanelet, &prev_lanelet)) {
      break;
    }

    lanelet_sequence_backward.insert(lanelet_sequence_backward.begin(), prev_lanelet);
    current_lanelet = prev_lanelet;
    length += boost::geometry::length(prev_lanelet.centerline().basicLineString());
  }

  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & current_pose,
  const double backward_distance, const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  lanelet::ConstLanelets lanelet_sequence_backward;
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(shoulder_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet_sequence_forward = getShoulderLaneletSequenceAfter(lanelet, forward_distance);

  const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
  if (arc_coordinate.length < backward_distance) {
    lanelet_sequence_backward = getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
  }

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());

  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

bool RouteHandler::getClosestLaneletWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  return lanelet::utils::query::getClosestLanelet(route_lanelets_, search_pose, closest_lanelet);
}

bool RouteHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const
{
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }
  lanelet::ConstLanelets following_lanelets = routing_graph_ptr_->following(lanelet);
  for (const auto & llt : following_lanelets) {
    if (exists(route_lanelets_, llt)) {
      *next_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getPreviousLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const
{
  if (exists(start_lanelets_, lanelet)) {
    return false;
  }
  lanelet::ConstLanelets previous_lanelets = routing_graph_ptr_->previous(lanelet);
  for (const auto & llt : previous_lanelets) {
    if (exists(route_lanelets_, llt)) {
      *prev_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getRightLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet) const
{
  auto opt_right_lanelet = routing_graph_ptr_->right(lanelet);
  if (!!opt_right_lanelet) {
    *right_lanelet = opt_right_lanelet.get();
    return exists(route_lanelets_, *right_lanelet);
  } else {
    return false;
  }
}
bool RouteHandler::getLeftLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet) const
{
  auto opt_left_lanelet = routing_graph_ptr_->left(lanelet);
  if (!!opt_left_lanelet) {
    *left_lanelet = opt_left_lanelet.get();
    return exists(route_lanelets_, *left_lanelet);
  } else {
    return false;
  }
}

bool RouteHandler::getLaneChangeTarget(
  const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const
{
  for (auto it = lanelets.begin(); it != lanelets.end(); ++it) {
    const auto lanelet = *it;

    int num = getNumLaneToPreferredLane(lanelet);
    if (num == 0) {
      continue;
    }

    if (num < 0) {
      if (!!routing_graph_ptr_->right(lanelet)) {
        auto right_lanelet = routing_graph_ptr_->right(lanelet);
        *target_lanelet = right_lanelet.get();
        return true;
      } else {
        continue;
      }
    }

    if (num > 0) {
      if (!!routing_graph_ptr_->left(lanelet)) {
        auto left_lanelet = routing_graph_ptr_->left(lanelet);
        *target_lanelet = left_lanelet.get();
        return true;
      } else {
        continue;
      }
    }
  }

  *target_lanelet = lanelets.front();
  return false;
}

bool RouteHandler::getPullOverTarget(
  const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const
{
  for (const auto & shoulder_lanelet : lanelets) {
    if (lanelet::utils::isInLanelet(getGoalPose(), shoulder_lanelet, 0.1)) {
      *target_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getPullOutStart(
  const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet,
  const Pose & pose) const
{
  for (const auto & shoulder_lanelet : lanelets) {
    if (lanelet::utils::isInLanelet(pose, shoulder_lanelet, 0.1)) {
      *target_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets RouteHandler::getClosestLaneletSequence(const Pose & pose) const
{
  lanelet::ConstLanelet lanelet;
  lanelet::ConstLanelets empty_lanelets;
  if (!getClosestLaneletWithinRoute(pose, &lanelet)) {
    return empty_lanelets;
  }
  return getLaneletSequence(lanelet);
}

int RouteHandler::getNumLaneToPreferredLane(const lanelet::ConstLanelet & lanelet) const
{
  int num = 0;
  if (exists(preferred_lanelets_, lanelet)) {
    return num;
  }
  const auto & right_lanes =
    lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
  for (const auto & right : right_lanes) {
    num--;
    if (exists(preferred_lanelets_, right)) {
      return num;
    }
  }
  const auto & left_lanes = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
  num = 0;
  for (const auto & left : left_lanes) {
    num++;
    if (exists(preferred_lanelets_, left)) {
      return num;
    }
  }

  return 0;  // TODO(Horibe) check if return 0 is appropriate.
}

bool RouteHandler::isInPreferredLane(const PoseStamped & pose) const
{
  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose.pose, &lanelet)) {
    return false;
  }
  return exists(preferred_lanelets_, lanelet);
}
bool RouteHandler::isInTargetLane(
  const PoseStamped & pose, const lanelet::ConstLanelets & target) const
{
  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose.pose, &lanelet)) {
    return false;
  }
  return exists(target, lanelet);
}

PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const Pose & pose,
  const double backward_path_length, const double forward_path_length,
  const BehaviorPathPlannerParameters & parameter) const
{
  PathWithLaneId reference_path;

  if (lanelet_sequence.empty()) {
    return reference_path;
  }

  const auto arc_coordinates = lanelet::utils::getArcCoordinates(lanelet_sequence, pose);
  const double s = arc_coordinates.length;
  const double s_backward = std::max(0., s - backward_path_length);
  double s_forward = s + forward_path_length;

  const double buffer =
    parameter.backward_length_buffer_for_end_of_lane;  // buffer for min_lane_change_length
  const int num_lane_change = std::abs(getNumLaneToPreferredLane(lanelet_sequence.back()));
  const double lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
  const double lane_change_buffer =
    num_lane_change * (parameter.minimum_lane_change_length + buffer);

  if (isDeadEndLanelet(lanelet_sequence.back())) {
    s_forward = std::min(s_forward, lane_length - lane_change_buffer);
  }

  if (isInGoalRouteSection(lanelet_sequence.back())) {
    const auto goal_arc_coordinates =
      lanelet::utils::getArcCoordinates(lanelet_sequence, getGoalPose());
    s_forward = std::min(s_forward, goal_arc_coordinates.length - lane_change_buffer);
  }

  return getCenterLinePath(lanelet_sequence, s_backward, s_forward, true);
}

PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  bool use_exact) const
{
  PathWithLaneId reference_path{};
  double s = 0;

  for (const auto & llt : lanelet_sequence) {
    lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(llt);
    const lanelet::ConstLineString3d centerline = llt.centerline();
    for (size_t i = 0; i < centerline.size(); i++) {
      const lanelet::ConstPoint3d pt = centerline[i];
      lanelet::ConstPoint3d next_pt =
        (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start) {
        if (use_exact) {
          const auto tmp_p = get3DPointFrom2DArcLength(lanelet_sequence, s_start);
          PathPointWithLaneId point{};
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(tmp_p);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        } else {
          PathPointWithLaneId point;
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(pt);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        }
      }

      if (s >= s_start && s <= s_end) {
        PathPointWithLaneId point{};
        point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(pt);
        point.lane_ids.push_back(llt.id());
        point.point.twist.linear.x = limit.speedLimit.value();
        reference_path.points.push_back(point);
      }
      if (s < s_end && s + distance > s_end) {
        if (use_exact) {
          const auto tmp_p = get3DPointFrom2DArcLength(lanelet_sequence, s_end);
          PathPointWithLaneId point{};
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(tmp_p);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        } else {
          PathPointWithLaneId point{};
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(next_pt);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        }
      }
      s += distance;
    }
  }

  reference_path = util::removeOverlappingPoints(reference_path);

  // append a point only when having one point so that yaw calculation would work
  if (reference_path.points.size() == 1) {
    const int lane_id = reference_path.points.front().lane_ids.front();
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
    const auto point = reference_path.points.front().point.pose.position;
    const auto lane_yaw = lanelet::utils::getLaneletAngle(lanelet, point);
    PathPointWithLaneId path_point{};
    path_point.lane_ids.push_back(lane_id);
    constexpr double ds{0.1};
    path_point.point.pose.position.x = point.x + ds * std::cos(lane_yaw);
    path_point.point.pose.position.y = point.y + ds * std::sin(lane_yaw);
    path_point.point.pose.position.z = point.z;
    reference_path.points.push_back(path_point);
  }

  // set angle
  for (size_t i = 0; i < reference_path.points.size(); i++) {
    double angle{0.0};
    auto & pt = reference_path.points.at(i);
    if (i + 1 < reference_path.points.size()) {
      auto next_pt = reference_path.points.at(i + 1);
      angle = std::atan2(
        next_pt.point.pose.position.y - pt.point.pose.position.y,
        next_pt.point.pose.position.x - pt.point.pose.position.x);
    } else if (i != 0) {
      auto prev_pt = reference_path.points.at(i - 1);
      auto pt = reference_path.points.at(i);
      angle = std::atan2(
        pt.point.pose.position.y - prev_pt.point.pose.position.y,
        pt.point.pose.position.x - prev_pt.point.pose.position.x);
    }
    tf2::Quaternion yaw_quat{};
    yaw_quat.setRPY(0, 0, angle);
    pt.point.pose.orientation = tf2::toMsg(yaw_quat);
  }

  return reference_path;
}

PathWithLaneId RouteHandler::setDecelerationVelocity(
  const PathWithLaneId & input,
  const lanelet::ConstLanelets & lanelet_sequence, const double lane_change_prepare_duration,
  const double lane_change_buffer) const
{
  auto reference_path = input;
  if (
    isDeadEndLanelet(lanelet_sequence.back()) &&
    lane_change_prepare_duration > std::numeric_limits<double>::epsilon())
  {
    for (auto & point : reference_path.points) {
      const double lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
      const auto arclength = lanelet::utils::getArcCoordinates(lanelet_sequence, point.point.pose);
      const double distance_to_end =
        std::max(0.0, lane_length - lane_change_buffer - arclength.length);
      point.point.twist.linear.x =
        std::min(point.point.twist.linear.x, (distance_to_end / lane_change_prepare_duration));
    }
  }
  return reference_path;
}

PathWithLaneId RouteHandler::setDecelerationVelocity(
  const PathWithLaneId & input, const lanelet::ConstLanelets & lanelet_sequence,
  const double distance_after_pullover, const double pullover_distance_min,
  const double distance_before_pull_over, const double deceleration_interval, Pose goal_pose) const
{
  auto reference_path = input;
  const auto pullover_buffer =
    distance_after_pullover + pullover_distance_min + distance_before_pull_over;
  const auto arclength_goal_pose =
    lanelet::utils::getArcCoordinates(lanelet_sequence, goal_pose).length;
  const auto arclength_pull_over_start = arclength_goal_pose - pullover_buffer;
  const auto arclength_path_front =
    lanelet::utils::getArcCoordinates(lanelet_sequence, reference_path.points.front().point.pose)
    .length;

  if (
    isDeadEndLanelet(lanelet_sequence.back()) &&
    pullover_distance_min > std::numeric_limits<double>::epsilon())
  {
    for (auto & point : reference_path.points) {
      const auto arclength =
        lanelet::utils::getArcCoordinates(lanelet_sequence, point.point.pose).length;
      const double distance_to_pull_over_start =
        std::max(0.0, arclength_pull_over_start - arclength);
      point.point.twist.linear.x = std::min(
        point.point.twist.linear.x,
        (distance_to_pull_over_start / deceleration_interval) * point.point.twist.linear.x);
    }
  }

  double distance_to_pull_over_start =
    std::max(0.0, arclength_pull_over_start - arclength_path_front);
  const auto stop_point = util::insertStopPoint(distance_to_pull_over_start, &reference_path);

  return reference_path;
}

PathWithLaneId RouteHandler::updatePathTwist(const PathWithLaneId & path) const
{
  PathWithLaneId updated_path = path;
  for (auto & point : updated_path.points) {
    const auto id = point.lane_ids.at(0);
    const auto llt = lanelet_map_ptr_->laneletLayer.get(id);
    lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(llt);
    point.point.twist.linear.x = limit.speedLimit.value();
  }
  return updated_path;
}

lanelet::ConstLanelets RouteHandler::getLaneChangeTargetLanes(const Pose & pose) const
{
  lanelet::ConstLanelet lanelet;
  lanelet::ConstLanelets target_lanelets;
  if (!getClosestLaneletWithinRoute(pose, &lanelet)) {
    return target_lanelets;
  }

  int num = getNumLaneToPreferredLane(lanelet);
  if (num < 0) {
    auto right_lanelet = (!!routing_graph_ptr_->right(lanelet)) ?
      routing_graph_ptr_->right(lanelet) :
      routing_graph_ptr_->adjacentRight(lanelet);
    target_lanelets = getLaneletSequence(right_lanelet.get());
  }
  if (num > 0) {
    auto left_lanelet = (!!routing_graph_ptr_->left(lanelet)) ?
      routing_graph_ptr_->left(lanelet) :
      routing_graph_ptr_->adjacentLeft(lanelet);
    target_lanelets = getLaneletSequence(left_lanelet.get());
  }
  return target_lanelets;
}

double RouteHandler::getLaneChangeableDistance(
  const Pose & current_pose, const LaneChangeDirection & direction) const
{
  lanelet::ConstLanelet current_lane;
  if (!getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    return 0;
  }

  // get lanelets after current lane
  auto lanelet_sequence = getLaneletSequenceAfter(current_lane);
  lanelet_sequence.insert(lanelet_sequence.begin(), current_lane);

  double accumulated_distance = 0;
  for (const auto & lane : lanelet_sequence) {
    lanelet::ConstLanelet target_lane;
    if (direction == LaneChangeDirection::RIGHT) {
      if (!getRightLaneletWithinRoute(lane, &target_lane)) {
        break;
      }
    }
    if (direction == LaneChangeDirection::LEFT) {
      if (!getLeftLaneletWithinRoute(lane, &target_lane)) {
        break;
      }
    }
    double lane_length = lanelet::utils::getLaneletLength3d(lane);

    // overwrite  goal because lane change must be finished before reaching goal
    if (isInGoalRouteSection(lane)) {
      const auto goal_position = lanelet::utils::conversion::toLaneletPoint(getGoalPose().position);
      const auto goal_arc_coordinates = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(lane.centerline()), lanelet::utils::to2D(goal_position).basicPoint());
      lane_length = std::min(goal_arc_coordinates.length, lane_length);
    }

    // subtract distance up to current position for first lane
    if (lane == current_lane) {
      const auto current_position =
        lanelet::utils::conversion::toLaneletPoint(current_pose.position);
      const auto arc_coordinate = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(lane.centerline()),
        lanelet::utils::to2D(current_position).basicPoint());
      lane_length = std::max(lane_length - arc_coordinate.length, 0.0);
    }
    accumulated_distance += lane_length;
  }
  return accumulated_distance;
}

lanelet::ConstLanelets RouteHandler::getCheckTargetLanesFromPath(
  const PathWithLaneId & path, const lanelet::ConstLanelets & target_lanes,
  const double check_length) const
{
  std::vector<int64_t> target_lane_ids;
  target_lane_ids.reserve(target_lanes.size());
  for (const auto & llt : target_lanes) {
    target_lane_ids.push_back(llt.id());
  }

  // find first lanelet in target lanes along path
  int64_t root_lane_id = lanelet::InvalId;
  for (const auto & point : path.points) {
    for (const auto & lane_id : point.lane_ids) {
      if (exists(target_lane_ids, lane_id)) {
        root_lane_id = lane_id;
      }
    }
  }
  // return emtpy lane if failed to find root lane_id
  if (root_lane_id == lanelet::InvalId) {
    return target_lanes;
  }
  lanelet::ConstLanelet root_lanelet;
  for (const auto & llt : target_lanes) {
    if (llt.id() == root_lane_id) {
      root_lanelet = llt;
    }
  }

  const auto sequences = lanelet::utils::query::getPrecedingLaneletSequences(
    routing_graph_ptr_, root_lanelet, check_length);
  lanelet::ConstLanelets check_lanelets;
  for (const auto & sequence : sequences) {
    for (const auto & llt : sequence) {
      if (!lanelet::utils::contains(check_lanelets, llt)) {
        check_lanelets.push_back(llt);
      }
    }
  }
  for (const auto & llt : target_lanes) {
    if (!lanelet::utils::contains(check_lanelets, llt)) {
      check_lanelets.push_back(llt);
    }
  }
  return check_lanelets;
}

lanelet::routing::RoutingGraphContainer RouteHandler::getOverallGraph() const
{
  return *overall_graphs_ptr_;
}

lanelet::routing::RelationType RouteHandler::getRelation(
  const lanelet::ConstLanelet & prev_lane, const lanelet::ConstLanelet & next_lane) const
{
  if (prev_lane == next_lane) {
    return lanelet::routing::RelationType::None;
  }
  const auto & relation = routing_graph_ptr_->routingRelation(prev_lane, next_lane);
  if (relation) {
    return relation.get();
  }

  // check if lane change extends across multiple lanes
  const auto shortest_path = routing_graph_ptr_->shortestPath(prev_lane, next_lane);
  if (shortest_path) {
    auto prev_llt = shortest_path->front();
    for (const auto & llt : shortest_path.get()) {
      if (prev_llt == llt) {
        continue;
      }
      const auto & relation = routing_graph_ptr_->routingRelation(prev_llt, llt);
      if (!relation) {
        continue;
      }
      if (
        relation.get() == lanelet::routing::RelationType::Left ||
        relation.get() == lanelet::routing::RelationType::Right)
      {
        return relation.get();
      }
      prev_llt = llt;
    }
  }

  return lanelet::routing::RelationType::None;
}
lanelet::ConstLanelets RouteHandler::getShoulderLanelets() const {return shoulder_lanelets_;}

}  // namespace behavior_path_planner
