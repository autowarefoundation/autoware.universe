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

#include "route_handler/lanelet_route.hpp"

#include "route_handler/lanelet_section.hpp"
#include "route_handler/utils.hpp"

#include <lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <unordered_set>

namespace route_handler
{

std::vector<LaneletSegmentMsg> LaneletRoute::toLaneletSegmentMsgs() const
{
  std::vector<LaneletSegmentMsg> route_sections;

  if (main_path_.empty()) {
    return route_sections;
  }

  route_sections.reserve(main_path_.size());
  for (const auto & main_section : main_path_) {
    const auto & main_llt = main_section.lanelet();
    LaneletSegmentMsg route_section_msg;
    const lanelet::ConstLanelets route_section_lanelets = getNeighborsWithinRoute(main_llt);
    route_section_msg.preferred_primitive.id = main_llt.id();
    route_section_msg.primitives.reserve(route_section_lanelets.size());
    for (const auto & section_llt : route_section_lanelets) {
      LaneletPrimitive p;
      p.id = section_llt.id();
      p.primitive_type = "lane";
      route_section_msg.primitives.push_back(p);
    }
    route_sections.push_back(route_section_msg);
  }

  return route_sections;
}

bool LaneletRoute::isOnFirstSegment(const LaneletPoint & lanelet_point) const
{
  return std::any_of(start_segments_.begin(), start_segments_.end(), [&](const auto & section) {
    return section.contains(lanelet_point);
  });
}

bool LaneletRoute::isOnLastSegment(const LaneletPoint & lanelet_point) const
{
  return std::any_of(goal_segments_.begin(), goal_segments_.end(), [&](const auto & section) {
    return section.contains(lanelet_point);
  });
}

bool LaneletRoute::isPathStraight(const LaneletPath & lanelet_path) const
{
  if (lanelet_path.empty()) {
    return false;
  }

  // check if all sections are directly connected to each other
  auto prev_it = lanelet_path.begin();
  auto curr_it = prev_it + 1;
  while (curr_it != lanelet_path.end()) {
    const lanelet::ConstLanelets following_lanelets =
      routing_graph_ptr_->following(prev_it->lanelet());
    if (!lanelet::utils::contains(following_lanelets, curr_it->lanelet())) {
      return false;  // the next lanelet in the path does not follow current one
    }
  }
  return true;
}

double LaneletRoute::getRemainingBackwardLengthWithinRoute(
  const LaneletPoint & lanelet_point, const double max_search_distance) const
{
  return getStraightPathUpTo(lanelet_point, max_search_distance).length();
}

double LaneletRoute::getRemainingForwardLengthWithinRoute(
  const LaneletPoint & lanelet_point, const double max_search_distance) const
{
  return getStraightPathFrom(lanelet_point, max_search_distance).length();
}

std::optional<double> LaneletRoute::getRouteArcLength(const LaneletPoint & lanelet_point) const
{
  if (main_path_.empty()) {
    return {};
  }

  // check if point is already on the main path
  auto optional_route_arc_length = main_path_.getPathArcLength(lanelet_point);
  if (optional_route_arc_length) {
    return optional_route_arc_length;
  }

  // find preferred neighbor
  auto optional_right_lane = getRightLaneletWithinRoute(lanelet_point.lanelet());
  while (optional_right_lane) {
    auto projected_point =
      LaneletPoint::fromProjection(*optional_right_lane, lanelet_point.toBasicPoint2d());
    optional_route_arc_length = main_path_.getPathArcLength(projected_point);
    if (optional_route_arc_length) {
      return optional_route_arc_length;
    }
    optional_right_lane = getRightLaneletWithinRoute(*optional_right_lane);
  }

  auto optional_left_lane = getLeftLaneletWithinRoute(lanelet_point.lanelet());
  while (optional_left_lane) {
    auto projected_point =
      LaneletPoint::fromProjection(*optional_left_lane, lanelet_point.toBasicPoint2d());
    optional_route_arc_length = main_path_.getPathArcLength(projected_point);
    if (optional_route_arc_length) {
      return optional_route_arc_length;
    }
    optional_left_lane = getRightLaneletWithinRoute(*optional_left_lane);
  }

  // the point is outside the route
  return {};
}

LaneletPath LaneletRoute::getStraightPath(
  const LaneletPoint & lanelet_point, const double backward_distance, const double forward_distance,
  const bool within_route, const OverlapRemovalStrategy overlap_removal_strategy) const
{
  if (within_route && !lanelet::utils::contains(route_lanelets_, lanelet_point.lanelet())) {
    return {};
  }

  const LaneletPath lanelet_path_backward = getStraightPathUpTo(
    lanelet_point, backward_distance, within_route, OverlapRemovalStrategy::DO_NOTHING);
  const LaneletPath lanelet_path_forward = getStraightPathFrom(
    lanelet_point, forward_distance, within_route, OverlapRemovalStrategy::DO_NOTHING);

  LaneletPath concatenated_path = LaneletPath::concatenate(
    lanelet_path_backward, lanelet_path_forward, OverlapRemovalStrategy::DO_NOTHING);

  if (!within_route) {
    // fix eventual overlapping issues
    concatenated_path = concatenated_path.fixOverlap(overlap_removal_strategy);
  }

  return concatenated_path;
}

LaneletPath LaneletRoute::extendPath(
  const LaneletPath & lanelet_path, const double backward_distance, const double forward_distance,
  const bool within_route = true, const OverlapRemovalStrategy overlap_removal_strategy) const
{
  if (lanelet_path.empty()) {
    return {};
  }

  LaneletPath extended_path = lanelet_path;

  if (backward_distance > 0.) {
    const LaneletPath extended_path_backward = getStraightPathUpTo(
      lanelet_path.getStartPoint(), backward_distance, within_route,
      OverlapRemovalStrategy::DO_NOTHING);
    extended_path = LaneletPath::concatenate(
      extended_path_backward, extended_path, OverlapRemovalStrategy::DO_NOTHING);
  }

  if (forward_distance > 0.) {
    const LaneletPath extended_path_forward = getStraightPathFrom(
      lanelet_path.getGoalPoint(), forward_distance, within_route,
      OverlapRemovalStrategy::DO_NOTHING);
    extended_path = LaneletPath::concatenate(
      extended_path, extended_path_forward, OverlapRemovalStrategy::DO_NOTHING);
  }

  if (!within_route) {
    // fix eventual overlapping issues
    extended_path = extended_path.fixOverlap(overlap_removal_strategy);
  }

  return extended_path;
}

LaneletPath LaneletRoute::changeLastLane(
  const LaneletPath & lanelet_path, const lanelet::ConstLanelet & lane_change_target,
  const bool within_route) const
{
  if (lanelet_path.size() == 0) {
    return {};
  }

  const auto & last_lanelet = lanelet_path.back().lanelet();

  if (within_route && !lanelet::utils::contains(route_lanelets_, lane_change_target)) {
    return {};  // outside route
  }

  // check if target lane is a neighbor of the last lanelet
  auto neighbors = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, last_lanelet);
  if (!lanelet::utils::contains(neighbors, lane_change_target)) {
    return {};  // not a valid lane change
  }

  auto & last_section = lanelet_path.sections().back();

  // project last section on the neighbor lanelet

  std::optional<double> start_arc_length_on_target_lanelet{};
  if (last_section.start_arc_length() != 0.) {
    auto projected_start = LaneletPoint::fromProjection(
      lane_change_target, last_section.getStartPoint().toBasicPoint2d());
    start_arc_length_on_target_lanelet = projected_start.arc_length();
  }
  std::optional<double> end_arc_length_on_target_lanelet{};
  if (last_section.end_arc_length() != last_section.lanelet_length()) {
    auto projected_end =
      LaneletPoint::fromProjection(lane_change_target, last_section.getEndPoint().toBasicPoint2d());
    end_arc_length_on_target_lanelet = projected_end.arc_length();
  }

  auto projected_last_section = LaneletSection{
    lane_change_target, start_arc_length_on_target_lanelet, end_arc_length_on_target_lanelet};

  // replace last section with the new one on the target lane
  LaneletSections sections_with_lane_change;
  sections_with_lane_change.reserve(lanelet_path.size());
  sections_with_lane_change.insert(
    sections_with_lane_change.end(), lanelet_path.begin(), lanelet_path.end() - 1);
  sections_with_lane_change.push_back(projected_last_section);

  return LaneletPath{sections_with_lane_change};
}

LaneletPoint LaneletRoute::getClosestLaneletPointWithinRoute(
  const geometry_msgs::msg::Pose pose) const
{
  lanelet::ConstLanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(route_lanelets_, pose, &closest_lanelet)) {
    return {};
  }
  auto closest_lanelet_point = LaneletPoint::fromProjection(closest_lanelet, pose);
  return closest_lanelet_point;
}

lanelet::ConstLanelets LaneletRoute::getFollowingLanelets(
  const LaneletPoint & lanelet_point, const bool within_route) const
{
  if (within_route && isOnLastSegment(lanelet_point)) {
    return {};  // nothing after goal
  }
  auto candidate_lanelets = routing_graph_ptr_->following(lanelet_point.lanelet());
  if (!within_route) {
    return candidate_lanelets;
  }
  // within route only
  lanelet::ConstLanelets next_lanelets;
  for (const auto & llt : candidate_lanelets) {
    if (lanelet::utils::contains(route_lanelets_, llt)) {
      next_lanelets.push_back(llt);
    }
  }
  return next_lanelets;
}

lanelet::ConstLanelets LaneletRoute::getPreviousLanelets(
  const LaneletPoint & lanelet_point, const bool within_route) const
{
  if (within_route && isOnFirstSegment(lanelet_point)) {
    return {};  // nothing before start
  }
  auto candidate_lanelets = routing_graph_ptr_->previous(lanelet_point.lanelet());
  if (!within_route) {
    return candidate_lanelets;
  }
  // within route only
  lanelet::ConstLanelets prev_lanelets;
  for (const auto & llt : candidate_lanelets) {
    if (lanelet::utils::contains(route_lanelets_, llt)) {
      prev_lanelets.push_back(llt);
    }
  }
  return prev_lanelets;
}

std::optional<lanelet::ConstLanelet> LaneletRoute::getLeftLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet) const
{
  const auto optional_left_lanelet = routing_graph_ptr_->left(lanelet);
  if (!optional_left_lanelet) {
    return {};
  }
  lanelet::ConstLanelet left_lanelet = *optional_left_lanelet;
  if (!lanelet::utils::contains(route_lanelets_, left_lanelet)) {
    return {};
  }
  return {left_lanelet};
}

std::optional<lanelet::ConstLanelet> LaneletRoute::getRightLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet) const
{
  const auto optional_right_lanelet = routing_graph_ptr_->right(lanelet);
  if (!optional_right_lanelet) {
    return {};
  }
  lanelet::ConstLanelet right_lanelet = *optional_right_lanelet;
  if (!lanelet::utils::contains(route_lanelets_, right_lanelet)) {
    return {};
  }
  return {right_lanelet};
}

lanelet::ConstLanelets LaneletRoute::getNeighborsWithinRoute(
  const lanelet::ConstLanelet & lanelet) const
{
  const lanelet::ConstLanelets neighbor_lanelets =
    lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, lanelet);
  lanelet::ConstLanelets neighbors_within_route;
  for (const auto & llt : neighbor_lanelets) {
    if (lanelet::utils::contains(route_lanelets_, llt)) {
      neighbors_within_route.push_back(llt);
    }
  }
  return neighbors_within_route;
}

bool LaneletRoute::isInPreferredLane(const LaneletPoint & point) const
{
  if (main_path_.empty()) {
    return false;
  }

  if (isOnLastSegment(point)) {
    return point.lanelet() == main_path_.back().lanelet();
  }

  if (isOnFirstSegment(point)) {
    return point.lanelet() == main_path_.front().lanelet();
  }

  return std::any_of(main_path_.begin(), main_path_.end(), [&](const auto & preferred_section) {
    return preferred_section.contains(point);
  });
}

std::optional<int> LaneletRoute::getNumLaneChangeToPreferredLane(
  const LaneletPoint & lanelet_point) const
{
  if (isInPreferredLane(lanelet_point)) {
    return {0};
  }

  int num = 0;
  auto optional_right_lane = getRightLaneletWithinRoute(lanelet_point.lanelet());
  while (optional_right_lane) {
    --num;
    auto projected_point =
      LaneletPoint::fromProjection(*optional_right_lane, lanelet_point.toBasicPoint2d());
    if (isInPreferredLane(projected_point)) {
      return {num};
    }
    optional_right_lane = getRightLaneletWithinRoute(*optional_right_lane);
  }

  num = 0;
  auto optional_left_lane = getLeftLaneletWithinRoute(lanelet_point.lanelet());
  while (optional_left_lane) {
    ++num;
    auto projected_point =
      LaneletPoint::fromProjection(*optional_left_lane, lanelet_point.toBasicPoint2d());
    if (isInPreferredLane(projected_point)) {
      return {num};
    }
    optional_left_lane = getLeftLaneletWithinRoute(*optional_left_lane);
  }

  // the point is outside the route
  return {};
}

LaneletPath LaneletRoute::getStraightPathFrom(
  const LaneletPoint & lanelet_point, const double forward_distance, const bool within_route,
  const OverlapRemovalStrategy overlap_removal_strategy) const
{
  if (within_route && !lanelet::utils::contains(route_lanelets_, lanelet_point.lanelet())) {
    return {};
  }

  LaneletSections sections_forward{};
  LaneletPoint curr_point = lanelet_point;
  LaneletSection curr_section = curr_point.forwardSection();
  double remaining_distance = forward_distance;
  bool has_looped = false;
  while (remaining_distance > curr_section.length()) {
    // has enough distance to go through current section

    // detect loop
    if (
      !within_route && std::any_of(
                         sections_forward.begin(), sections_forward.end(),
                         [&](const auto & section) { return section.contains(curr_point); })) {
      has_looped = true;
      break;  // no need to continue any further
    }

    // check if there is any lanelet after
    lanelet::ConstLanelets candidate_lanelets = getFollowingLanelets(curr_point, within_route);
    if (candidate_lanelets.empty()) {
      break;  // cannot go any further
    }

    // select first (no particular reason)
    const auto & next_lanelet = candidate_lanelets.front();

    // move to the next lanelet
    sections_forward.push_back(curr_section);
    remaining_distance -= curr_section.length();

    // continue from the start of the next lanelet
    curr_point = LaneletPoint::startOf(next_lanelet);
    curr_section = curr_point.forwardSection();
  }

  // move forward as far as possible within the last section

  remaining_distance = std::min(remaining_distance, curr_section.length());

  if (within_route && isOnLastSegment(curr_point)) {
    // we need to make sure not to go beyond goal
    auto goal_line_it = std::find_if(
      goal_line_.begin(), goal_line_.end(),
      [&](const auto & goal_point) { return goal_point.lanelet() == curr_point.lanelet(); });
    const double dist_to_goal = std::max(0.0, goal_line_it->arc_length() - curr_point.arc_length());
    remaining_distance = std::min(remaining_distance, dist_to_goal);
  }

  if (!has_looped) {
    LaneletSection remaining_section{
      curr_section.lanelet(), curr_point.arc_length(),
      curr_point.arc_length() + remaining_distance};
    sections_forward.push_back(remaining_section);
  }

  LaneletPath path_forward{sections_forward};

  if (!within_route) {
    // fix eventual overlapping issues
    path_forward = path_forward.fixOverlap(overlap_removal_strategy);
  }

  if (!utils::validatePath(path_forward, routing_graph_ptr_)) {
    return {};  // path is broken
  }

  return path_forward;
}

LaneletPath LaneletRoute::getStraightPathUpTo(
  const LaneletPoint & lanelet_point, const double backward_distance, const bool within_route,
  const OverlapRemovalStrategy overlap_removal_strategy) const
{
  if (within_route && !lanelet::utils::contains(route_lanelets_, lanelet_point.lanelet())) {
    return {};
  }

  LaneletSections sections_backward{};
  LaneletPoint curr_point = lanelet_point;
  LaneletSection curr_section = lanelet_point.backwardSection();
  double remaining_distance = backward_distance;
  bool has_looped = false;
  while (remaining_distance > curr_section.length()) {
    // has enough distance to go through current section

    // detect loop
    if (
      !within_route && std::any_of(
                         sections_backward.begin(), sections_backward.end(),
                         [&](const auto & section) { return section.contains(curr_point); })) {
      has_looped = true;
      break;  // no need to continue any further
    }

    // check if there is any lanelet before
    lanelet::ConstLanelets candidate_lanelets = getPreviousLanelets(curr_point, within_route);
    if (candidate_lanelets.empty()) {
      break;  // cannot go any further
    }

    // select first (no particular reason)
    const auto & prev_lanelet = candidate_lanelets.front();

    // move to the previous lanelet
    sections_backward.push_back(curr_section);
    remaining_distance -= curr_section.length();

    // continue from the end of the previous lanelet
    curr_point = LaneletPoint::endOf(prev_lanelet);
    curr_section = curr_point.backwardSection();
  }

  // move backward as far as possible within the last section

  remaining_distance = std::min(remaining_distance, curr_section.length());

  if (within_route && isOnFirstSegment(curr_point)) {
    // we need to make sure not to go behind start
    auto start_line_it = std::find_if(
      start_line_.begin(), start_line_.end(),
      [&](const auto & start_point) { return start_point.lanelet() == curr_point.lanelet(); });
    const double dist_to_start =
      std::max(0.0, curr_point.arc_length() - start_line_it->arc_length());
    remaining_distance = std::min(remaining_distance, dist_to_start);
  }

  if (!has_looped) {
    LaneletSection remaining_section{
      curr_section.lanelet(), curr_point.arc_length() - remaining_distance,
      curr_point.arc_length()};
    sections_backward.push_back(remaining_section);
  }

  // put back sections in order
  std::reverse(sections_backward.begin(), sections_backward.end());

  LaneletPath path_backward{sections_backward};

  if (!within_route) {
    // fix eventual overlapping issues
    path_backward = path_backward.fixOverlap(overlap_removal_strategy);
  }

  return path_backward;
}

LaneletPath LaneletRoute::getPathFromLanelets(const lanelet::ConstLanelets & lanelets) const
{
  if (lanelets.empty()) {
    return {};
  }

  return getPathFromLanelets(
    lanelets, LaneletPoint::startOf(lanelets.front()), LaneletPoint::endOf(lanelets.back()));
}

LaneletPath LaneletRoute::getPathFromLanelets(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & goal_pose) const
{
  if (lanelets.empty()) {
    return {};
  }

  lanelet::ConstLanelet start_closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, start_pose, &start_closest_lanelet)) {
    return {};
  }
  lanelet::ConstLanelet goal_closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, goal_pose, &goal_closest_lanelet)) {
    return {};
  }

  auto start_point = LaneletPoint::fromProjection(start_closest_lanelet, start_pose);
  auto goal_point = LaneletPoint::fromProjection(goal_closest_lanelet, goal_pose);

  return getPathFromLanelets(lanelets, start_point, goal_point);
}

LaneletPath LaneletRoute::getPathFromLanelets(
  const lanelet::ConstLanelets & lanelets, const LaneletPoint & start_point,
  const LaneletPoint & goal_point) const
{
  LaneletPath path{lanelets, start_point, goal_point};
  if (!utils::validatePath(path, routing_graph_ptr_)) {
    return {};
  }

  return path;
}

}  // namespace route_handler
