// Copyright 2019 Autoware Foundation
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

#include "mission_planner/lanelet2_impl/route_handler.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mission_planner
{
RouteHandler::RouteHandler(
  const lanelet::LaneletMapConstPtr & lanelet_map_ptr,
  const lanelet::routing::RoutingGraphPtr & routing_graph,
  const lanelet::ConstLanelets & path_lanelets)
{
  setRouteLanelets(lanelet_map_ptr, routing_graph, path_lanelets);
}

void RouteHandler::setRouteLanelets(
  const lanelet::LaneletMapConstPtr & lanelet_map_ptr,
  const lanelet::routing::RoutingGraphPtr & routing_graph,
  const lanelet::ConstLanelets & path_lanelets)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  routing_graph_ptr_ = routing_graph;

  if (!path_lanelets.empty()) {
    auto first_lanelet = path_lanelets.front();
    start_lanelets_ = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, first_lanelet);
    auto last_lanelet = path_lanelets.back();
    goal_lanelets_ = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, last_lanelet);
  }

  // set route lanelets
  std::unordered_set<lanelet::Id> route_lanelets_id;
  std::unordered_set<lanelet::Id> candidate_lanes_id;
  for (const auto & lane : path_lanelets) {
    route_lanelets_id.insert(lane.id());
    const auto right_relations = routing_graph_ptr_->rightRelations(lane);
    for (const auto & right_relation : right_relations) {
      if (right_relation.relationType == lanelet::routing::RelationType::Right) {
        route_lanelets_id.insert(right_relation.lanelet.id());
      } else if (right_relation.relationType == lanelet::routing::RelationType::AdjacentRight) {
        candidate_lanes_id.insert(right_relation.lanelet.id());
      }
    }
    const auto left_relations = routing_graph_ptr_->leftRelations(lane);
    for (const auto & left_relation : left_relations) {
      if (left_relation.relationType == lanelet::routing::RelationType::Left) {
        route_lanelets_id.insert(left_relation.lanelet.id());
      } else if (left_relation.relationType == lanelet::routing::RelationType::AdjacentLeft) {
        candidate_lanes_id.insert(left_relation.lanelet.id());
      }
    }
  }

  //  check if candidates are really part of route
  for (const auto & candidate_id : candidate_lanes_id) {
    lanelet::ConstLanelet lanelet = lanelet_map_ptr_->laneletLayer.get(candidate_id);
    auto previous_lanelets = routing_graph_ptr_->previous(lanelet);
    bool is_connected_to_main_lanes_prev = false;
    bool is_connected_to_candidate_prev = true;
    if (exists(start_lanelets_, lanelet)) {
      is_connected_to_candidate_prev = false;
    }
    while (!previous_lanelets.empty() && is_connected_to_candidate_prev &&
           !is_connected_to_main_lanes_prev) {
      is_connected_to_candidate_prev = false;

      for (const auto & prev_lanelet : previous_lanelets) {
        if (route_lanelets_id.find(prev_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_prev = true;
          break;
        }
        if (exists(start_lanelets_, prev_lanelet)) {
          break;
        }

        if (candidate_lanes_id.find(prev_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_prev = true;
          previous_lanelets = routing_graph_ptr_->previous(prev_lanelet);
          break;
        }
      }
    }

    auto following_lanelets = routing_graph_ptr_->following(lanelet);
    bool is_connected_to_main_lanes_next = false;
    bool is_connected_to_candidate_next = true;
    if (exists(goal_lanelets_, lanelet)) {
      is_connected_to_candidate_next = false;
    }
    while (!following_lanelets.empty() && is_connected_to_candidate_next &&
           !is_connected_to_main_lanes_next) {
      is_connected_to_candidate_next = false;
      for (const auto & next_lanelet : following_lanelets) {
        if (route_lanelets_id.find(next_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_next = true;
          break;
        }
        if (exists(goal_lanelets_, next_lanelet)) {
          break;
        }
        if (candidate_lanes_id.find(next_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_next = true;
          following_lanelets = routing_graph_ptr_->following(next_lanelet);
          break;
        }
      }
    }

    if (is_connected_to_main_lanes_next && is_connected_to_main_lanes_prev) {
      route_lanelets_id.insert(candidate_id);
    }
  }

  for (const auto & id : route_lanelets_id) {
    route_lanelets_.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
}

lanelet::ConstLanelets RouteHandler::getRouteLanelets() const { return route_lanelets_; }

lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  lanelet_sequence_forward.push_back(lanelet);

  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok()) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
  }

  return lanelet_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok()) {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousLaneletWithinRoute(current_lanelet, &prev_lanelet)) {
      break;
    }
    lanelet_sequence_backward.push_back(prev_lanelet);
    current_lanelet = prev_lanelet;
  }

  std::reverse(lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lanelet_sequence;
  lanelet::ConstLanelets lanelet_sequence_backward;
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet_sequence_forward = getLaneletSequenceAfter(lanelet);
  lanelet_sequence_backward = getLaneletSequenceUpTo(lanelet);

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }

  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getPreviousLaneletSequence(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  lanelet::ConstLanelets previous_lanelet_sequence;
  if (lanelet_sequence.empty()) {
    return previous_lanelet_sequence;
  }

  auto first_lane = lanelet_sequence.front();
  if (exists(start_lanelets_, first_lane)) {
    return previous_lanelet_sequence;
  }

  auto right_relations =
    lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, first_lane);
  for (const auto & right : right_relations) {
    previous_lanelet_sequence = getLaneletSequenceUpTo(right);
    if (!previous_lanelet_sequence.empty()) {
      return previous_lanelet_sequence;
    }
  }

  auto left_relations = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, first_lane);
  for (const auto & left : left_relations) {
    previous_lanelet_sequence = getLaneletSequenceUpTo(left);
    if (!previous_lanelet_sequence.empty()) {
      return previous_lanelet_sequence;
    }
  }
  return previous_lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getLaneSequence(const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lane_sequence;
  lanelet::ConstLanelets lane_sequence_up_to = getLaneSequenceUpTo(lanelet);
  lanelet::ConstLanelets lane_sequence_after = getLaneSequenceAfter(lanelet);

  lane_sequence.insert(lane_sequence.end(), lane_sequence_up_to.begin(), lane_sequence_up_to.end());
  lane_sequence.insert(lane_sequence.end(), lane_sequence_after.begin(), lane_sequence_after.end());
  return lane_sequence;
}

lanelet::ConstLanelets RouteHandler::getLaneSequenceUpTo(
  const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lane_sequence_backward;
  if (!exists(route_lanelets_, lanelet)) {
    return lane_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok()) {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousLaneletWithinRoute(current_lanelet, &prev_lanelet)) {
      break;
    }

    lanelet::ConstLanelets current_lanelet_section = getNeighborsWithinRoute(current_lanelet);
    lanelet::ConstLanelets prev_lanelet_section = getNeighborsWithinRoute(prev_lanelet);
    if (!isBijectiveConnection(prev_lanelet_section, current_lanelet_section)) {
      break;
    }
    lane_sequence_backward.push_back(prev_lanelet);
    current_lanelet = prev_lanelet;
  }

  std::reverse(lane_sequence_backward.begin(), lane_sequence_backward.end());
  return lane_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getLaneSequenceAfter(
  const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lane_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lane_sequence_forward;
  }
  lane_sequence_forward.push_back(lanelet);

  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok()) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
      break;
    }

    lanelet::ConstLanelets current_lanelet_section = getNeighborsWithinRoute(current_lanelet);
    lanelet::ConstLanelets next_lanelet_section = getNeighborsWithinRoute(next_lanelet);
    if (!isBijectiveConnection(current_lanelet_section, next_lanelet_section)) {
      break;
    }
    lane_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
  }

  return lane_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getNeighborsWithinRoute(
  const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets neighbor_lanelets =
    lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, lanelet);
  lanelet::ConstLanelets neighbors_within_route;
  for (const auto & llt : neighbor_lanelets) {
    if (exists(route_lanelets_, llt)) {
      neighbors_within_route.push_back(llt);
    }
  }
  return neighbors_within_route;
}

std::vector<lanelet::ConstLanelets> RouteHandler::getLaneSection(
  const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets neighbors = getNeighborsWithinRoute(lanelet);
  std::vector<lanelet::ConstLanelets> lane_section;
  for (const auto & llt : neighbors) {
    lane_section.push_back(getLaneSequence(llt));
  }
  return lane_section;
}

bool RouteHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const
{
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }
  lanelet::ConstLanelets following_lanelets = routing_graph_ptr_->following(lanelet);
  for (const auto & llt : following_lanelets) {
    if (exists(route_lanelets_, llt) && !exists(start_lanelets_, llt)) {
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
    if (exists(route_lanelets_, llt) && !(exists(goal_lanelets_, llt))) {
      *prev_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::isBijectiveConnection(
  const lanelet::ConstLanelets & lanelet_section1,
  const lanelet::ConstLanelets & lanelet_section2) const
{
  if (lanelet_section1.size() != lanelet_section2.size()) {
    return false;
  }

  // check injection
  for (const auto & lanelet : lanelet_section1) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(lanelet, &next_lanelet)) {
      return false;
    }
    if (!exists(lanelet_section2, next_lanelet)) {
      return false;
    }
  }

  // check surjection
  for (const auto & lanelet : lanelet_section2) {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousLaneletWithinRoute(lanelet, &prev_lanelet)) {
      return false;
    }
    if (!exists(lanelet_section1, prev_lanelet)) {
      return false;
    }
  }
  return true;
}

lanelet::ConstLanelets RouteHandler::getNextLaneSequence(
  const lanelet::ConstLanelets & lane_sequence) const
{
  lanelet::ConstLanelets next_lane_sequence;
  if (lane_sequence.empty()) {
    return next_lane_sequence;
  }
  lanelet::ConstLanelet final_lanelet = lane_sequence.back();
  lanelet::ConstLanelet next_lanelet;
  if (!getNextLaneletWithinRoute(final_lanelet, &next_lanelet)) {
    return next_lane_sequence;
  }
  return getLaneSequence(next_lanelet);
}
}  // namespace mission_planner
