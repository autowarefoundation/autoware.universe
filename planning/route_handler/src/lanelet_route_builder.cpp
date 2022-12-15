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

#include "route_handler/lanelet_route_builder.hpp"

#include "route_handler/utils.hpp"

#include <lanelet2_extension/utility/query.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <unordered_set>

namespace route_handler
{

namespace
{

std::string toString(const geometry_msgs::msg::Point & point)
{
  std::stringstream ss;
  ss << "(" << point.x << ", " << point.y << "," << point.z << ")";
  return ss.str();
}

std::string toString(const geometry_msgs::msg::Pose & pose) { return toString(pose.position); }

}  // namespace

LaneletRouteBuilder::LaneletRouteBuilder(
  const lanelet::LaneletMapPtr lanelet_map_ptr,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  routing_graph_ptr_ = routing_graph_ptr;
}

LaneletRoutePtr LaneletRouteBuilder::buildFromStartGoalPose(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose) const
{
  LaneletRoutePtr lanelet_route_ptr = std::make_shared<LaneletRoute>();
  lanelet_route_ptr->routing_graph_ptr_ = routing_graph_ptr_;

  // get shortest lanelet route from start to pose
  const lanelet::Optional<lanelet::routing::Route> optional_route =
    getShortestRoute(start_pose, goal_pose);

  if (!optional_route) {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to find a proper path!"
                 << std::endl
                 << "start pose: " << toString(start_pose) << std::endl
                 << "goal pose: " << toString(goal_pose) << std::endl);
    return {};
  }

  const auto & shortest_path = optional_route->shortestPath();

  if (shortest_path.empty()) {
    RCLCPP_ERROR_STREAM(
      logger_, "Generated an empty path!" << std::endl
                                          << "start pose: " << toString(start_pose) << std::endl
                                          << "goal pose: " << toString(goal_pose) << std::endl);
    return {};
  }

  lanelet::ConstLanelets shortest_path_lanelets;
  shortest_path_lanelets.insert(
    shortest_path_lanelets.begin(), shortest_path.begin(), shortest_path.end());

  auto start_point = LaneletPoint::fromProjection(shortest_path_lanelets.front(), start_pose);
  auto goal_point = LaneletPoint::fromProjection(shortest_path_lanelets.back(), goal_pose);

  if (!updateStartGoalSegments(lanelet_route_ptr.get(), start_point, goal_point)) {
    return {};
  }

  // list all the reachable lanelets within the route
  // Note: this route may be slightly different from the result of getShortestRoute()
  // For example, all neighbors of start/goal lanelet are reachable, yet not present in the
  // lanelet::routing::Route
  lanelet_route_ptr->route_lanelets_ = getRouteLanelets(shortest_path_lanelets);

  if (!updateMainPath(lanelet_route_ptr.get(), start_point, goal_point)) {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to create main path!" << std::endl
                                             << "start pose: " << toString(start_pose) << std::endl
                                             << "goal pose: " << toString(goal_pose) << std::endl);
    return {};
  }

  return lanelet_route_ptr;
}

LaneletRoutePtr LaneletRouteBuilder::buildFromLaneletRouteMsg(
  const LaneletRouteMsg & route_msg) const
{
  LaneletRoutePtr lanelet_route_ptr = std::make_shared<LaneletRoute>();
  lanelet_route_ptr->routing_graph_ptr_ = routing_graph_ptr_;

  if (route_msg.segments.empty()) {
    return {};
  }

  // Note: start pose lanelet may not be the first preferred lanelet
  lanelet::ConstLanelets start_lanelets;
  start_lanelets.reserve(route_msg.segments.front().primitives.size());
  for (const auto & primitive : route_msg.segments.front().primitives) {
    const auto id = primitive.id;
    const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
    start_lanelets.push_back(llt);
  }
  lanelet::ConstLanelet start_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        start_lanelets, route_msg.start_pose, &start_lanelet)) {
    RCLCPP_ERROR(logger_, "Failed to find starting lanelet");
    return {};
  };
  const LaneletPoint start_point =
    LaneletPoint::fromProjection(start_lanelet, route_msg.start_pose.position);

  // But goal pose lanelet is always the last preferred lanelet
  const auto goal_lanelet =
    lanelet_map_ptr_->laneletLayer.get(route_msg.segments.back().preferred_primitive.id);
  const LaneletPoint goal_point =
    LaneletPoint::fromProjection(goal_lanelet, route_msg.goal_pose.position);

  if (!updateStartGoalSegments(lanelet_route_ptr.get(), start_point, goal_point)) {
    return {};
  }

  size_t primitive_size{0};
  for (const auto & route_section : route_msg.segments) {
    primitive_size += route_section.primitives.size();
  }

  lanelet_route_ptr->route_lanelets_.reserve(primitive_size);
  lanelet::ConstLanelets preferred_lanelets;
  for (const auto & route_section : route_msg.segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      lanelet_route_ptr->route_lanelets_.push_back(llt);
      if (id == route_section.preferred_primitive.id) {
        preferred_lanelets.push_back(llt);
      }
    }
  }

  if (preferred_lanelets.empty()) {
    return {};
  }

  LaneletPoint start_point_on_first_preferred_lanelet =
    LaneletPoint::fromProjection(preferred_lanelets.front(), start_point.toBasicPoint2d());
  LaneletPath main_path{preferred_lanelets, start_point_on_first_preferred_lanelet, goal_point};

  if (!utils::validatePath(main_path, routing_graph_ptr_)) {
    return {};  // path is broken
  }

  lanelet_route_ptr->main_path_ = main_path;

  return lanelet_route_ptr;
}

lanelet::Optional<lanelet::routing::Route> LaneletRouteBuilder::getShortestRoute(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose) const
{
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  const lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

  lanelet::Lanelet start_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets, start_pose, &start_lanelet)) {
    RCLCPP_ERROR(
      logger_, "Failed to find lanelet next to start pose: %s", toString(start_pose).c_str());
    return {};
  }
  lanelet::Lanelet goal_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets, goal_pose, &goal_lanelet)) {
    RCLCPP_ERROR(
      logger_, "Failed to find lanelet next to goal pose: %s", toString(start_pose).c_str());
    return {};
  }

  // a "loop" is required in the case the goal is behind the start but on the same or a neighbor
  // lanelet
  bool is_goal_behind_start = false;
  lanelet::ConstLanelets neighbor_lanelets =
    lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, start_lanelet);
  if (start_lanelet == goal_lanelet || lanelet::utils::contains(neighbor_lanelets, goal_lanelet)) {
    // check if goal pose is behind start pose by projecting both on starting lanelet
    LaneletPoint start_point = LaneletPoint::fromProjection(start_lanelet, start_pose);
    LaneletPoint goal_point = LaneletPoint::fromProjection(start_lanelet, goal_pose);
    is_goal_behind_start = start_point.arc_length() > goal_point.arc_length();
  }

  lanelet::Optional<lanelet::routing::Route> optional_route{};
  if (is_goal_behind_start) {
    // list all lanelet following starting lanelet or one of its reachable neighbor
    lanelet::ConstLanelets following_lanelets;
    for (const auto & neighbor : neighbor_lanelets) {
      const lanelet::ConstLanelets next_lanelets = routing_graph_ptr_->following(neighbor);
      following_lanelets.insert(
        following_lanelets.end(), next_lanelets.begin(), next_lanelets.end());
    }

    // find all routes from start to goal that pass by one of the following lanelet
    lanelet::routing::Routes candidate_routes{};
    for (auto & via_lanelet : following_lanelets) {
      lanelet::Optional<lanelet::routing::Route> optional_route =
        routing_graph_ptr_->getRouteVia(start_lanelet, {via_lanelet}, goal_lanelet);
      if (optional_route) {
        candidate_routes.emplace_back(std::move(*optional_route));
      }
    }

    if (!candidate_routes.empty()) {
      // find route with the shortest path
      const auto shortest_route = std::min_element(
        candidate_routes.begin(), candidate_routes.end(),
        [](auto & a, auto & b) { return a.length2d() < b.length2d(); });

      optional_route = {std::move(*shortest_route)};
    }
  } else {
    // get all possible lanes that can be used to reach goal (including all possible lane change)
    optional_route = routing_graph_ptr_->getRoute(start_lanelet, goal_lanelet);
  }

  return optional_route;
}

bool LaneletRouteBuilder::updateStartGoalSegments(
  LaneletRoute * lanelet_route_ptr, const LaneletPoint & start_point,
  const LaneletPoint & goal_point) const
{
  lanelet_route_ptr->start_segments_.clear();
  lanelet_route_ptr->goal_segments_.clear();
  lanelet_route_ptr->start_line_.clear();
  lanelet_route_ptr->goal_line_.clear();

  lanelet::ConstLanelets start_lanelets =
    lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, start_point.lanelet());
  lanelet::ConstLanelets goal_lanelets =
    lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, goal_point.lanelet());

  lanelet_route_ptr->start_segments_.reserve(start_lanelets.size());
  lanelet_route_ptr->goal_segments_.reserve(goal_lanelets.size());
  lanelet_route_ptr->start_line_.reserve(start_lanelets.size());
  lanelet_route_ptr->goal_line_.reserve(goal_lanelets.size());

  if (!lanelet::utils::contains(start_lanelets, goal_point.lanelet())) {
    // simple case: start and goal lanelets are disjoints
    // start and goal segments are expanded on the full lanelets
    for (const auto & llt : start_lanelets) {
      lanelet_route_ptr->start_segments_.push_back(LaneletSection{llt});
      lanelet_route_ptr->start_line_.push_back(
        LaneletPoint::fromProjection(llt, start_point.toBasicPoint2d()));
    }
    for (const auto & llt : goal_lanelets) {
      lanelet_route_ptr->goal_segments_.push_back(LaneletSection{llt});
      lanelet_route_ptr->goal_line_.push_back(
        LaneletPoint::fromProjection(llt, goal_point.toBasicPoint2d()));
    }
    return true;
  }

  // start_lanelets == goal_lanelets

  // check if goal is behind start, on the same lanelet or a neighbor lanelet
  const LaneletPoint projected_goal_on_start_lanelet =
    LaneletPoint::fromProjection(start_point.lanelet(), goal_point.toBasicPoint2d());
  const LaneletPoint projected_start_on_goal_lanelet =
    LaneletPoint::fromProjection(goal_point.lanelet(), start_point.toBasicPoint2d());

  // unless the start and goal pose are very close, or the lanelet shape is totally weird, goal
  // should be clearly behind start point, whichever lanelet is used as reference.
  bool is_goal_behind_start_on_start_lanelet =
    projected_goal_on_start_lanelet.arc_length() < start_point.arc_length();
  bool is_goal_behind_start_on_goal_lanelet =
    goal_point.arc_length() < projected_start_on_goal_lanelet.arc_length();
  if (is_goal_behind_start_on_start_lanelet ^ is_goal_behind_start_on_goal_lanelet) {
    RCLCPP_ERROR(
      logger_,
      "Cannot really tell if goal pose is behind start pose or not. Are goal and start points "
      "too close?");
    return false;
  }

  bool is_goal_behind_start =
    is_goal_behind_start_on_start_lanelet && is_goal_behind_start_on_goal_lanelet;

  if (!is_goal_behind_start) {
    // simple case: start and goal segments are the same and expand on the full lanes
    for (const auto & llt : start_lanelets) {
      lanelet_route_ptr->start_segments_.push_back(LaneletSection{llt});
      lanelet_route_ptr->start_line_.push_back(
        LaneletPoint::fromProjection(llt, start_point.toBasicPoint2d()));
    }
    for (const auto & llt : goal_lanelets) {
      lanelet_route_ptr->goal_segments_.push_back(LaneletSection{llt});
      lanelet_route_ptr->goal_line_.push_back(
        LaneletPoint::fromProjection(llt, goal_point.toBasicPoint2d()));
    }
    return true;
  }

  // complex case: goal is behind start
  // the start/goal lanes must be split so that the two segments are disjoint

  // reminder: start_lanelets == goal_lanelets
  for (const auto & llt : start_lanelets) {
    auto projected_start_point = LaneletPoint::fromProjection(llt, start_point.toBasicPoint2d());
    auto projected_goal_point = LaneletPoint::fromProjection(llt, goal_point.toBasicPoint2d());

    // goal should be clearly behind start, whichever lanelet is used as reference
    if (projected_goal_point.arc_length() > projected_start_point.arc_length()) {
      RCLCPP_ERROR(
        logger_,
        "Cannot really tell if goal pose is behind start pose or not. Are goal and start points "
        "too close?");
      return false;
    }

    // split the lanelet in the middle of projected start and goal pose
    LaneletPoint split_point{
      llt, (projected_start_point.arc_length() + projected_goal_point.arc_length()) / 2.0};
    LaneletSection section_before;
    LaneletSection section_after;
    LaneletSection full_section = LaneletSection{llt};
    if (!full_section.split(split_point, &section_before, &section_after)) {
      RCLCPP_ERROR(logger_, "failed to split the start/goal lanes");
      return false;
    };

    lanelet_route_ptr->start_segments_.push_back(section_after);
    lanelet_route_ptr->goal_segments_.push_back(section_before);
    lanelet_route_ptr->start_line_.push_back(projected_start_point);
    lanelet_route_ptr->goal_line_.push_back(projected_goal_point);
  }

  return true;
}

lanelet::ConstLanelets LaneletRouteBuilder::getRouteLanelets(
  const lanelet::ConstLanelets & path_lanelets) const
{
  // set route lanelets
  std::unordered_set<lanelet::Id> route_lanelets_id;
  std::unordered_set<lanelet::Id> candidate_lanes_id;
  for (const auto & lane : path_lanelets) {
    route_lanelets_id.insert(lane.id());

    for (const auto & llt : routing_graph_ptr_->rights(lane)) {
      route_lanelets_id.insert(llt.id());
    }
    for (const auto & llt : routing_graph_ptr_->adjacentRights(lane)) {
      candidate_lanes_id.insert(llt.id());
    }

    for (const auto & llt : routing_graph_ptr_->lefts(lane)) {
      route_lanelets_id.insert(llt.id());
    }
    for (const auto & llt : routing_graph_ptr_->adjacentLefts(lane)) {
      candidate_lanes_id.insert(llt.id());
    }
  }

  //  check if candidates are really part of route
  for (const auto & candidate_id : candidate_lanes_id) {
    lanelet::ConstLanelet lanelet = lanelet_map_ptr_->laneletLayer.get(candidate_id);

    std::unordered_set<lanelet::Id> explored_previous_id{candidate_id};
    lanelet::ConstLanelets previous_lanelets = routing_graph_ptr_->previous(lanelet);
    lanelet::ConstLanelets next_previous_lanelets;
    bool is_connected_to_main_lanes_prev = false;
    while (!previous_lanelets.empty() && !is_connected_to_main_lanes_prev) {
      next_previous_lanelets.clear();
      for (const auto & prev_lanelet : previous_lanelets) {
        if (explored_previous_id.find(prev_lanelet.id()) != explored_previous_id.end()) {
          continue;
        }
        explored_previous_id.insert(prev_lanelet.id());

        if (route_lanelets_id.find(prev_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_prev = true;
          break;
        }

        if (candidate_lanes_id.find(prev_lanelet.id()) != candidate_lanes_id.end()) {
          for (const auto & llt : routing_graph_ptr_->previous(prev_lanelet)) {
            if (explored_previous_id.find(llt.id()) == explored_previous_id.end()) {
              next_previous_lanelets.push_back(llt);
            }
          }
        }
      }
      previous_lanelets = next_previous_lanelets;
    }

    std::unordered_set<lanelet::Id> explored_following_id{candidate_id};
    lanelet::ConstLanelets following_lanelets = routing_graph_ptr_->following(lanelet);
    lanelet::ConstLanelets next_following_lanelets;
    bool is_connected_to_main_lanes_next = false;
    while (!following_lanelets.empty() && !is_connected_to_main_lanes_next) {
      next_following_lanelets.clear();
      for (const auto & next_lanelet : following_lanelets) {
        if (explored_following_id.find(next_lanelet.id()) != explored_following_id.end()) {
          continue;
        }
        explored_following_id.insert(next_lanelet.id());

        if (route_lanelets_id.find(next_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_next = true;
          break;
        }

        if (candidate_lanes_id.find(next_lanelet.id()) != candidate_lanes_id.end()) {
          for (const auto & llt : routing_graph_ptr_->previous(next_lanelet)) {
            if (explored_following_id.find(llt.id()) == explored_following_id.end()) {
              next_following_lanelets.push_back(llt);
            }
          }
          following_lanelets = routing_graph_ptr_->following(next_lanelet);
          break;
        }
      }
      following_lanelets = next_following_lanelets;
    }

    if (is_connected_to_main_lanes_next && is_connected_to_main_lanes_prev) {
      route_lanelets_id.insert(candidate_id);
    }
  }

  lanelet::ConstLanelets route_lanelets;
  route_lanelets.reserve(route_lanelets_id.size());
  for (const auto & id : route_lanelets_id) {
    route_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }

  return route_lanelets;
}

bool LaneletRouteBuilder::updateMainPath(
  LaneletRoute * lanelet_route_ptr, const LaneletPoint & start_point [[maybe_unused]],
  const LaneletPoint & goal_point) const
{
  // main path is built by exploring the route straight backward from the goal point
  // whenever a dead end is reached (can't go back any further from the current lanelet), we change
  // to a neighbor lane and move straight backward again we repeat the process until start point is
  // reached

  const double backward_distance = std::numeric_limits<double>::infinity();

  LaneletSections main_sections{};

  // first get backward as much as we can
  LaneletPath straight_path = lanelet_route_ptr->getStraightPathUpTo(goal_point, backward_distance);
  LaneletPoint curr_point = straight_path.getStartPoint();
  main_sections.insert(main_sections.begin(), straight_path.begin(), straight_path.end());

outer:
  while (!lanelet_route_ptr->isOnFirstSegment(curr_point)) {
    // as long as we have not reached the starting segment, lane change to a neighbor lane and keep
    // moving straight backward Note: by construction, curr_point points to the beginning of a
    // lanelet

    // check right neighbors
    auto right_relations =
      lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, curr_point.lanelet());
    for (const auto & right : right_relations) {
      // try with a lane change to a right neighbor
      const LaneletPoint right_point = LaneletPoint::endOf(right);
      straight_path = lanelet_route_ptr->getStraightPathUpTo(right_point, backward_distance);
      if (straight_path.size() < 2) {  // path contains at least the point
        continue;
      }
      // all path sections except the last are preferred lanelets
      main_sections.insert(main_sections.begin(), straight_path.begin(), straight_path.end() - 1);
      curr_point = straight_path.getStartPoint();
      goto outer;  // keep exploring down this path
    }

    // check left neighbors
    auto left_relations =
      lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, curr_point.lanelet());
    for (const auto & left : left_relations) {
      // try with a lane change to a left neighbor
      const LaneletPoint left_point = LaneletPoint::endOf(left);
      straight_path = lanelet_route_ptr->getStraightPathUpTo(left_point, backward_distance);
      if (straight_path.size() < 2) {  // path contains at least the point
        continue;
      }
      // all path sections except the last are preferred lanelets
      main_sections.insert(main_sections.begin(), straight_path.begin(), straight_path.end() - 1);
      curr_point = straight_path.getStartPoint();
      goto outer;  // keep exploring down this path
    }

    // reached a dead end which is not start. Should not happen
    RCLCPP_ERROR(logger_, "Reached a dead end while building the main path.");
    return false;
  }

  LaneletPath main_path{main_sections};
  if (!utils::validatePath(main_path, lanelet_route_ptr->routing_graph_ptr_)) {
    RCLCPP_ERROR(logger_, "Has built a broken main path");
    return false;
  }

  lanelet_route_ptr->main_path_ = main_path;

  return true;
}

}  // namespace route_handler
