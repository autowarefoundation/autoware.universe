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

#include "route_handler/route_handler.hpp"
#include "route_handler/lanelet_route_builder.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/route_checker.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

// TODO(vrichard) reorder functions based on route_handler.hpp order

namespace
{
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LaneletPrimitive;
using geometry_msgs::msg::Pose;
using lanelet::utils::to2D;

[[maybe_unused]] bool exists(const std::vector<LaneletPrimitive> & primitives, const int64_t & id)
{
  return std::any_of(
    primitives.begin(), primitives.end(), [&](const auto & p) { return p.id == id; });
}

template <typename T>
bool exists(const std::vector<T> & vector, const T & item)
{
  return std::find(vector.begin(), vector.end(), item) != vector.end();
}

// TODO(vrichard) simplify the function as we don't need lanelet_sequence anymore
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
      const double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated_distance2d + distance2d > s) {
        const double ratio = (s - accumulated_distance2d) / distance2d;
        const auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
        return lanelet::ConstPoint3d{
          lanelet::InvalId, interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z()};
      }
      accumulated_distance2d += distance2d;
      prev_pt = pt;
    }
  }
  return lanelet::ConstPoint3d{};
}

PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path)
{
  PathWithLaneId filtered_path;
  for (const auto & pt : input_path.points) {
    if (filtered_path.points.empty()) {
      filtered_path.points.push_back(pt);
      continue;
    }

    constexpr double min_dist = 0.001;
    if (
      tier4_autoware_utils::calcDistance3d(filtered_path.points.back().point, pt.point) <
      min_dist) {
      filtered_path.points.back().lane_ids.push_back(pt.lane_ids.front());
      filtered_path.points.back().point.longitudinal_velocity_mps = std::min(
        pt.point.longitudinal_velocity_mps,
        filtered_path.points.back().point.longitudinal_velocity_mps);
    } else {
      filtered_path.points.push_back(pt);
    }
  }
  filtered_path.left_bound = input_path.left_bound;
  filtered_path.right_bound = input_path.right_bound;
  return filtered_path;
}

[[maybe_unused]] std::string toString(const geometry_msgs::msg::Pose & pose)
{
  std::stringstream ss;
  ss << "(" << pose.position.x << ", " << pose.position.y << "," << pose.position.z << ")";
  return ss.str();
}

}  // namespace

namespace route_handler
{

RouteHandler::RouteHandler(const HADMapBin & map_msg) { setMap(map_msg); }

void RouteHandler::setMap(const HADMapBin & map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  const lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  const lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  const lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);

  start_pose_ = geometry_msgs::msg::Pose{};
  goal_pose_ = geometry_msgs::msg::Pose{};
  lanelet_route_ptr_ = nullptr;

  is_map_msg_ready_ = true;
  is_handler_ready_ = false;

  buildRouteFromMsg();
}

bool RouteHandler::setRoute(const LaneletRouteMsg & route_msg)
{
  route_msg_ = route_msg;
  
  start_pose_ = geometry_msgs::msg::Pose{};
  goal_pose_ = geometry_msgs::msg::Pose{};
  lanelet_route_ptr_ = nullptr;

  is_route_msg_ready_ = true;
  is_handler_ready_ = false;
  
  buildRouteFromMsg();

  return true;
}

void RouteHandler::buildRouteFromMsg()
{
  if (!is_route_msg_ready_ || !is_map_msg_ready_) {
    return;
  }

  const bool is_route_valid = lanelet::utils::route::isRouteValid(route_msg_, lanelet_map_ptr_);
  if (!is_route_valid) {
    return;
  }

  LaneletRouteBuilder builder(lanelet_map_ptr_, routing_graph_ptr_);
  LaneletRoutePtr lanelet_route_ptr = builder.buildFromLaneletRouteMsg(route_msg_);
  if (!lanelet_route_ptr) {
    return;
  }

  start_pose_ = route_msg_.start_pose;
  goal_pose_ = route_msg_.goal_pose;
  lanelet_route_ptr_ = lanelet_route_ptr;

  is_handler_ready_ = true;
}

bool RouteHandler::buildRoute(const Pose & start, const Pose & goal)
{
  if (!is_map_msg_ready_) {
    return false;
  }

  LaneletRouteBuilder builder(lanelet_map_ptr_, routing_graph_ptr_);
  LaneletRoutePtr lanelet_route_ptr = builder.buildFromStartGoalPose(start, goal);
  if (!lanelet_route_ptr) {
    return false;
  }

  start_pose_ = start;
  goal_pose_ = goal;
  lanelet_route_ptr_ = lanelet_route_ptr;

  is_handler_ready_ = true;
  return true;
}


// lanelet::Id RouteHandler::getGoalLaneId() const
// {
//   if (route_msg_.segments.empty()) {
//     return lanelet::InvalId;
//   }

//   return route_msg_.segments.back().preferred_primitive.id;
// }


bool RouteHandler::isRouteMsgValid(const LaneletRouteMsg & route_msg) const
{
  if (route_msg.segments.empty()) {
    return true;
  }

  if (!is_map_msg_ready_) {
    return false; // can't tell
  }
  
  // The simplest way is to try and build the route from it
  LaneletRouteBuilder builder(lanelet_map_ptr_, routing_graph_ptr_);
  LaneletRoutePtr lanelet_route_ptr = builder.buildFromLaneletRouteMsg(route_msg);
  return !!lanelet_route_ptr;
}

lanelet::ConstLanelet RouteHandler::getLaneletsFromId(const lanelet::Id id) const
{
  if (!is_map_msg_ready_) {
    return lanelet::ConstLanelet{};
  }
  return lanelet_map_ptr_->laneletLayer.get(id);
}

lanelet::ConstLanelets RouteHandler::getLaneletsFromIds(const lanelet::Ids & ids) const
{
  if (!is_map_msg_ready_) {
    return lanelet::ConstLanelets{};
  }
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(ids.size());
  for (const auto & id : ids) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  return lanelets;
}


lanelet::ConstPolygon3d RouteHandler::getIntersectionAreaById(const lanelet::Id id) const
{
  if (!is_map_msg_ready_) {
    return {};
  }
  return lanelet_map_ptr_->polygonLayer.get(id);
}

// bool RouteHandler::isDeadEndLanelet(const LaneletPoint & lanelet_point) const
// {
//   if (!is_handler_ready_) {
//     return {};
//   }
//   return !lanelet_route_ptr_->getFollowingLanelet(lanelet_point);
// }

// LaneletPath RouteHandler::getStraightPath(
//   const LaneletPoint & lanelet_point, const double backward_distance,
//   const double forward_distance) const
// {
//   if (!is_handler_ready_) {
//     return {};
//   }

//   LaneletPath lanelet_path =
//     lanelet_route_ptr_->getStraightPath(lanelet_point, backward_distance, forward_distance, true);

//   return lanelet_path;
// }

bool RouteHandler::getFollowingShoulderLanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * following_lanelet) const
{
  Pose back_pose;
  back_pose.position.x = lanelet.centerline2d().back().x();
  back_pose.position.y = lanelet.centerline2d().back().y();
  back_pose.position.z = 0;

  for (const auto & shoulder_lanelet : shoulder_lanelets_) {
    Pose front_pose;
    front_pose.position.x = shoulder_lanelet.centerline2d().front().x();
    front_pose.position.y = shoulder_lanelet.centerline2d().front().y();
    front_pose.position.z = 0;
    if (
      std::hypot(
        front_pose.position.x - back_pose.position.x,
        front_pose.position.y - back_pose.position.y) < 5) {
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
    length +=
      static_cast<double>(boost::geometry::length(next_lanelet.centerline().basicLineString()));
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

  for (const auto & shoulder_lanelet : shoulder_lanelets_) {
    Pose back_pose;
    back_pose.position.x = shoulder_lanelet.centerline2d().back().x();
    back_pose.position.y = shoulder_lanelet.centerline2d().back().y();
    back_pose.position.z = 0;
    if (
      std::hypot(
        front_pose.position.x - back_pose.position.x,
        front_pose.position.y - back_pose.position.y) < 5) {
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
    length +=
      static_cast<double>(boost::geometry::length(prev_lanelet.centerline().basicLineString()));
  }

  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & pose, const double backward_distance,
  const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  if (!exists(shoulder_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getShoulderLaneletSequenceAfter(lanelet, forward_distance);
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, pose);
    if (arc_coordinate.length < backward_distance) {
      return getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
    }
    return lanelet::ConstLanelets{};
  });

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

bool RouteHandler::getPullOverTarget(
  const lanelet::ConstLanelets & lanelets, const Pose & goal_pose,
  lanelet::ConstLanelet * target_lanelet)
{
  for (const auto & shoulder_lanelet : lanelets) {
    if (lanelet::utils::isInLanelet(goal_pose, shoulder_lanelet, 0.1)) {
      *target_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getPullOutStartLane(
  const lanelet::ConstLanelets & lanelets, const Pose & pose, const double vehicle_width,
  lanelet::ConstLanelet * target_lanelet)
{
  for (const auto & shoulder_lanelet : lanelets) {
    if (lanelet::utils::isInLanelet(pose, shoulder_lanelet, vehicle_width / 2.0)) {
      *target_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

// bool RouteHandler::getClosestLaneletWithinRoute(
//   const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
// {
//   if (!is_handler_ready_) {
//     return false;
//   }
//   std::optional<lanelet::ConstLanelet> optional_lanelet =
//     lanelet_route_ptr_->getClosestLaneletWithinRoute(search_pose);
//   if (!optional_lanelet) {
//     return false;
//   }
//   *closest_lanelet = *optional_lanelet;
//   return true;
// }

// lanelet::ConstLanelets RouteHandler::getNextLanelets(const lanelet::ConstLanelet & lanelet) const
// {
//   return routing_graph_ptr_->following(lanelet);
// }

// bool RouteHandler::getPreviousLaneletsWithinRoute(
//   const LaneletPoint & lanelet_point, lanelet::ConstLanelets * prev_lanelets) const
// {
//   if (!is_handler_ready_) {
//     return false;
//   }
//   *prev_lanelets = lanelet_route_ptr_->getPreviousLanelets(lanelet_point);
//   return !(prev_lanelets->empty());
// }

// lanelet::ConstLanelets RouteHandler::getPreviousLanelets(
//   const lanelet::ConstLanelet & lanelet) const
// {
//   return routing_graph_ptr_->previous(lanelet);
// }

// lanelet::ConstLanelets RouteHandler::getLaneletsFromPoint(const lanelet::ConstPoint3d & point) const
// {
//   return lanelet::utils::findUsagesInLanelets(*lanelet_map_ptr_, point);
// }

// bool RouteHandler::getRightLaneletWithinRoute(
//   const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet) const
// {
//   if (!is_handler_ready_) {
//     return false;
//   }
//   const std::optional<lanelet::ConstLanelet> optional_right_lanelet =
//     lanelet_route_ptr_->getRightLaneletWithinRoute(lanelet);
//   if (!optional_right_lanelet) {
//     return false;
//   }
//   *right_lanelet = *optional_right_lanelet;
//   return true;
// }

boost::optional<lanelet::ConstLanelet> RouteHandler::getRightLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  // routable lane
  const auto & right_lane = routing_graph_ptr_->right(lanelet);
  if (right_lane) {
    return right_lane;
  }

  // non-routable lane (e.g. lane change infeasible)
  const auto & adjacent_right_lane = routing_graph_ptr_->adjacentRight(lanelet);
  return adjacent_right_lane;
}

// bool RouteHandler::getLeftLaneletWithinRoute(
//   const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet) const
// {
//   if (!is_handler_ready_) {
//     return false;
//   }
//   const std::optional<lanelet::ConstLanelet> optional_left_lanelet =
//     lanelet_route_ptr_->getLeftLaneletWithinRoute(lanelet);
//   if (!optional_left_lanelet) {
//     return false;
//   }
//   *left_lanelet = *optional_left_lanelet;
//   return true;
// }

boost::optional<lanelet::ConstLanelet> RouteHandler::getLeftLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  if (!is_map_msg_ready_) {
    return {};
  }

  // routable lane
  const auto & left_lane = routing_graph_ptr_->left(lanelet);
  if (left_lane) {
    return left_lane;
  }

  // non-routable lane (e.g. lane change infeasible)
  const auto & adjacent_left_lane = routing_graph_ptr_->adjacentLeft(lanelet);
  return adjacent_left_lane;
}

lanelet::Lanelets RouteHandler::getRightOppositeLanelets(
  const lanelet::ConstLanelet & lanelet) const
{
  if (!is_map_msg_ready_) {
    return {};
  }

  const auto opposite_candidate_lanelets =
    lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.leftBound().id() == lanelet.rightBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

lanelet::ConstLanelets RouteHandler::getAllLeftSharedLinestringLanelets(
  const lanelet::ConstLanelet & lane, const bool & include_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets linestring_shared;
  auto lanelet_at_left = getLeftLanelet(lane);
  auto lanelet_at_left_opposite = getLeftOppositeLanelets(lane);
  while (lanelet_at_left) {
    linestring_shared.push_back(lanelet_at_left.get());
    lanelet_at_left = getLeftLanelet(lanelet_at_left.get());
    if (!lanelet_at_left) {
      break;
    }
    lanelet_at_left_opposite = getLeftOppositeLanelets(lanelet_at_left.get());
  }

  if (!lanelet_at_left_opposite.empty() && include_opposite) {
    if (invert_opposite) {
      linestring_shared.push_back(lanelet_at_left_opposite.front().invert());
    } else {
      linestring_shared.push_back(lanelet_at_left_opposite.front());
    }
    auto lanelet_at_right = getRightLanelet(lanelet_at_left_opposite.front());
    while (lanelet_at_right) {
      if (invert_opposite) {
        linestring_shared.push_back(lanelet_at_right.get().invert());
      } else {
        linestring_shared.push_back(lanelet_at_right.get());
      }
      lanelet_at_right = getRightLanelet(lanelet_at_right.get());
    }
  }
  return linestring_shared;
}

lanelet::ConstLanelets RouteHandler::getAllRightSharedLinestringLanelets(
  const lanelet::ConstLanelet & lane, const bool & include_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets linestring_shared;
  auto lanelet_at_right = getRightLanelet(lane);
  auto lanelet_at_right_opposite = getRightOppositeLanelets(lane);
  while (lanelet_at_right) {
    linestring_shared.push_back(lanelet_at_right.get());
    lanelet_at_right = getRightLanelet(lanelet_at_right.get());
    if (!lanelet_at_right) {
      break;
    }
    lanelet_at_right_opposite = getRightOppositeLanelets(lanelet_at_right.get());
  }

  if (!lanelet_at_right_opposite.empty() && include_opposite) {
    if (invert_opposite) {
      linestring_shared.push_back(lanelet_at_right_opposite.front().invert());
    } else {
      linestring_shared.push_back(lanelet_at_right_opposite.front());
    }
    auto lanelet_at_left = getLeftLanelet(lanelet_at_right_opposite.front());
    while (lanelet_at_left) {
      if (invert_opposite) {
        linestring_shared.push_back(lanelet_at_left.get().invert());
      } else {
        linestring_shared.push_back(lanelet_at_left.get());
      }
      lanelet_at_left = getLeftLanelet(lanelet_at_left.get());
    }
  }
  return linestring_shared;
}

lanelet::ConstLanelets RouteHandler::getAllSharedLineStringLanelets(
  const lanelet::ConstLanelet & current_lane, bool is_right, bool is_left, bool is_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets shared{current_lane};

  if (is_right) {
    const lanelet::ConstLanelets all_right_lanelets =
      getAllRightSharedLinestringLanelets(current_lane, is_opposite, invert_opposite);
    shared.insert(shared.end(), all_right_lanelets.begin(), all_right_lanelets.end());
  }

  if (is_left) {
    const lanelet::ConstLanelets all_left_lanelets =
      getAllLeftSharedLinestringLanelets(current_lane, is_opposite, invert_opposite);
    shared.insert(shared.end(), all_left_lanelets.begin(), all_left_lanelets.end());
  }

  return shared;
}

lanelet::Lanelets RouteHandler::getLeftOppositeLanelets(const lanelet::ConstLanelet & lanelet) const
{
  if (!is_map_msg_ready_) {
    return {};
  }

  const auto opposite_candidate_lanelets =
    lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.rightBound().id() == lanelet.leftBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

lanelet::ConstLanelet RouteHandler::getMostLeftLanelet(const lanelet::ConstLanelet & lanelet) const
{
  // recursively compute the width of the lanes
  const auto & same = getLeftLanelet(lanelet);

  if (same) {
    return getMostLeftLanelet(same.get());
  }
  return lanelet;
}

lanelet::ConstLineString3d RouteHandler::getRightMostSameDirectionLinestring(
  const lanelet::ConstLanelet & lanelet) const noexcept
{
  // recursively compute the width of the lanes
  const auto & same = getRightLanelet(lanelet);

  if (same) {
    return getRightMostSameDirectionLinestring(same.get());
  }
  return lanelet.rightBound();
}

lanelet::ConstLineString3d RouteHandler::getRightMostLinestring(
  const lanelet::ConstLanelet & lanelet) const noexcept
{
  const auto & same = getRightLanelet(lanelet);
  const auto & opposite = getRightOppositeLanelets(lanelet);
  if (!same && opposite.empty()) {
    return lanelet.rightBound();
  }

  if (same) {
    return getRightMostLinestring(same.get());
  }

  if (!opposite.empty()) {
    return getLeftMostLinestring(lanelet::ConstLanelet(opposite.front()));
  }

  return lanelet.rightBound();
}

lanelet::ConstLineString3d RouteHandler::getLeftMostSameDirectionLinestring(
  const lanelet::ConstLanelet & lanelet) const noexcept
{
  // recursively compute the width of the lanes
  const auto & same = getLeftLanelet(lanelet);

  if (same) {
    return getLeftMostSameDirectionLinestring(same.get());
  }
  return lanelet.leftBound();
}

lanelet::ConstLineString3d RouteHandler::getLeftMostLinestring(
  const lanelet::ConstLanelet & lanelet) const noexcept
{
  // recursively compute the width of the lanes
  const auto & same = getLeftLanelet(lanelet);
  const auto & opposite = getLeftOppositeLanelets(lanelet);

  if (!same && opposite.empty()) {
    return lanelet.leftBound();
  }

  if (same) {
    return getLeftMostLinestring(same.get());
  }

  if (!opposite.empty()) {
    return getRightMostLinestring(lanelet::ConstLanelet(opposite.front()));
  }

  return lanelet.leftBound();
}

lanelet::ConstLineStrings3d RouteHandler::getFurthestLinestring(
  const lanelet::ConstLanelet & lanelet, bool is_right, bool is_left,
  bool is_opposite) const noexcept
{
  lanelet::ConstLineStrings3d linestrings;
  linestrings.reserve(2);

  if (is_right && is_opposite) {
    linestrings.emplace_back(getRightMostLinestring(lanelet));
  } else if (is_right && !is_opposite) {
    linestrings.emplace_back(getRightMostSameDirectionLinestring(lanelet));
  } else {
    linestrings.emplace_back(lanelet.rightBound());
  }

  if (is_left && is_opposite) {
    linestrings.emplace_back(getLeftMostLinestring(lanelet));
  } else if (is_left && !is_opposite) {
    linestrings.emplace_back(getLeftMostSameDirectionLinestring(lanelet));
  } else {
    linestrings.emplace_back(lanelet.leftBound());
  }
  return linestrings;
}

bool RouteHandler::getNextLaneChangeTarget(
  const LaneletPath & lanelet_path, lanelet::ConstLanelet * target_lanelet) const
{
  if (!is_handler_ready_) {
    return false;
  }

  for (const auto & section : lanelet_path) {
    const std::optional<int> optional_num = lanelet_route_ptr_->getNumLaneChangeToPreferredLane(section.getStartPoint());
    if (!optional_num) {
      continue; // TODO(vrichard) Is it ok?
    }

    const int num = *optional_num;
    if (num == 0) {
      continue; // already on the preferred section
    }

    if (num < 0) {
      // need to go left
      *target_lanelet = *lanelet_route_ptr_->getRightLaneletWithinRoute(section.lanelet());
      return true;
    }

    if (num > 0) {
      // need to go right
      *target_lanelet = *lanelet_route_ptr_->getLeftLaneletWithinRoute(section.lanelet());
      return true;
    }
  }

  // no lane change required
  return false;
}

// int RouteHandler::getNumLaneToPreferredLane(const LaneletPoint & lanelet_point) const
// {
//   if (!is_handler_ready_) {
//     return 0;  // TODO(vrichard) same question than below
//   }

//   const auto & preferred_path = lanelet_route_ptr_->getMainPath();

//   int num = 0;
//   if (preferred_path.contains(lanelet_point)) {
//     return num; // already on preferred lane
//   }

//   const auto & right_lanes =
//     lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet_point.lanelet());
//   for (const auto & right_llt : right_lanes) {
//     num--;
//     const auto right_point = LaneletPoint::fromProjection(right_llt, lanelet_point.toBasicPoint2d());
//     if (preferred_path.contains(right_point)) {
//       return num;
//     }
//   }

//   const auto & left_lanes = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet_point.lanelet());
//   num = 0;
//   for (const auto & left_llt : left_lanes) {
//     num++;
//     const auto left_point = LaneletPoint::fromProjection(left_llt, lanelet_point.toBasicPoint2d());
//     if (preferred_path.contains(left_point)) {
//       return num;
//     }
//   }

//   // the point is outside the route
//   return 0;  // TODO(Horibe) check if return 0 is appropriate.
// }

// bool RouteHandler::isInPreferredLane(const PoseStamped & pose) const
// {
//   if (!is_handler_ready_) {
//     return false;
//   }

//   std::optional<lanelet::ConstLanelet> optional_lanelet =
//     lanelet_route_ptr_->getClosestLaneletWithinRoute(pose.pose);
//   if (!optional_lanelet) {
//     return false;
//   }

//   const LaneletPoint lanelet_point = LaneletPoint::fromProjection(*optional_lanelet, pose);

//   const auto & preferred_path = lanelet_route_ptr_->getMainPath();

//   return preferred_path.contains(lanelet_point);
// }

// bool RouteHandler::isInTargetLane(
//   const PoseStamped & pose, const lanelet::ConstLanelets & target) const
// {
//   if (!is_handler_ready_) {
//     return false;
//   }
//   std::optional<lanelet::ConstLanelet> optional_lanelet =
//     lanelet_route_ptr_->getClosestLaneletWithinRoute(pose.pose);
//   if (!optional_lanelet) {
//     return false;
//   }
//   return exists(target, *optional_lanelet);
// }

PathWithLaneId RouteHandler::getCenterLinePath(
  const LaneletPath & lanelet_path, bool use_exact) const
{
  PathWithLaneId reference_path{};

  if (!is_map_msg_ready_) {
    return reference_path;
  }

  for (const auto & section : lanelet_path) {
    const auto& llt = section.lanelet();
    const lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(llt);
    const lanelet::ConstLineString3d centerline = llt.centerline();

    const auto add_path_point = [&reference_path, &limit, &llt](const auto & pt) {
      PathPointWithLaneId p{};
      p.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(pt);
      p.lane_ids.push_back(llt.id());
      p.point.longitudinal_velocity_mps = static_cast<float>(limit.speedLimit.value());
      reference_path.points.push_back(p);
    };

    const double s_start = section.start_arc_length();
    const double s_end = section.end_arc_length();
    double s = 0;
    for (size_t i = 0; i < centerline.size(); i++) {
      const auto & pt = centerline[i];
      const lanelet::ConstPoint3d next_pt =
        (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      const double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start) {
        const auto p = use_exact ? get3DPointFrom2DArcLength({llt}, s_start) : pt;
        add_path_point(p);
      }
      if (s >= s_start && s <= s_end) {
        add_path_point(pt);
      }
      if (s < s_end && s + distance > s_end) {
        const auto p = use_exact ? get3DPointFrom2DArcLength({llt}, s_end) : next_pt;
        add_path_point(p);
      }
      s += distance;
    }
  }

  reference_path = removeOverlappingPoints(reference_path);

  // append a point only when having one point so that yaw calculation would work
  if (reference_path.points.size() == 1) {
    const int lane_id = static_cast<int>(reference_path.points.front().lane_ids.front());
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
    const auto & pts = reference_path.points;
    if (i + 1 < reference_path.points.size()) {
      angle = tier4_autoware_utils::calcAzimuthAngle(
        pts.at(i).point.pose.position, pts.at(i + 1).point.pose.position);
    } else if (i != 0) {
      angle = tier4_autoware_utils::calcAzimuthAngle(
        pts.at(i - 1).point.pose.position, pts.at(i).point.pose.position);
    }
    reference_path.points.at(i).point.pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(angle);
  }

  return reference_path;
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

#if 0
lanelet::routing::RelationType RouteHandler::getRelation(
  const lanelet::ConstLanelet & prev_lane, const lanelet::ConstLanelet & next_lane) const
{
  if (!is_map_msg_ready_) {
    return lanelet::routing::RelationType::None; // can't tell
  }

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
        relation.get() == lanelet::routing::RelationType::Right) {
        return relation.get();
      }
      prev_llt = llt;
    }
  }

  return lanelet::routing::RelationType::None;
}
#endif

// std::vector<LaneletSegment> RouteHandler::createLaneletSegments() const
// {
//   if (!is_handler_ready_) {
//     return {};
//   }

//   return lanelet_route_.toLaneletSegmentMsgs();
// }

}  // namespace route_handler
