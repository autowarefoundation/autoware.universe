// Copyright 2024 TIER IV, Inc.
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

#include "autoware/path_generator/utils.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::path_generator
{
namespace utils
{
namespace
{
template <typename T>
bool exists(const std::vector<T> & vec, const T & item)
{
  return std::find(vec.begin(), vec.end(), item) != vec.end();
}
}  // namespace

std::optional<lanelet::ConstLanelets> get_lanelet_sequence(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double forward_distance,
  const double backward_distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  auto lanelet_sequence = [&]() -> std::optional<lanelet::ConstLanelets> {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      return get_lanelet_sequence_up_to(lanelet, planner_data, backward_distance);
    }
    return lanelet::ConstLanelets{};
  }();
  if (!lanelet_sequence) {
    return std::nullopt;
  }

  const auto lanelet_sequence_forward =
    get_lanelet_sequence_after(lanelet, planner_data, forward_distance);
  if (!lanelet_sequence_forward) {
    return std::nullopt;
  }

  // loop check
  if (
    !lanelet_sequence_forward->empty() && !lanelet_sequence->empty() &&
    lanelet_sequence->back().id() == lanelet_sequence_forward->front().id()) {
    return lanelet_sequence_forward;
  }

  lanelet_sequence->push_back(lanelet);
  std::move(
    lanelet_sequence_forward->begin(), lanelet_sequence_forward->end(),
    std::back_inserter(*lanelet_sequence));

  return lanelet_sequence;
}

std::optional<lanelet::ConstLanelets> get_lanelet_sequence_after(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelet_sequence{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < distance) {
    auto next_lanelet = get_next_lanelet_within_route(current_lanelet, planner_data);
    if (!next_lanelet) {
      break;
    }

    // loop check
    if (lanelet.id() == next_lanelet->id()) {
      break;
    }

    lanelet_sequence.push_back(*next_lanelet);
    current_lanelet = *next_lanelet;
    length += lanelet::utils::getLaneletLength2d(*next_lanelet);
  }

  return lanelet_sequence;
}

std::optional<lanelet::ConstLanelets> get_lanelet_sequence_up_to(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelet_sequence{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < distance) {
    auto previous_lanelets = get_previous_lanelets_within_route(current_lanelet, planner_data);
    if (!previous_lanelets) {
      break;
    }

    if (
      std::count_if(
        previous_lanelets->begin(), previous_lanelets->end(), [&](const auto & prev_llt) {
          return lanelet.id() == prev_llt.id();
        }) >= (previous_lanelets ? static_cast<ptrdiff_t>(previous_lanelets->size()) : 1)) {
      break;
    }

    for (const auto & prev_lanelet : *previous_lanelets) {
      if (
        lanelet.id() == prev_lanelet.id() ||
        std::any_of(
          lanelet_sequence.begin(), lanelet_sequence.end(),
          [&](const auto & llt) { return llt.id() == prev_lanelet.id(); }) ||
        exists(planner_data.goal_lanelets, prev_lanelet)) {
        continue;
      }
      lanelet_sequence.push_back(prev_lanelet);
      length += lanelet::utils::getLaneletLength2d(prev_lanelet);
      current_lanelet = prev_lanelet;
      break;
    }
  }

  std::reverse(lanelet_sequence.begin(), lanelet_sequence.end());
  return lanelet_sequence;
}

std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  const auto next_lanelets = get_next_lanelets_within_route(lanelet, planner_data);

  if (next_lanelets) {
    return next_lanelets->front();
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelets> get_next_lanelets_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }

  if (exists(planner_data.goal_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets next_lanelets{};
  for (const auto & lanelet : planner_data.routing_graph_ptr->following(lanelet)) {
    if (
      planner_data.preferred_lanelets.front().id() != lanelet.id() &&
      exists(planner_data.route_lanelets, lanelet)) {
      next_lanelets.push_back(lanelet);
    }
  }

  if (next_lanelets.empty()) {
    return std::nullopt;
  }
  return next_lanelets;
}

std::optional<lanelet::ConstLanelets> get_previous_lanelets_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (exists(planner_data.start_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets prev_lanelets{};
  for (const auto & lanelet : planner_data.routing_graph_ptr->previous(lanelet)) {
    if (exists(planner_data.route_lanelets, lanelet)) {
      prev_lanelets.push_back(lanelet);
    }
  }

  if (prev_lanelets.empty()) {
    return std::nullopt;
  }
  return prev_lanelets;
}

std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> get_waypoint_groups(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::LaneletMap & lanelet_map)
{
  std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> waypoint_groups;

  double arc_length_offset = 0.;
  for (const auto & lanelet : lanelet_sequence) {
    if (!lanelet.hasAttribute("waypoints")) {
      arc_length_offset += lanelet::utils::getLaneletLength2d(lanelet);
      continue;
    }

    const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
    const auto & waypoints = lanelet_map.lineStringLayer.get(waypoints_id);
    const auto get_interval_bound =
      [&](const lanelet::ConstPoint3d & point, const double lateral_distance_factor) {
        const auto arc_coordinates =
          lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), lanelet::utils::to2D(point));
        return arc_length_offset + arc_coordinates.length +
               lateral_distance_factor * std::abs(arc_coordinates.distance);
      };

    constexpr auto new_group_threshold = 1.0;
    constexpr auto lateral_distance_factor = 10.0;
    if (
      waypoint_groups.empty() ||
      lanelet::geometry::distance2d(waypoint_groups.back().first.back(), waypoints.front()) >
        new_group_threshold) {
      waypoint_groups.emplace_back().second.first =
        get_interval_bound(waypoints.front(), -lateral_distance_factor);
    }
    waypoint_groups.back().second.second =
      get_interval_bound(waypoints.back(), lateral_distance_factor);

    for (const auto & waypoint : waypoints) {
      waypoint_groups.back().first.push_back(waypoint);
    }

    arc_length_offset += lanelet::utils::getLaneletLength2d(lanelet);
  }

  return waypoint_groups;
}

void remove_overlapping_points(PathWithLaneId & path)
{
  auto & filtered_path_end = path.points.front();
  for (auto it = std::next(path.points.begin()); it != path.points.end();) {
    constexpr auto min_interval = 0.001;
    if (
      autoware::universe_utils::calcDistance3d(filtered_path_end.point, it->point) < min_interval) {
      filtered_path_end.lane_ids.push_back(it->lane_ids.front());
      filtered_path_end.point.longitudinal_velocity_mps = std::min(
        it->point.longitudinal_velocity_mps, filtered_path_end.point.longitudinal_velocity_mps);
      it = path.points.erase(it);
    } else {
      filtered_path_end = *it;
      ++it;
    }
  }
}
}  // namespace utils
}  // namespace autoware::path_generator
