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

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
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

std::optional<PathWithLaneId> generateCenterLinePath(const PlannerData & planner_data)
{
  lanelet::ConstLanelet current_lane;
  if (!lanelet::utils::query::getClosestLanelet(
        planner_data.preferred_lanelets, planner_data.current_pose, &current_lane)) {
    return std::nullopt;
  }

  const auto lanelet_sequence = getLaneletSequence(current_lane, planner_data);
  if (!lanelet_sequence) {
    return std::nullopt;
  }

  const auto centerline_path = getCenterLinePath(*lanelet_sequence, planner_data);
  if (!centerline_path) {
    return std::nullopt;
  }

  return centerline_path;
}

std::optional<PathWithLaneId> getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const PlannerData & planner_data)
{
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  if (!planner_data.route_ptr) {
    return std::nullopt;
  }

  const auto arc_coordinates =
    lanelet::utils::getArcCoordinates(lanelet_sequence, planner_data.current_pose);
  const auto s = arc_coordinates.length;  // s denotes longitudinal position in Frenet coordinates
  const auto s_start = std::max(0., s - planner_data.backward_path_length);
  const auto s_end = [&]() {
    auto s_end = s + planner_data.forward_path_length;

    if (!getNextLaneletWithinRoute(lanelet_sequence.back(), planner_data)) {
      const auto lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
      s_end = std::clamp(s_end, 0.0, lane_length);
    }

    if (exists(planner_data.goal_lanelets, lanelet_sequence.back())) {
      const auto goal_arc_coordinates =
        lanelet::utils::getArcCoordinates(lanelet_sequence, planner_data.route_ptr->goal_pose);
      s_end = std::clamp(s_end, 0.0, goal_arc_coordinates.length);
    }

    return s_end;
  }();

  const auto raw_centerline_path =
    getCenterLinePath(lanelet_sequence, s_start, s_end, planner_data);
  if (!raw_centerline_path) {
    return std::nullopt;
  }

  auto centerline_path = autoware::motion_utils::resamplePath(
    *raw_centerline_path, planner_data.input_path_interval, planner_data.enable_akima_spline_first);

  const auto current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      centerline_path.points, planner_data.current_pose, planner_data.ego_nearest_dist_threshold,
      planner_data.ego_nearest_yaw_threshold);

  centerline_path.points = autoware::motion_utils::cropPoints(
    centerline_path.points, planner_data.current_pose.position, current_seg_idx,
    planner_data.forward_path_length,
    planner_data.backward_path_length + planner_data.input_path_interval);

  return centerline_path;
}

std::optional<PathWithLaneId> getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  const PlannerData & planner_data)
{
  if (!planner_data.lanelet_map_ptr || !planner_data.traffic_rules_ptr || !planner_data.route_ptr) {
    return std::nullopt;
  }

  PathWithLaneId centerline_path{};
  auto & path_points = centerline_path.points;

  const auto waypoint_groups =
    utils::getWaypointGroups(lanelet_sequence, *planner_data.lanelet_map_ptr);

  double s = 0.;
  for (const auto & lanelet : lanelet_sequence) {
    std::vector<geometry_msgs::msg::Point> reference_points;
    const auto & centerline = lanelet.centerline();

    std::optional<size_t> overlapped_waypoint_group_index = std::nullopt;
    for (auto it = centerline.begin(); it != centerline.end(); ++it) {
      if (s <= s_end) {
        const lanelet::Point3d point(*it);
        if (s >= s_start) {
          for (size_t i = 0; i < waypoint_groups.size(); ++i) {
            const auto & [waypoints, interval] = waypoint_groups[i];
            if (s >= interval.first && s <= interval.second) {
              overlapped_waypoint_group_index = i;
              break;
            } else if (i == overlapped_waypoint_group_index) {
              for (const auto & waypoint : waypoints) {
                reference_points.push_back(lanelet::utils::conversion::toGeomMsgPt(waypoint));
              }
              overlapped_waypoint_group_index = std::nullopt;
            }
          }
          if (!overlapped_waypoint_group_index) {
            reference_points.push_back(lanelet::utils::conversion::toGeomMsgPt(point));
          }
        }
        if (it == std::prev(centerline.end())) {
          break;
        }

        const lanelet::Point3d next_point(*std::next(it));
        const auto distance = lanelet::geometry::distance2d(point, next_point);
        std::optional<double> s_interpolation = std::nullopt;
        if (s + distance > s_end) {
          s_interpolation = s_end - s;
        } else if (s < s_start && s + distance > s_start) {
          s_interpolation = s_start - s;
        }

        if (s_interpolation) {
          const auto interpolated_point = lanelet::geometry::interpolatedPointAtDistance(
            lanelet::ConstLineString3d{lanelet::InvalId, {point, next_point}}, *s_interpolation);
          reference_points.push_back(lanelet::utils::conversion::toGeomMsgPt(interpolated_point));
        }
        s += distance;
      } else {
        break;
      }
    }

    const auto speed_limit = planner_data.traffic_rules_ptr->speedLimit(lanelet).speedLimit.value();
    for (const auto & reference_point : reference_points) {
      PathPointWithLaneId path_point{};
      path_point.point.pose.position = reference_point;
      path_point.lane_ids.push_back(lanelet.id());
      path_point.point.longitudinal_velocity_mps = static_cast<float>(speed_limit);
      path_points.push_back(path_point);
    }
  }

  utils::removeOverlappingPoints(centerline_path);

  // append a point if having only one point so that yaw calculation would work
  if (path_points.size() == 1) {
    const auto & lane_id = path_points.front().lane_ids.front();
    const auto & lanelet = planner_data.lanelet_map_ptr->laneletLayer.get(lane_id);
    const auto & point = path_points.front().point.pose.position;
    const auto lane_yaw = lanelet::utils::getLaneletAngle(lanelet, point);

    PathPointWithLaneId path_point{};
    path_point.lane_ids.push_back(lane_id);
    constexpr double ds = 0.1;
    path_point.point.pose.position.x = point.x + ds * std::cos(lane_yaw);
    path_point.point.pose.position.y = point.y + ds * std::sin(lane_yaw);
    path_point.point.pose.position.z = point.z;
    path_points.push_back(path_point);
  }

  // set yaw to each point
  for (auto it = path_points.begin(); it != std::prev(path_points.end()); ++it) {
    const auto angle = autoware::universe_utils::calcAzimuthAngle(
      it->point.pose.position, std::next(it)->point.pose.position);
    it->point.pose.orientation = autoware::universe_utils::createQuaternionFromYaw(angle);
  }
  path_points.back().point.pose.orientation =
    std::prev(path_points.end(), 2)->point.pose.orientation;

  centerline_path.header = planner_data.route_ptr->header;
  return centerline_path;
}

std::optional<lanelet::ConstLanelets> getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  auto lanelet_sequence = [&]() -> std::optional<lanelet::ConstLanelets> {
    const auto arc_coordinate =
      lanelet::utils::getArcCoordinates({lanelet}, planner_data.current_pose);
    if (arc_coordinate.length < planner_data.backward_path_length) {
      return getLaneletSequenceUpTo(lanelet, planner_data);
    }
    return lanelet::ConstLanelets{};
  }();
  if (!lanelet_sequence) {
    return std::nullopt;
  }

  const auto lanelet_sequence_forward = getLaneletSequenceAfter(lanelet, planner_data);
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

std::optional<lanelet::ConstLanelets> getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (!planner_data.routing_graph_ptr) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelet_sequence{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < planner_data.forward_path_length) {
    auto next_lanelet = getNextLaneletWithinRoute(current_lanelet, planner_data);
    if (!next_lanelet) {
      const auto next_lanelets = planner_data.routing_graph_ptr->following(current_lanelet);
      if (next_lanelets.empty()) {
        break;
      }
      next_lanelet = next_lanelets.front();
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

std::optional<lanelet::ConstLanelets> getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (!planner_data.routing_graph_ptr) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelet_sequence{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < planner_data.backward_path_length) {
    auto previous_lanelets = getPreviousLaneletsWithinRoute(current_lanelet, planner_data);
    if (!previous_lanelets) {
      previous_lanelets = planner_data.routing_graph_ptr->previous(current_lanelet);
      if (previous_lanelets->empty()) {
        break;
      }
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

std::optional<lanelet::ConstLanelet> getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  const auto next_lanelets = getNextLaneletsWithinRoute(lanelet, planner_data);

  if (next_lanelets) {
    return next_lanelets->front();
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelets> getNextLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (!planner_data.routing_graph_ptr) {
    return std::nullopt;
  }

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

std::optional<lanelet::ConstLanelets> getPreviousLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (!planner_data.routing_graph_ptr) {
    return std::nullopt;
  }

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

std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> getWaypointGroups(
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

void removeOverlappingPoints(PathWithLaneId & path)
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
