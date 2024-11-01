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

#include "autoware/path_generator/path_handler.hpp"

#include "autoware/path_generator/utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/route_checker.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::path_generator
{
namespace
{
template <typename T>
bool exists(const std::vector<T> & vec, const T & item)
{
  return std::find(vec.begin(), vec.end(), item) != vec.end();
}
}  // namespace

PathWithLaneId PathHandler::generateCenterLinePath(const Pose & current_pose, const Params & param)
{
  if (!route_ptr_) {
    return PathWithLaneId{};
  }

  lanelet::ConstLanelet current_lane;
  if (!lanelet::utils::query::getClosestLanelet(preferred_lanelets_, current_pose, &current_lane)) {
    return PathWithLaneId{};
  }

  const auto lanelet_sequence = getLaneletSequence(
    current_lane, current_pose, param.backward_path_length, param.forward_path_length);

  auto centerline_path = generateCenterLinePath(lanelet_sequence, current_pose, param);
  centerline_path.header = route_ptr_->header;

  return centerline_path;
}

PathWithLaneId PathHandler::generateCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const Pose & current_pose, const Params & param)
{
  if (lanelet_sequence.empty() || !vehicle_info_) {
    return PathWithLaneId{};
  }

  const auto arc_coordinates = lanelet::utils::getArcCoordinates(lanelet_sequence, current_pose);
  const auto s = arc_coordinates.length;  // s denotes longitudinal position in Frenet coordinates
  const auto s_start = std::max(0., s - param.backward_path_length);
  const auto s_end = [&]() {
    auto s_end = s + param.forward_path_length;

    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(lanelet_sequence.back(), next_lanelet)) {
      const auto lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
      s_end = std::clamp(s_end, 0.0, lane_length);
    }

    if (exists(goal_lanelets_, lanelet_sequence.back())) {
      const auto goal_arc_coordinates =
        lanelet::utils::getArcCoordinates(lanelet_sequence, route_ptr_->goal_pose);
      s_end = std::clamp(s_end, 0.0, goal_arc_coordinates.length);
    }

    return s_end;
  }();

  const auto raw_path_with_lane_id = getCenterLinePath(lanelet_sequence, s_start, s_end);
  const auto resampled_path_with_lane_id = autoware::motion_utils::resamplePath(
    raw_path_with_lane_id, param.input_path_interval, param.enable_akima_spline_first);

  // convert centerline, which we consider as CoG center, to rear wheel center
  if (param.enable_cog_on_centerline) {
    const auto rear_to_cog = vehicle_info_->vehicle_length_m / 2 - vehicle_info_->rear_overhang_m;
    return autoware::motion_utils::convertToRearWheelCenter(
      resampled_path_with_lane_id, rear_to_cog);
  }

  return resampled_path_with_lane_id;
}

PathHandler & PathHandler::setRoute(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & lanelet_map_bin_ptr,
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr & route_ptr)
{
  lanelet::utils::conversion::fromBinMsg(
    *lanelet_map_bin_ptr, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);

  if (!lanelet::utils::route::isRouteValid(*route_ptr, lanelet_map_ptr_)) {
    return *this;
  }
  route_ptr_ = route_ptr;

  route_lanelets_.clear();
  preferred_lanelets_.clear();
  start_lanelets_.clear();
  goal_lanelets_.clear();

  if (!route_ptr_->segments.empty()) {
    size_t primitive_size = 0;
    for (const auto & route_section : route_ptr_->segments) {
      primitive_size += route_section.primitives.size();
    }
    route_lanelets_.reserve(primitive_size);

    for (const auto & route_section : route_ptr_->segments) {
      for (const auto & primitive : route_section.primitives) {
        const auto id = primitive.id;
        const auto & lanelet = lanelet_map_ptr_->laneletLayer.get(id);
        route_lanelets_.push_back(lanelet);
        if (id == route_section.preferred_primitive.id) {
          preferred_lanelets_.push_back(lanelet);
        }
      }
    }

    const auto set_lanelets_from_segment =
      [&](
        const autoware_planning_msgs::msg::LaneletSegment & segment,
        lanelet::ConstLanelets & lanelets) {
        lanelets.reserve(segment.primitives.size());
        for (const auto & primitive : segment.primitives) {
          const auto & lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
          lanelets.push_back(lanelet);
        }
      };
    set_lanelets_from_segment(route_ptr_->segments.front(), start_lanelets_);
    set_lanelets_from_segment(route_ptr_->segments.back(), goal_lanelets_);
  }

  return *this;
}

PathWithLaneId PathHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end)
{
  if (!lanelet_map_ptr_ || !traffic_rules_ptr_) {
    return PathWithLaneId{};
  }

  PathWithLaneId centerline_path{};
  auto & path_points = centerline_path.points;

  const auto waypoint_groups = utils::getWaypointGroups(lanelet_sequence, *lanelet_map_ptr_);

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

    const auto speed_limit = traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value();
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
    const auto & lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
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
  {
    auto it = path_points.begin();
    for (; it != std::prev(path_points.end()); ++it) {
      const auto angle = autoware::universe_utils::calcAzimuthAngle(
        it->point.pose.position, std::next(it)->point.pose.position);
      it->point.pose.orientation = autoware::universe_utils::createQuaternionFromYaw(angle);
    }
    path_points.back().point.pose.orientation = it->point.pose.orientation;
  }

  return centerline_path;
}

lanelet::ConstLanelets PathHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & current_pose,
  const double backward_path_length, const double forward_path_length)
{
  auto lanelet_sequence = [&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_path_length) {
      return getLaneletSequenceUpTo(lanelet, backward_path_length);
    }
    return lanelet::ConstLanelets{};
  }();
  const auto lanelet_sequence_forward = getLaneletSequenceAfter(lanelet, forward_path_length);

  // loop check
  if (
    !lanelet_sequence_forward.empty() && !lanelet_sequence.empty() &&
    lanelet_sequence.back().id() == lanelet_sequence_forward.front().id()) {
    return lanelet_sequence_forward;
  }

  lanelet_sequence.push_back(lanelet);
  std::move(
    lanelet_sequence_forward.begin(), lanelet_sequence_forward.end(),
    std::back_inserter(lanelet_sequence));

  return lanelet_sequence;
}

lanelet::ConstLanelets PathHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length)
{
  if (!routing_graph_ptr_) {
    return lanelet::ConstLanelets{};
  }

  lanelet::ConstLanelets lanelet_sequence{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, next_lanelet)) {
      const auto next_lanes = routing_graph_ptr_->following(current_lanelet);
      if (next_lanes.empty()) {
        break;
      }
      next_lanelet = next_lanes.front();
    }

    // loop check
    if (lanelet.id() == next_lanelet.id()) {
      break;
    }

    lanelet_sequence.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length += lanelet::utils::getLaneletLength2d(next_lanelet);
  }

  return lanelet_sequence;
}

lanelet::ConstLanelets PathHandler::getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length)
{
  if (!routing_graph_ptr_) {
    return lanelet::ConstLanelets{};
  }

  lanelet::ConstLanelets lanelet_sequence{};
  lanelet::ConstLanelets previous_lanelets;
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < min_length) {
    bool is_route_lanelets = getPreviousLaneletsWithinRoute(current_lanelet, previous_lanelets);
    if (!is_route_lanelets) {
      previous_lanelets = routing_graph_ptr_->previous(current_lanelet);
      if (previous_lanelets.empty()) {
        break;
      }
    }

    if (
      std::count_if(previous_lanelets.begin(), previous_lanelets.end(), [&](const auto & prev_llt) {
        return lanelet.id() == prev_llt.id();
      }) >= (is_route_lanelets ? static_cast<ptrdiff_t>(previous_lanelets.size()) : 1)) {
      break;
    }

    for (const auto & prev_lanelet : previous_lanelets) {
      if (
        lanelet.id() == prev_lanelet.id() ||
        std::any_of(
          lanelet_sequence.begin(), lanelet_sequence.end(),
          [&](const auto & llt) { return llt.id() == prev_lanelet.id(); }) ||
        exists(goal_lanelets_, prev_lanelet)) {
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

bool PathHandler::getNextLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & next_lanelets)
{
  if (!routing_graph_ptr_ || preferred_lanelets_.empty()) {
    return false;
  }

  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }

  next_lanelets.clear();
  for (const auto & lanelet : routing_graph_ptr_->following(lanelet)) {
    if (preferred_lanelets_.front().id() != lanelet.id() && exists(route_lanelets_, lanelet)) {
      next_lanelets.push_back(lanelet);
    }
  }

  return !next_lanelets.empty();
}

bool PathHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet & next_lanelet)
{
  lanelet::ConstLanelets next_lanelets{};
  if (getNextLaneletsWithinRoute(lanelet, next_lanelets)) {
    next_lanelet = next_lanelets.front();
    return true;
  }
  return false;
}

bool PathHandler::getPreviousLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & prev_lanelets)
{
  if (!routing_graph_ptr_) {
    return false;
  }

  if (exists(start_lanelets_, lanelet)) {
    return false;
  }

  prev_lanelets.clear();
  for (const auto & lanelet : routing_graph_ptr_->previous(lanelet)) {
    if (exists(route_lanelets_, lanelet)) {
      prev_lanelets.push_back(lanelet);
    }
  }

  return !prev_lanelets.empty();
}
}  // namespace autoware::path_generator
