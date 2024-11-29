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

#include "node.hpp"

#include "autoware/path_generator/utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <typename T, typename U>
double get_arc_length_along_centerline(const T & lanelet, const U & point)
{
  return lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), lanelet::utils::to2D(point))
    .length;
}
}  // namespace

namespace autoware::path_generator
{
PathGenerator::PathGenerator(const rclcpp::NodeOptions & node_options)
: Node("path_generator", node_options)
{
  param_listener_ =
    std::make_shared<::path_generator::ParamListener>(this->get_node_parameters_interface());

  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);

  const auto params = param_listener_->get_params();
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params.planning_hz).period(),
    std::bind(&PathGenerator::run, this));
}

void PathGenerator::run()
{
  const auto input_data = take_data();
  set_planner_data(input_data);
  if (!is_data_ready(input_data)) {
    return;
  }

  const auto path = plan_path(input_data);
  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return;
  }

  path_publisher_->publish(*path);
}

PathGenerator::InputData PathGenerator::take_data()
{
  InputData input_data;

  // route
  if (const auto msg = route_subscriber_.takeData()) {
    if (msg->segments.empty()) {
      RCLCPP_ERROR(get_logger(), "input route is empty, ignoring...");
    } else {
      input_data.route_ptr = msg;
    }
  }

  // map
  if (const auto msg = vector_map_subscriber_.takeData()) {
    input_data.lanelet_map_bin_ptr = msg;
  }

  // velocity
  if (const auto msg = odometry_subscriber_.takeData()) {
    input_data.odometry_ptr = msg;
  }

  return input_data;
}

void PathGenerator::set_planner_data(const InputData & input_data)
{
  if (input_data.lanelet_map_bin_ptr) {
    planner_data_.lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
      *input_data.lanelet_map_bin_ptr, planner_data_.lanelet_map_ptr,
      &planner_data_.traffic_rules_ptr, &planner_data_.routing_graph_ptr);
  }

  if (input_data.route_ptr) {
    set_route(input_data.route_ptr);
  }
}

void PathGenerator::set_route(const LaneletRoute::ConstSharedPtr & route_ptr)
{
  planner_data_.route_frame_id = route_ptr->header.frame_id;
  planner_data_.goal_pose = route_ptr->goal_pose;

  planner_data_.route_lanelets.clear();
  planner_data_.preferred_lanelets.clear();
  planner_data_.start_lanelets.clear();
  planner_data_.goal_lanelets.clear();

  size_t primitives_num = 0;
  for (const auto & route_section : route_ptr->segments) {
    primitives_num += route_section.primitives.size();
  }
  planner_data_.route_lanelets.reserve(primitives_num);

  for (const auto & route_section : route_ptr->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(id);
      planner_data_.route_lanelets.push_back(lanelet);
      if (id == route_section.preferred_primitive.id) {
        planner_data_.preferred_lanelets.push_back(lanelet);
      }
    }
  }

  const auto set_lanelets_from_segment =
    [&](
      const autoware_planning_msgs::msg::LaneletSegment & segment,
      lanelet::ConstLanelets & lanelets) {
      lanelets.reserve(segment.primitives.size());
      for (const auto & primitive : segment.primitives) {
        const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(primitive.id);
        lanelets.push_back(lanelet);
      }
    };
  set_lanelets_from_segment(route_ptr->segments.front(), planner_data_.start_lanelets);
  set_lanelets_from_segment(route_ptr->segments.back(), planner_data_.goal_lanelets);
}

bool PathGenerator::is_data_ready(const InputData & input_data)
{
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  if (!planner_data_.lanelet_map_ptr) {
    notify_waiting("map");
    return false;
  }

  if (planner_data_.route_lanelets.empty()) {
    notify_waiting("route");
    return false;
  }

  if (!input_data.odometry_ptr) {
    notify_waiting("odometry");
    return false;
  }

  return true;
}

std::optional<PathWithLaneId> PathGenerator::plan_path(const InputData & input_data)
{
  const auto path =
    generate_path(input_data.odometry_ptr->pose.pose, param_listener_->get_params());

  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return std::nullopt;
  } else if (path->points.empty()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is empty");
    return std::nullopt;
  }

  return path;
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const geometry_msgs::msg::Pose & current_pose, const Params & params) const
{
  lanelet::ConstLanelet current_lane;
  if (!lanelet::utils::query::getClosestLanelet(
        planner_data_.preferred_lanelets, current_pose, &current_lane)) {
    return std::nullopt;
  }

  const auto lanelets = utils::get_lanelets_within_route(
    current_lane, planner_data_, current_pose, params.backward_path_length,
    params.forward_path_length);
  if (!lanelets) {
    return std::nullopt;
  }

  return generate_path(*lanelets, current_pose, params);
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & current_pose,
  const Params & params) const
{
  if (lanelets.empty()) {
    return std::nullopt;
  }

  const auto arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const auto s = arc_coordinates.length;  // s denotes longitudinal position in Frenet coordinates
  const auto s_start = std::max(0., s - params.backward_path_length);
  const auto s_end = [&]() {
    auto s_end = s + params.forward_path_length;

    if (!utils::get_next_lanelet_within_route(lanelets.back(), planner_data_)) {
      s_end = std::min(s_end, lanelet::utils::getLaneletLength2d(lanelets));
    }

    if (std::any_of(
          planner_data_.goal_lanelets.begin(), planner_data_.goal_lanelets.end(),
          [&](const auto & goal_lanelet) { return lanelets.back().id() == goal_lanelet.id(); })) {
      const auto goal_arc_coordinates =
        lanelet::utils::getArcCoordinates(lanelets, planner_data_.goal_pose);
      s_end = std::min(s_end, goal_arc_coordinates.length);
    }

    return s_end;
  }();

  return generate_path(lanelets, s_start, s_end, params);
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const lanelet::ConstLanelets & lanelets, const double s_start, const double s_end,
  const Params & params) const
{
  std::vector<PathPointWithLaneId> path_points_with_lane_id{};

  const auto add_path_point = [&](const auto & path_point, const lanelet::ConstLanelet & lanelet) {
    PathPointWithLaneId path_point_with_lane_id{};
    path_point_with_lane_id.lane_ids.push_back(lanelet.id());
    path_point_with_lane_id.point.pose.position =
      lanelet::utils::conversion::toGeomMsgPt(path_point);
    path_point_with_lane_id.point.longitudinal_velocity_mps =
      planner_data_.traffic_rules_ptr->speedLimit(lanelet).speedLimit.value();
    path_points_with_lane_id.push_back(std::move(path_point_with_lane_id));
  };

  const auto waypoint_groups = utils::get_waypoint_groups(
    lanelets, *planner_data_.lanelet_map_ptr, params.waypoint_group_separation_threshold,
    params.waypoint_group_interval_margin_ratio);

  auto extended_lanelets = lanelets;
  auto s_offset = 0.;

  for (const auto & [waypoints, interval] : waypoint_groups) {
    if (interval.first > 0.) {
      continue;
    }
    const auto prev_lanelet =
      utils::get_previous_lanelet_within_route(lanelets.front(), planner_data_);
    if (!prev_lanelet) {
      break;
    }
    extended_lanelets.insert(extended_lanelets.begin(), *prev_lanelet);
    s_offset = lanelet::geometry::length2d(*prev_lanelet);
    break;
  }

  const lanelet::LaneletSequence extended_lanelet_sequence(extended_lanelets);
  std::optional<size_t> overlapping_waypoint_group_index = std::nullopt;

  for (auto lanelet_it = extended_lanelet_sequence.begin();
       lanelet_it != extended_lanelet_sequence.end(); ++lanelet_it) {
    const auto & centerline = lanelet_it->centerline();
    auto s = get_arc_length_along_centerline(extended_lanelet_sequence, centerline.front());

    for (auto point_it = centerline.begin(); point_it != centerline.end(); ++point_it) {
      if (point_it != centerline.begin()) {
        s += lanelet::geometry::distance2d(*std::prev(point_it), *point_it);
      } else if (lanelet_it != extended_lanelet_sequence.begin()) {
        continue;
      }

      if (overlapping_waypoint_group_index) {
        const auto & [waypoints, interval] = waypoint_groups[*overlapping_waypoint_group_index];
        if (s >= interval.first + s_offset && s <= interval.second + s_offset) {
          continue;
        }
        overlapping_waypoint_group_index = std::nullopt;
      }

      for (size_t i = 0; i < waypoint_groups.size(); ++i) {
        const auto & [waypoints, interval] = waypoint_groups[i];
        if (s < interval.first + s_offset || s > interval.second + s_offset) {
          continue;
        }
        for (const auto & waypoint : waypoints) {
          const auto s_waypoint =
            get_arc_length_along_centerline(extended_lanelet_sequence, waypoint);
          for (auto waypoint_lanelet_it = extended_lanelet_sequence.begin();
               waypoint_lanelet_it != extended_lanelet_sequence.end(); ++waypoint_lanelet_it) {
            if (
              s_waypoint > get_arc_length_along_centerline(
                             extended_lanelet_sequence, waypoint_lanelet_it->centerline().back())) {
              continue;
            }
            add_path_point(waypoint, *waypoint_lanelet_it);
            break;
          }
        }
        overlapping_waypoint_group_index = i;
        break;
      }

      if (overlapping_waypoint_group_index) {
        continue;
      }
      add_path_point(*point_it, *lanelet_it);
    }
  }

  s_offset -= get_arc_length_along_centerline(
    extended_lanelet_sequence, lanelet::utils::conversion::toLaneletPoint(
                                 path_points_with_lane_id.front().point.pose.position));

  auto trajectory = Trajectory::Builder().build(path_points_with_lane_id);
  if (!trajectory) {
    return std::nullopt;
  }

  trajectory->crop(s_offset + s_start, s_end - s_start);

  PathWithLaneId path{};
  path.header.frame_id = planner_data_.route_frame_id;
  path.header.stamp = now();
  path.points = trajectory->restore();

  for (const auto & left_bound_point : extended_lanelet_sequence.leftBound()) {
    path.left_bound.push_back(lanelet::utils::conversion::toGeomMsgPt(left_bound_point));
  }
  for (const auto & right_bound_point : extended_lanelet_sequence.rightBound()) {
    path.right_bound.push_back(lanelet::utils::conversion::toGeomMsgPt(right_bound_point));
  }

  return path;
}
}  // namespace autoware::path_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGenerator)
