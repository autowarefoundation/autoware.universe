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
#include <vector>

namespace
{
template <typename T, typename U>
double get_arc_length_along_centerline(const T & lanelet, const U & point)
{
  return lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), lanelet::utils::to2D(point))
    .length;
}

template <typename T, typename U>
lanelet::BasicPoint3d get_interpolated_point(const T & start, const U & end, const double distance)
{
  lanelet::Point3d start_point, end_point;
  start_point.x() = start.x();
  start_point.y() = start.y();
  start_point.z() = start.z();
  end_point.x() = end.x();
  end_point.y() = end.y();
  end_point.z() = end.z();

  return lanelet::geometry::interpolatedPointAtDistance(
    lanelet::ConstLineString3d(lanelet::InvalId, lanelet::Points3d{start_point, end_point}),
    distance);
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

  const auto lanelet_sequence = utils::get_lanelet_sequence(
    current_lane, planner_data_, current_pose, params.forward_path_length,
    params.backward_path_length);
  if (!lanelet_sequence) {
    return std::nullopt;
  }

  return generate_path(*lanelet_sequence, current_pose, params);
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const lanelet::LaneletSequence & lanelet_sequence, const geometry_msgs::msg::Pose & current_pose,
  const Params & params) const
{
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  const auto arc_coordinates =
    lanelet::utils::getArcCoordinates(lanelet_sequence.lanelets(), current_pose);
  const auto s = arc_coordinates.length;  // s denotes longitudinal position in Frenet coordinates
  const auto s_start = std::max(0., s - params.backward_path_length);
  const auto s_end = [&]() {
    auto s_end = s + params.forward_path_length;

    if (!utils::get_next_lanelet_within_route(lanelet_sequence.lanelets().back(), planner_data_)) {
      const double lane_length = lanelet::geometry::length(lanelet_sequence.centerline2d());
      s_end = std::min(s_end, lane_length);
    }

    if (std::any_of(
          planner_data_.goal_lanelets.begin(), planner_data_.goal_lanelets.end(),
          [&](const auto & goal_lanelet) {
            return lanelet_sequence.lanelets().back().id() == goal_lanelet.id();
          })) {
      const auto goal_arc_coordinates =
        lanelet::utils::getArcCoordinates(lanelet_sequence.lanelets(), planner_data_.goal_pose);
      s_end = std::min(s_end, goal_arc_coordinates.length);
    }

    return s_end;
  }();

  auto path = generate_path(lanelet_sequence, s_start, s_end, params);
  if (!path) {
    return std::nullopt;
  }

  path = autoware::motion_utils::resamplePath(
    *path, params.input_path_interval, params.enable_akima_spline_first);

  const auto current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path->points, current_pose, params.ego_nearest_dist_threshold,
      params.ego_nearest_yaw_threshold);

  path->points = autoware::motion_utils::cropPoints(
    path->points, current_pose.position, current_seg_idx, params.forward_path_length,
    params.backward_path_length + params.input_path_interval);

  return path;
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const Params & params) const
{
  auto path_points = generate_path_points(lanelet_sequence, s_start, s_end, params);
  if (path_points.empty()) {
    return std::nullopt;
  }

  // append a point if having only one point so that yaw calculation would work
  if (path_points.size() == 1) {
    const auto & lane_id = path_points.front().lane_ids.front();
    const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(lane_id);
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

  PathWithLaneId path{};
  path.header.frame_id = planner_data_.route_frame_id;
  path.header.stamp = now();
  path.points = std::move(path_points);

  for (const auto & left_bound_point : lanelet_sequence.leftBound()) {
    path.left_bound.push_back(lanelet::utils::conversion::toGeomMsgPt(left_bound_point));
  }
  for (const auto & right_bound_point : lanelet_sequence.rightBound()) {
    path.right_bound.push_back(lanelet::utils::conversion::toGeomMsgPt(right_bound_point));
  }

  return path;
}

std::vector<PathPointWithLaneId> PathGenerator::generate_path_points(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const Params & params) const
{
  lanelet::ConstPoints3d path_points = lanelet_sequence.centerline().constData()->points();

  auto waypoint_groups = utils::get_waypoint_groups(
    lanelet_sequence, *planner_data_.lanelet_map_ptr, params.waypoint_group_separation_threshold,
    params.waypoint_group_interval_margin_ratio);

  for (const auto & [waypoints, interval] : waypoint_groups) {
    const auto overlap_start_it = std::find_if(
      path_points.begin(), path_points.end(), [&](const lanelet::BasicPoint3d & point) {
        return get_arc_length_along_centerline(lanelet_sequence, point) >= interval.first;
      });
    const auto overlap_end_it =
      std::find_if(overlap_start_it, path_points.end(), [&](const lanelet::BasicPoint3d & point) {
        return get_arc_length_along_centerline(lanelet_sequence, point) > interval.second;
      });
    if (overlap_start_it == overlap_end_it) {
      continue;
    }
    const auto waypoint_start_it = path_points.erase(overlap_start_it, overlap_end_it);
    path_points.insert(waypoint_start_it, waypoints.begin(), waypoints.end());
  }

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

  for (auto it = path_points.begin(); it != path_points.end(); ++it) {
    const auto & path_point = *it;

    const auto lanelet_it = std::find_if(
      lanelet_sequence.begin(), lanelet_sequence.end(), [&](const lanelet::ConstLanelet & lanelet) {
        return get_arc_length_along_centerline(lanelet_sequence, path_point) >=
               get_arc_length_along_centerline(lanelet_sequence, lanelet.centerline().front());
      });
    if (lanelet_it == lanelet_sequence.end()) {
      continue;
    }

    const auto s = get_arc_length_along_centerline(lanelet_sequence, path_point);

    if (s < s_start) {
      if (it == std::prev(path_points.end())) {
        break;
      }
      const auto & next_path_point = *std::next(it);
      const auto s_next = get_arc_length_along_centerline(lanelet_sequence, next_path_point);
      if (s_next > s_start) {
        add_path_point(
          get_interpolated_point(path_point, next_path_point, s_start - s), *lanelet_it);
      }
      continue;
    }

    if (s > s_end) {
      if (it == path_points.begin()) {
        break;
      }
      const auto & prev_path_point = *std::prev(it);
      add_path_point(get_interpolated_point(path_point, prev_path_point, s - s_end), *lanelet_it);
      break;
    }

    add_path_point(path_point, *lanelet_it);
  }

  utils::remove_overlapping_points(path_points_with_lane_id);

  return path_points_with_lane_id;
}
}  // namespace autoware::path_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGenerator)
