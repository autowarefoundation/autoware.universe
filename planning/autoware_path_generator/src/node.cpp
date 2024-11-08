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

namespace autoware::path_generator
{
PathGenerator::PathGenerator(const rclcpp::NodeOptions & node_options)
: Node("path_generator", node_options)
{
  param_listener_ =
    std::make_shared<::path_generator::ParamListener>(this->get_node_parameters_interface());

  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);

  {
    const auto planning_hz = declare_parameter<double>("planning_hz");
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(planning_hz).period(), std::bind(&PathGenerator::run, this));
  }
}

void PathGenerator::run()
{
  const auto input_data = take_data();
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

bool PathGenerator::is_data_ready(const InputData & input_data)
{
  const auto missing = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
    return false;
  };

  if (!input_data.lanelet_map_bin_ptr && !planner_data_.lanelet_map_ptr) {
    return missing("map");
  }

  if (!input_data.route_ptr && !planner_data_.route_ptr) {
    return missing("route");
  }

  if (!input_data.odometry_ptr) {
    return missing("odometry");
  }

  return true;
}

std::optional<PathWithLaneId> PathGenerator::plan_path(const InputData & input_data)
{
  if (input_data.lanelet_map_bin_ptr) {
    lanelet::utils::conversion::fromBinMsg(
      *input_data.lanelet_map_bin_ptr, planner_data_.lanelet_map_ptr,
      &planner_data_.traffic_rules_ptr, &planner_data_.routing_graph_ptr);
  }

  if (input_data.route_ptr) {
    set_route(input_data.route_ptr);
  }

  const auto path = generate_centerline_path(
    input_data.route_ptr, input_data.odometry_ptr->pose.pose, param_listener_->get_params());

  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return std::nullopt;
  } else if (path->points.empty()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is empty");
    return std::nullopt;
  }

  return path;
}

void PathGenerator::set_route(const LaneletRoute::ConstSharedPtr & route_ptr)
{
  planner_data_.route_ptr = route_ptr;
  if (route_ptr->segments.empty()) {
    return;
  }

  planner_data_.route_lanelets.clear();
  planner_data_.preferred_lanelets.clear();
  planner_data_.start_lanelets.clear();
  planner_data_.goal_lanelets.clear();

  size_t primitives_num = 0;
  for (const auto & route_section : planner_data_.route_ptr->segments) {
    primitives_num += route_section.primitives.size();
  }
  planner_data_.route_lanelets.reserve(primitives_num);

  for (const auto & route_section : planner_data_.route_ptr->segments) {
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
  set_lanelets_from_segment(
    planner_data_.route_ptr->segments.front(), planner_data_.start_lanelets);
  set_lanelets_from_segment(planner_data_.route_ptr->segments.back(), planner_data_.goal_lanelets);
}

std::optional<PathWithLaneId> PathGenerator::generate_centerline_path(
  const LaneletRoute::ConstSharedPtr & route_ptr, const geometry_msgs::msg::Pose & current_pose,
  const Params & params) const
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

  const auto centerline_path =
    get_centerline_path(*lanelet_sequence, route_ptr, current_pose, params);
  if (!centerline_path) {
    return std::nullopt;
  }

  return centerline_path;
}

std::optional<PathWithLaneId> PathGenerator::get_centerline_path(
  const lanelet::ConstLanelets & lanelet_sequence, const LaneletRoute::ConstSharedPtr & route_ptr,
  const geometry_msgs::msg::Pose & current_pose, const Params & params) const
{
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  if (!route_ptr) {
    return std::nullopt;
  }

  const auto arc_coordinates = lanelet::utils::getArcCoordinates(lanelet_sequence, current_pose);
  const auto s = arc_coordinates.length;  // s denotes longitudinal position in Frenet coordinates
  const auto s_start = std::max(0., s - params.backward_path_length);
  const auto s_end = [&]() {
    auto s_end = s + params.forward_path_length;

    if (!utils::get_next_lanelet_within_route(lanelet_sequence.back(), planner_data_)) {
      const auto lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
      s_end = std::clamp(s_end, 0.0, lane_length);
    }

    if (
      std::find(
        planner_data_.goal_lanelets.begin(), planner_data_.goal_lanelets.end(),
        lanelet_sequence.back()) != planner_data_.goal_lanelets.end()) {
      const auto goal_arc_coordinates =
        lanelet::utils::getArcCoordinates(lanelet_sequence, planner_data_.route_ptr->goal_pose);
      s_end = std::clamp(s_end, 0.0, goal_arc_coordinates.length);
    }

    return s_end;
  }();

  const auto raw_centerline_path = get_centerline_path(lanelet_sequence, s_start, s_end);
  if (!raw_centerline_path) {
    return std::nullopt;
  }

  auto centerline_path = autoware::motion_utils::resamplePath(
    *raw_centerline_path, params.input_path_interval, params.enable_akima_spline_first);

  const auto current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      centerline_path.points, current_pose, params.ego_nearest_dist_threshold,
      params.ego_nearest_yaw_threshold);

  centerline_path.points = autoware::motion_utils::cropPoints(
    centerline_path.points, current_pose.position, current_seg_idx, params.forward_path_length,
    params.backward_path_length + params.input_path_interval);

  return centerline_path;
}

std::optional<PathWithLaneId> PathGenerator::get_centerline_path(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end) const
{
  if (
    !planner_data_.lanelet_map_ptr || !planner_data_.traffic_rules_ptr ||
    !planner_data_.route_ptr) {
    return std::nullopt;
  }

  PathWithLaneId centerline_path{};
  auto & path_points = centerline_path.points;

  const auto waypoint_groups =
    utils::get_waypoint_groups(lanelet_sequence, *planner_data_.lanelet_map_ptr);

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

    const auto speed_limit =
      planner_data_.traffic_rules_ptr->speedLimit(lanelet).speedLimit.value();
    for (const auto & reference_point : reference_points) {
      PathPointWithLaneId path_point{};
      path_point.point.pose.position = reference_point;
      path_point.lane_ids.push_back(lanelet.id());
      path_point.point.longitudinal_velocity_mps = static_cast<float>(speed_limit);
      path_points.push_back(path_point);
    }
  }

  utils::remove_overlapping_points(centerline_path);

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

  centerline_path.header = planner_data_.route_ptr->header;
  return centerline_path;
}
}  // namespace autoware::path_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGenerator)
