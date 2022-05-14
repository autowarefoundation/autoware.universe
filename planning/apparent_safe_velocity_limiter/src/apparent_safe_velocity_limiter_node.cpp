// Copyright 2022 Tier IV, Inc.
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

#include "apparent_safe_velocity_limiter/apparent_safe_velocity_limiter_node.hpp"
#include <numeric>

namespace apparent_safe_velocity_limiter
{
ApparentSafeVelocityLimiterNode::ApparentSafeVelocityLimiterNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("apparent_safe_velocity_limiter", node_options)
{
  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, [this](const Trajectory::ConstSharedPtr msg) { onTrajectory(msg); });
  sub_occupancy_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "~/input/occupancy_grid", 1,
    [this](const OccupancyGrid::ConstSharedPtr msg) { occupancy_grid_ptr_ = msg; });
  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/dynamic_obstacles", 1,
    [this](const PredictedObjects::ConstSharedPtr msg) { dynamic_obstacles_ptr_ = msg; });

  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_debug_markers_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_markers", 1);
  pub_debug_occupancy_grid_ = create_publisher<OccupancyGrid>("~/output/occupancy_grid", 1);

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  vehicle_lateral_offset_ = static_cast<Float>(vehicle_info.max_lateral_offset_m);
  vehicle_front_offset_ = static_cast<Float>(vehicle_info.max_longitudinal_offset_m);

  set_param_res_ =
    add_on_set_parameters_callback([this](const auto & params) { return onParameter(params); });
  self_pose_listener_.waitForFirstPose();
}

rcl_interfaces::msg::SetParametersResult ApparentSafeVelocityLimiterNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "time_buffer") {
      time_buffer_ = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == "distance_buffer") {
      distance_buffer_ = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == "start_distance") {
      start_distance_ = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == "downsample_factor") {
      downsample_factor_ = static_cast<int>(parameter.as_int());
    } else if (parameter.get_name() == "occupancy_grid_obstacle_threshold") {
      occupancy_grid_obstacle_threshold_ = static_cast<int8_t>(parameter.as_int());
    } else if (parameter.get_name() == "dynamic_obstacles_buffer") {
      dynamic_obstacles_buffer_ = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == "dynamic_obstacles_min_vel") {
      dynamic_obstacles_min_vel_ = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == "min_adjusted_velocity") {
      min_adjusted_velocity_ = static_cast<Float>(parameter.as_double());
    } else {
      RCLCPP_WARN(get_logger(), "Unknown parameter %s", parameter.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

void ApparentSafeVelocityLimiterNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  const auto ego_idx =
    tier4_autoware_utils::findNearestIndex(msg->points, self_pose_listener_.getCurrentPose()->pose);
  if (!occupancy_grid_ptr_ || !dynamic_obstacles_ptr_ || !ego_idx) {
    constexpr auto one_sec = rcutils_duration_value_t(1000);
    if (!occupancy_grid_ptr_)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), one_sec, "Occupancy grid not yet received");
    if (!dynamic_obstacles_ptr_)
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), one_sec, "Dynamic obstable not yet received");
    if (!ego_idx)
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), one_sec, "Cannot calculate ego index on the trajectory");
    return;
  }
  const auto extra_vehicle_length = vehicle_front_offset_ + distance_buffer_;
  const auto start_idx = calculateStartIndex(*msg, *ego_idx, start_distance_);

  // Downsample trajectory
  const size_t downsample_step = downsample_factor_;
  Trajectory downsampled_traj;
  downsampled_traj.header = msg->header;
  downsampled_traj.points.reserve(msg->points.size() / downsample_step);
  for (size_t i = start_idx; i < msg->points.size(); i += downsample_step)
    downsampled_traj.points.push_back(msg->points[i]);

  // TODO(Maxime CLEMENT): used for debugging, remove before merging
  double obs_poly_duration{};
  double obs_footprint_duration{};
  double obs_envelope_duration{};
  double obs_filter_duration{};
  double footprint_duration{};
  double dist_poly_duration{};
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stopwatch;

  // Obstacle Polygon from Occupancy Grid
  nav_msgs::msg::OccupancyGrid debug_occ_grid;
  stopwatch.tic("obs_poly_duration");
  auto polygon_masks = createObjectPolygons(
    *dynamic_obstacles_ptr_, dynamic_obstacles_buffer_, dynamic_obstacles_min_vel_);
  linestring_t trajectory_linestring;
  for (size_t i = start_idx; i < msg->points.size(); ++i) {
    trajectory_linestring.push_back(
      {msg->points[i].pose.position.x, msg->points[i].pose.position.y});
  }
  obs_poly_duration = stopwatch.toc("obs_poly_duration");
  stopwatch.tic("obs_footprint_duration");
  polygon_masks.push_back(generateFootprint(trajectory_linestring, vehicle_lateral_offset_));
  obs_footprint_duration += stopwatch.toc("obs_footprint_duration");
  // Calculate envelope polygon
  stopwatch.tic("obs_envelope_duration");
  polygon_t envelope_polygon;
  for (const auto & point : trajectory_linestring) envelope_polygon.outer().push_back(point);
  for (auto it = msg->points.rbegin(); it != msg->points.rend(); ++it) {
    const auto forward_simulated_vector =
      forwardSimulatedSegment(*it, time_buffer_, extra_vehicle_length);
    envelope_polygon.outer().push_back(forward_simulated_vector.second);
  }
  bg::correct(envelope_polygon);
  obs_envelope_duration += stopwatch.toc("obs_envelope_duration");
  stopwatch.tic("obs_filter_duration");
  // Filter obstacles outside the envelope polygon
  const auto obstacles = extractStaticObstaclePolygons(
    *occupancy_grid_ptr_, polygon_masks, envelope_polygon, occupancy_grid_obstacle_threshold_);
  obs_filter_duration += stopwatch.toc("obs_filter_duration");
  pub_debug_occupancy_grid_->publish(debug_occ_grid);

  Trajectory safe_trajectory = *msg;
  for (auto & trajectory_point : downsampled_traj.points) {
    const auto forward_simulated_vector =
      forwardSimulatedSegment(trajectory_point, time_buffer_, extra_vehicle_length);
    stopwatch.tic("footprint_duration");
    const auto footprint = generateFootprint(forward_simulated_vector, vehicle_lateral_offset_);
    footprint_duration += stopwatch.toc("footprint_duration");
    stopwatch.tic("dist_poly_duration");
    const auto dist_to_collision =
      distanceToClosestCollision(forward_simulated_vector, footprint, obstacles);
    dist_poly_duration += stopwatch.toc("dist_poly_duration");
    if (dist_to_collision) {
      trajectory_point.longitudinal_velocity_mps = calculateSafeVelocity(
        trajectory_point, static_cast<Float>(*dist_to_collision - extra_vehicle_length));
    }
  }

  for (size_t i = 0; i < downsampled_traj.points.size(); ++i) {
    safe_trajectory.points[start_idx + i * downsample_step].longitudinal_velocity_mps =
      downsampled_traj.points[i].longitudinal_velocity_mps;
  }

  safe_trajectory.header.stamp = now();
  pub_trajectory_->publish(safe_trajectory);
  publishDebugMarkers(*msg, safe_trajectory, obstacles);
  // TODO(Maxime CLEMENT): removes or change to RCLCPP_DEBUG before merging
  RCLCPP_WARN(get_logger(), "Runtimes");
  RCLCPP_WARN(get_logger(), "  obstacles poly       = %2.2fms", obs_poly_duration);
  RCLCPP_WARN(get_logger(), "  obstacles footprint  = %2.2fms", obs_footprint_duration);
  RCLCPP_WARN(get_logger(), "  obstacles envelope   = %2.2fms", obs_envelope_duration);
  RCLCPP_WARN(get_logger(), "  obstacles filter     = %2.2fms", obs_filter_duration);
  RCLCPP_WARN(get_logger(), "  footprint generation = %2.2fms", footprint_duration);
  RCLCPP_WARN(get_logger(), "  distance to obstacle = %2.2fms", dist_poly_duration);
  const auto runtime = stopwatch.toc();
  runtimes.insert(runtime);
  const auto sum = std::accumulate(runtimes.begin(), runtimes.end(), 0.0);
  RCLCPP_WARN(get_logger(), "**************** Total = %2.2fms", runtime);
  RCLCPP_WARN(get_logger(), "**************** Total (max) = %2.2fms", *std::prev(runtimes.end()));
  RCLCPP_WARN(get_logger(), "**************** Total (med) = %2.2fms", *std::next(runtimes.begin(), runtimes.size() / 2));
  RCLCPP_WARN(get_logger(), "**************** Total (avg) = %2.2fms", sum / runtimes.size());
}

Float ApparentSafeVelocityLimiterNode::calculateSafeVelocity(
  const TrajectoryPoint & trajectory_point, const Float & dist_to_collision) const
{
  return std::min(
    trajectory_point.longitudinal_velocity_mps,
    std::max(min_adjusted_velocity_, static_cast<Float>(dist_to_collision / time_buffer_)));
}

visualization_msgs::msg::Marker ApparentSafeVelocityLimiterNode::makePolygonMarker(
  const linestring_t & polygon, const int id) const
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.id = id;
  marker.ns = "obstacles";
  for (const auto & point : polygon) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = occupancy_grid_ptr_->info.origin.position.z;
    marker.points.push_back(p);
  }
  return marker;
}

visualization_msgs::msg::Marker ApparentSafeVelocityLimiterNode::makeEnvelopeMarker(
  const Trajectory & trajectory) const
{
  visualization_msgs::msg::Marker envelope;
  envelope.header = trajectory.header;
  envelope.type = visualization_msgs::msg::Marker::LINE_STRIP;
  envelope.scale.x = 0.1;
  envelope.color.a = 1.0;
  for (const auto & point : trajectory.points) {
    const auto vector =
      forwardSimulatedSegment(point, time_buffer_, distance_buffer_ + vehicle_front_offset_);
    geometry_msgs::msg::Point p;
    p.x = vector.second.x();
    p.y = vector.second.y();
    p.z = point.pose.position.z;
    envelope.points.push_back(p);
  }
  return envelope;
}

void ApparentSafeVelocityLimiterNode::publishDebugMarkers(
  const Trajectory & original_trajectory, const Trajectory & adjusted_trajectory,
  const multilinestring_t & polygons) const
{
  visualization_msgs::msg::MarkerArray debug_markers;
  auto original_envelope = makeEnvelopeMarker(original_trajectory);
  original_envelope.color.r = 1.0;
  original_envelope.ns = "original";
  debug_markers.markers.push_back(original_envelope);
  auto adjusted_envelope = makeEnvelopeMarker(adjusted_trajectory);
  adjusted_envelope.color.g = 1.0;
  adjusted_envelope.ns = "adjusted";
  debug_markers.markers.push_back(adjusted_envelope);

  static auto max_id = 0;
  auto id = 0;
  for (const auto & poly : polygons) {
    auto marker = makePolygonMarker(poly, id++);
    marker.header.frame_id = occupancy_grid_ptr_->header.frame_id;
    marker.header.stamp = now();
    debug_markers.markers.push_back(marker);
  }
  max_id = std::max(id, max_id);
  while (id <= max_id) {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.ns = "obstacles";
    marker.id = id++;
    debug_markers.markers.push_back(marker);
  }
  pub_debug_markers_->publish(debug_markers);
}

size_t ApparentSafeVelocityLimiterNode::calculateStartIndex(
  const Trajectory & trajectory, const size_t ego_idx, const Float start_distance)
{
  auto dist = 0.0;
  auto idx = ego_idx;
  while (idx + 1 < trajectory.points.size() && dist < start_distance) {
    dist +=
      tier4_autoware_utils::calcDistance2d(trajectory.points[idx], trajectory.points[idx + 1]);
    ++idx;
  }
  return idx;
}
}  // namespace apparent_safe_velocity_limiter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(apparent_safe_velocity_limiter::ApparentSafeVelocityLimiterNode)
