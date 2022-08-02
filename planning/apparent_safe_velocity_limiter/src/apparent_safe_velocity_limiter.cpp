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

#include "apparent_safe_velocity_limiter/apparent_safe_velocity_limiter.hpp"

#include "apparent_safe_velocity_limiter/distance.hpp"
#include "apparent_safe_velocity_limiter/forward_projection.hpp"
#include "apparent_safe_velocity_limiter/occupancy_grid_utils.hpp"
#include "apparent_safe_velocity_limiter/pointcloud_utils.hpp"
#include "apparent_safe_velocity_limiter/types.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

namespace apparent_safe_velocity_limiter
{

Float calculateSafeVelocity(
  const TrajectoryPoint & trajectory_point, const Float dist_to_collision, const Float time_buffer,
  const Float min_adjusted_velocity)
{
  return std::min(
    trajectory_point.longitudinal_velocity_mps,
    std::max(min_adjusted_velocity, static_cast<Float>(dist_to_collision / time_buffer)));
}

size_t calculateStartIndex(
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

Trajectory downsampleTrajectory(
  const Trajectory & trajectory, const size_t start_idx, const int factor)
{
  if (factor < 1) return trajectory;
  Trajectory downsampled_traj;
  downsampled_traj.header = trajectory.header;
  downsampled_traj.points.reserve(trajectory.points.size() / factor);
  for (size_t i = start_idx; i < trajectory.points.size(); i += factor)
    downsampled_traj.points.push_back(trajectory.points[i]);
  return downsampled_traj;
}

void calculateSteeringAngles(Trajectory & trajectory, const Float wheel_base)
{
  auto t = 0.0;
  auto prev_point = trajectory.points.front();
  auto prev_heading = tf2::getYaw(prev_point.pose.orientation);
  for (auto i = 1ul; i < trajectory.points.size(); ++i) {
    const auto & prev_point = trajectory.points[i - 1];
    auto & point = trajectory.points[i];
    const auto dt = tier4_autoware_utils::calcDistance2d(prev_point, point) /
                    prev_point.longitudinal_velocity_mps;
    t += dt;
    const auto heading = tf2::getYaw(point.pose.orientation);
    const auto d_heading = heading - prev_heading;
    prev_heading = heading;
    point.front_wheel_angle_rad =
      std::atan2(wheel_base * d_heading, point.longitudinal_velocity_mps * dt);
  }
}

multipolygon_t createPolygonMasks(
  const autoware_auto_perception_msgs::msg::PredictedObjects & dynamic_obstacles,
  const Float buffer, const Float min_vel)
{
  return createObjectPolygons(dynamic_obstacles, buffer, min_vel);
}

std::vector<polygon_t> createFootprintPolygons(
  const std::vector<multilinestring_t> & projected_linestrings, const Float lateral_offset)
{
  std::vector<polygon_t> footprints;
  footprints.reserve(projected_linestrings.size());
  for (const auto & linestrings : projected_linestrings) {
    footprints.push_back(generateFootprint(linestrings, lateral_offset));
  }
  return footprints;
}

polygon_t createEnvelopePolygon(
  const Trajectory & trajectory, const size_t start_idx, ProjectionParameters & projection_params)
{
  polygon_t envelope_polygon;
  const auto trajectory_size = trajectory.points.size() - start_idx;
  if (trajectory_size < 2) return envelope_polygon;

  envelope_polygon.outer().resize(trajectory_size * 2 + 1);
  for (size_t i = 0; i < trajectory_size; ++i) {
    const auto & point = trajectory.points[i + start_idx];
    projection_params.update(point);
    const auto forward_simulated_vector =
      forwardSimulatedSegment(point.pose.position, projection_params);
    envelope_polygon.outer()[i] = forward_simulated_vector.second;
    const auto reverse_index = 2 * trajectory_size - i - 1;
    envelope_polygon.outer()[reverse_index] = forward_simulated_vector.first;
  }
  envelope_polygon.outer().push_back(envelope_polygon.outer().front());
  boost::geometry::correct(envelope_polygon);
  return envelope_polygon;
}

polygon_t createEnvelopePolygon(const std::vector<polygon_t> & footprints)
{
  multipolygon_t unions;
  multipolygon_t result;
  for (const auto & footprint : footprints) {
    boost::geometry::union_(footprint, unions, result);
    unions = result;
    boost::geometry::clear(result);
  }
  if (unions.empty()) return {};
  return unions.front();
}

std::vector<multilinestring_t> createProjectedLines(
  const Trajectory & trajectory, ProjectionParameters & params)
{
  std::vector<multilinestring_t> projections;
  projections.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    params.update(point);
    if (params.model == ProjectionParameters::PARTICLE) {
      const auto projection = forwardSimulatedSegment(point.pose.position, params);
      projections.push_back({{projection.first, projection.second}});
    } else {
      projections.push_back(bicycleProjectionLines(point.pose.position, params));
    }
  }
  return projections;
}

std::vector<Obstacle> createObstacles(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, const PointCloud & pointcloud,
  const multipolygon_t & polygon_masks, const polygon_t & envelope_polygon,
  tier4_autoware_utils::TransformListener & transform_listener, const std::string & target_frame,
  const ObstacleParameters & obstacle_params, PointCloud & debug_pointcloud)
{
  std::vector<Obstacle> obstacles;
  if (obstacle_params.dynamic_source == ObstacleParameters::OCCUPANCYGRID) {
    auto grid_map = convertToGridMap(occupancy_grid);
    threshold(grid_map, obstacle_params.occupancy_grid_threshold);
    obstacles = extractObstacles(grid_map, occupancy_grid);
  } else {
    const auto filtered_pcd = transformPointCloud(pointcloud, transform_listener, target_frame);
    obstacles = extractObstacles(filtered_pcd, obstacle_params.pcd_cluster_max_dist);
  }
  std::vector<Obstacle> filtered_obstacles;
  for (auto & obstacle : obstacles) {
    if (
      !boost::geometry::within(obstacle.line, polygon_masks) &&
      (!obstacle_params.filter_envelope ||
       boost::geometry::within(obstacle.line, envelope_polygon)))
      filtered_obstacles.push_back(std::move(obstacle));
  }
  return filtered_obstacles;
}

void limitVelocity(
  Trajectory & trajectory, const std::vector<Obstacle> & obstacles,
  const std::vector<multilinestring_t> & projections, const std::vector<polygon_t> & footprints,
  ProjectionParameters & projection_params, const VelocityParameters & velocity_params,
  const bool filter_envelope)
{
  constexpr auto no_filter_envelope = std::optional<double>();
  Float time = 0.0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    auto & trajectory_point = trajectory.points[i];
    if (i > 0) {
      const auto & prev_point = trajectory.points[i - 1];
      time += static_cast<Float>(
        tier4_autoware_utils::calcDistance2d(prev_point, trajectory_point) /
        prev_point.longitudinal_velocity_mps);
    }
    // First linestring is used to calculate distance
    if (projections[i].empty()) continue;
    projection_params.update(trajectory_point);
    const auto dist_to_collision = distanceToClosestCollision(
      projections[i][0], footprints[i], obstacles, projection_params,
      filter_envelope ? no_filter_envelope
                      : trajectory_point.longitudinal_velocity_mps * projection_params.duration +
                          projection_params.extra_length);
    if (dist_to_collision) {
      const auto min_feasible_velocity =
        velocity_params.current_ego_velocity - velocity_params.max_deceleration * time;
      trajectory_point.longitudinal_velocity_mps = std::max(
        min_feasible_velocity,
        calculateSafeVelocity(
          trajectory_point, static_cast<Float>(*dist_to_collision - projection_params.extra_length),
          projection_params.duration, velocity_params.min_velocity));
    }
  }
}

Trajectory copyDownsampledVelocity(
  const Trajectory & downsampled_traj, Trajectory trajectory, const size_t start_idx,
  const int factor)
{
  const auto size = std::min(downsampled_traj.points.size(), trajectory.points.size());
  for (size_t i = 0; i < size; ++i) {
    trajectory.points[start_idx + i * factor].longitudinal_velocity_mps =
      downsampled_traj.points[i].longitudinal_velocity_mps;
  }
  return trajectory;
}
}  // namespace apparent_safe_velocity_limiter
