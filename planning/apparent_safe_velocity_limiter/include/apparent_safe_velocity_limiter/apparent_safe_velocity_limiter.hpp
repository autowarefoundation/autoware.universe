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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__APPARENT_SAFE_VELOCITY_LIMITER_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__APPARENT_SAFE_VELOCITY_LIMITER_HPP_

#include "apparent_safe_velocity_limiter/types.hpp"

#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <string>
#include <vector>

namespace apparent_safe_velocity_limiter
{

/// @brief calculate the apparent safe velocity
/// @param[in] trajectory_point trajectory point for which to calculate the apparent safe velocity
/// @param[in] dist_to_collision distance from the trajectory point to the apparent collision
/// @return apparent safe velocity
Float calculateSafeVelocity(
  const TrajectoryPoint & trajectory_point, const Float dist_to_collision);

/// @brief calculate trajectory index that is ahead of the given index by the given distance
/// @param[in] trajectory trajectory
/// @param[in] ego_idx index closest to the current ego position in the trajectory
/// @param[in] start_distance desired distance ahead of the ego_idx
/// @return trajectory index ahead of ego_idx by the start_distance
size_t calculateStartIndex(
  const Trajectory & trajectory, const size_t ego_idx, const Float start_distance);

/// @brief downsample a trajectory, reducing its number of points by the given factor
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index of the input trajectory
/// @param[in] factor factor used for downsampling
/// @return downsampled trajectory
Trajectory downsampleTrajectory(
  const Trajectory & trajectory, const size_t start_idx, const int factor);

/// @brief create negative polygon masks from the dynamic objects
/// @param[in] dynamic_obstacles the dynamic objects to mask
/// @param[in] buffer buffer used to enlarge the mask
/// @param[in] min_vel minimum velocity for an object to be masked
/// @return polygon masks around dynamic objects
multipolygon_t createPolygonMasks(
  const autoware_auto_perception_msgs::msg::PredictedObjects & dynamic_obstacles,
  const Float buffer, const Float min_vel);

/// @brief create footprint polygons from projection lines
/// @details A footprint is create for each group of lines. Each group of lines is assumed to share
/// some points such that their footprint is a single connected polygon.
/// @param[in] projections the projection lines
/// @param[in] lateral_offset offset to create polygons around the lines
/// @return polygon footprint of each projection lines
std::vector<polygon_t> createFootprintPolygons(
  const std::vector<multilinestring_t> & projected_linestrings, const Float lateral_offset);

/// @brief create a polygon of the safety envelope
/// @details the safety envelope is the area covered by forward projections at each trajectory
/// point
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index in the input trajectory
/// @param[in] projection_params parameters of the forward projection
/// @return the envelope polygon
polygon_t createEnvelopePolygon(
  const Trajectory & trajectory, const size_t start_idx, ProjectionParameters & projection_params);

/// @brief create a polygon of the safety envelope from the projection footprints
/// @details the safety envelope is the area covered by forward projections at each trajectory
/// point
/// @param[in] footprints projection footprints
/// @return the envelope polygon
polygon_t createEnvelopePolygon(const std::vector<polygon_t> & footprints);

/// @brief create projection lines for each trajectory point
/// @details depending on the method used, multiple lines can be created for a same trajectory point
/// @param[in] trajectory input trajectory
/// @param[in] params projection parameters
/// @return projecton lines for each trajectory point
std::vector<multilinestring_t> createProjectedLines(
  const Trajectory & trajectory, ProjectionParameters & params);

/// @brief create linestrings around obstacles
/// @param[in] occupancy_grid occupancy grid
/// @param[in] pointcloud pointcloud
/// @param[in] polygon_masks negative masks where obstacles will be ignored
/// @param[in] positive_mask positive masks where obstacles must reside
/// @param[in] transform_listener object used to retrieve the latest transform
/// @param[in] target_frame frame of the returned obstacles
/// @param[in] obstacle_params obstacle parameters
/// @param[out] debug_pointcloud resulting filtered pointcloud
/// @return linestrings representing obstacles to avoid
multilinestring_t createObstacleLines(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const sensor_msgs::msg::PointCloud2 & pointcloud, const multipolygon_t & polygon_masks,
  const polygon_t & envelope_polygon, tier4_autoware_utils::TransformListener & transform_listener,
  const std::string & target_frame, const ObstacleParameters & obstacle_params,
  PointCloud & debug_pointcloud);

/// @brief limit the velocity of the given trajectory
/// @param[in] trajectory input trajectory
/// @param[in] obstacles obstacles that must be avoided by the forward projection
/// @param[in] projections forward projection lines at each trajectory point
/// @param[in] footprints footprint of the forward projection at each trajectory point
/// @param[in] current_ego_velocity current ego velocity (at the first trajectory point)
/// @param[in] max_deceleration maximum allowed deceleration
/// @param[in] params projection parameters
/// @param[in] min_velocity minimum velocity that can be set by this function
void limitVelocity(
  Trajectory & trajectory, const multilinestring_t & obstacles,
  const std::vector<multilinestring_t> & projections, const std::vector<polygon_t> & footprints,
  const ProjectionParameters & projection_params, const VelocityParameters & velocity_params);

/// @brief copy the velocity profile of a downsampled trajectory to the original trajectory
/// @param[in] downsampled_trajectory downsampled trajectory
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index of the downsampled trajectory relative to the input
/// @param[in] factor downsampling factor
/// @return input trajectory with the velocity profile of the downsampled trajectory
Trajectory copyDownsampledVelocity(
  const Trajectory & downsampled_traj, Trajectory trajectory, const size_t start_idx,
  const int factor);
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__APPARENT_SAFE_VELOCITY_LIMITER_HPP_
