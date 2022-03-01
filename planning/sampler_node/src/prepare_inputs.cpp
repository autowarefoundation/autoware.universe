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

#include "sampler_node/prepare_inputs.hpp"

#include "frenet_planner/structures.hpp"
#include "sampler_node/utils/occupancy_grid_to_polygons.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <algorithm>
#include <vector>

namespace sampler_node
{

sampler_common::Constraints prepareConstraints(
  const nav_msgs::msg::OccupancyGrid & drivable_area,
  const autoware_auto_perception_msgs::msg::PredictedObjects & predicted_objects)
{
  sampler_common::Constraints constraints;
  constraints.hard.max_curvature = 0.3;
  constraints.hard.min_curvature = -constraints.hard.max_curvature;
  constraints.hard.max_lateral_deviation = 6.0;
  constraints.hard.min_lateral_deviation = -5.0;
  constraints.hard.max_acceleration = 3.0;
  constraints.hard.min_acceleration = -6.0;
  constraints.hard.max_jerk = 10.0;   // TODO(Maxime CLEMENT): not used yet
  constraints.hard.min_jerk = -10.0;  // TODO(Maxime CLEMENT): not used yet
  constraints.hard.max_velocity = 25.0;
  constraints.hard.min_velocity = 0.0;  // no reverse allowed
  constraints.soft.velocity_deviation_weight = 1.0;
  constraints.soft.lateral_deviation_weight = 0.1;
  constraints.soft.longitudinal_deviation_weight = 1.0;
  constraints.soft.jerk_weight = 1.0;
  constraints.soft.curvature_weight = 10.0;

  if (false) {
    for (const auto & occ_grid_polygon : utils::occupancyGridToPolygons(drivable_area)) {
      constraints.obstacle_polygons.push_back(occ_grid_polygon);
    }
  }
  for (const auto & dynamic_obstacle_polygon :
       utils::predictedObjectsToPolygons(predicted_objects)) {
    constraints.obstacle_polygons.push_back(dynamic_obstacle_polygon);
  }
  return constraints;
}

frenet_planner::SamplingParameters prepareSamplingParameters(
  const frenet_planner::FrenetState & initial_state,
  const autoware_auto_planning_msgs::msg::Path & path, const double base_duration,
  const sampler_common::transform::Spline2D & path_spline)
{
  const auto max_s =
    path_spline.frenet({path.points.back().pose.position.x, path.points.back().pose.position.y}).s;
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.time_resolution = 0.1;
  for (auto d = 4; d <= 12; d += 2) {
    const auto duration = std::max(0.0, d - base_duration);
    sampling_parameters.target_durations.push_back(duration);
    if (duration == 0.0) {
      break;
    }
  }
  sampling_parameters.target_lateral_positions = {-2.0, -1.0, 0.0, 1.0, 2.0};
  sampling_parameters.target_longitudinal_accelerations = {0.0};
  sampling_parameters.target_lateral_velocities = {-0.5, 0.0, 0.5};
  sampling_parameters.target_lateral_accelerations = {0.0};
  // TODO(Maxime CLEMENT): get target vel + target longitudinal pos based on the Path
  const auto max_velocity =
    std::max_element(path.points.begin(), path.points.end(), [](const auto & p1, const auto & p2) {
      return p1.longitudinal_velocity_mps < p2.longitudinal_velocity_mps;
    })->longitudinal_velocity_mps;
  sampling_parameters.target_longitudinal_velocities = {max_velocity};
  for (const auto target_duration : sampling_parameters.target_durations) {
    for (const auto target_vel : sampling_parameters.target_longitudinal_velocities) {
      const auto target_s = initial_state.position.s + target_duration * target_vel;
      // Prevent target past the end
      if (target_s < max_s) sampling_parameters.target_longitudinal_positions.push_back(target_s);
    }
  }
  // Stopping
  if (sampling_parameters.target_longitudinal_positions.empty()) {
    sampling_parameters.target_longitudinal_positions = {max_s};
    sampling_parameters.target_longitudinal_velocities = {0.0};
    if (initial_state.longitudinal_velocity > 0) {
      const auto const_decel_duration = std::min(
        10.0,  // prevents very high values when driving at very low velocity
        2 * (max_s - initial_state.position.s) / initial_state.longitudinal_velocity);
      sampling_parameters.target_durations.clear();
      for (auto offset = -2.0; offset <= 2.0; offset += 0.5) {
        sampling_parameters.target_durations.push_back(const_decel_duration + offset);
      }
      sampling_parameters.target_longitudinal_accelerations = {-1.0, -0.5, 0.0};
    }
  }
  return sampling_parameters;
}

sampler_common::transform::Spline2D preparePathSpline(
  const autoware_auto_planning_msgs::msg::Path & path_msg)
{
  std::vector<double> x;
  std::vector<double> y;
  x.reserve(path_msg.points.size());
  y.reserve(path_msg.points.size());
  for (const auto & point : path_msg.points) {
    x.push_back(point.pose.position.x);
    y.push_back(point.pose.position.y);
  }
  return {x, y};
}

frenet_planner::Trajectory preparePreviousTrajectory(
  const frenet_planner::Trajectory & prev_trajectory,
  const sampler_common::transform::Spline2D & path_spline)
{
  frenet_planner::Trajectory trajectory = prev_trajectory;
  // Update frenet points for the new reference path
  trajectory.frenet_points.clear();
  for (const auto & p : trajectory.points) {
    trajectory.frenet_points.push_back(path_spline.frenet(p));
  }
  return trajectory;
}
}  // namespace sampler_node
