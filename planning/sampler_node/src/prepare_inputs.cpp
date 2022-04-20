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
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/path_generation.hpp"
#include "sampler_node/utils/occupancy_grid_to_polygons.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <algorithm>
#include <vector>

namespace sampler_node
{

sampler_common::Constraints prepareConstraints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & predicted_objects,
  const lanelet::LaneletMap & map, const lanelet::Ids & drivable_ids, const lanelet::Ids & prefered_ids)
{
  sampler_common::Constraints constraints;
  constraints.hard.max_curvature = 0.3;
  constraints.hard.min_curvature = -constraints.hard.max_curvature;
  constraints.soft.lateral_deviation_weight = 0.1;
  constraints.soft.longitudinal_deviation_weight = 1.0;
  constraints.soft.jerk_weight = 1.0;
  constraints.soft.length_weight = -1.0;  // negative weight to consider length as a reward
  constraints.soft.curvature_weight = 1.0;

  for (auto & dynamic_obstacle_polygon :
       utils::predictedObjectsToPolygons(predicted_objects)) {
    boost::geometry::correct(dynamic_obstacle_polygon);
    constraints.obstacle_polygons.push_back(dynamic_obstacle_polygon);
  }
  for(const auto & drivable_id : drivable_ids) {
    const auto lanelet = map.laneletLayer.get(drivable_id);
    sampler_common::Polygon road_polygon;
    for(const auto & point: lanelet.polygon2d().basicPolygon()) {
      road_polygon.outer().push_back({point.x(), point.y()});
    }
    boost::geometry::correct(road_polygon);
    constraints.drivable_polygons.push_back(std::move(road_polygon));
  }
  for(const auto & prefered_id : prefered_ids) {
    const auto lanelet = map.laneletLayer.get(prefered_id);
    sampler_common::Polygon prefered_polygon;
    for(const auto & point: lanelet.polygon2d().basicPolygon()) {
      prefered_polygon.outer().push_back({point.x(), point.y()});
    }
    boost::geometry::correct(prefered_polygon);
    constraints.prefered_polygons.push_back(std::move(prefered_polygon));
  }
  return constraints;
}

frenet_planner::SamplingParameters prepareSamplingParameters(
  const sampler_common::State & initial_state, const autoware_auto_planning_msgs::msg::Path & path,
  const double base_length, const sampler_common::transform::Spline2D & path_spline, const Parameters & params)
{
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.time_resolution = params.sampling.resolution;
  sampling_parameters.target_lateral_positions = params.sampling.frenet.target_lateral_positions;
  sampling_parameters.target_lateral_velocities = params.sampling.frenet.target_lateral_velocities;
  sampling_parameters.target_lateral_accelerations = params.sampling.frenet.target_lateral_accelerations;
  const auto max_s =
    path_spline.frenet({path.points.back().pose.position.x, path.points.back().pose.position.y}).s;
  for (const auto target_length : params.sampling.target_lengths) {
    const auto target_s =
      path_spline.frenet(initial_state.pose).s + std::max(0.0, target_length - base_length);
    // Prevent a target past the end of the reference path
    if (target_s < max_s)
      sampling_parameters.target_longitudinal_positions.push_back(target_s);
    else
      break;
  }
  // Stopping case
  if (sampling_parameters.target_longitudinal_positions.empty()) {
    sampling_parameters.target_longitudinal_positions = {max_s};
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
