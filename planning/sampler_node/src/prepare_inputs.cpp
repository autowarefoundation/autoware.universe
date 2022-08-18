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
#include "lanelet2_core/LaneletMap.h"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/path_generation.hpp"
#include "sampler_node/utils/occupancy_grid_to_polygons.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/detail/intersects/interface.hpp>
#include <boost/geometry/geometry.hpp>
#include <eigen3/unsupported/Eigen/Splines>

#include <algorithm>
#include <list>
#include <memory>
#include <vector>

namespace sampler_node
{

void prepareConstraints(
  sampler_common::Constraints & constraints,
  const autoware_auto_perception_msgs::msg::PredictedObjects & predicted_objects,
  [[maybe_unused]] const lanelet::LaneletMap & map,
  [[maybe_unused]] const lanelet::Ids & drivable_ids,
  [[maybe_unused]] const lanelet::Ids & prefered_ids,
  const nav_msgs::msg::OccupancyGrid & drivable_area)
{
  constraints.obstacle_polygons = sampler_common::MultiPolygon();
  for (auto & dynamic_obstacle_polygon : utils::predictedObjectsToPolygons(predicted_objects)) {
    boost::geometry::correct(dynamic_obstacle_polygon);
    constraints.obstacle_polygons.push_back(dynamic_obstacle_polygon);
  }

  constraints.drivable_polygons = utils::occupancyGridToPolygons(drivable_area);
  constraints.prefered_polygons = constraints.drivable_polygons;
}

frenet_planner::SamplingParameters prepareSamplingParameters(
  const sampler_common::State & initial_state, const autoware_auto_planning_msgs::msg::Path & path,
  const double base_length, const sampler_common::transform::Spline2D & path_spline,
  const Parameters & params)
{
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.time_resolution = params.sampling.resolution;
  sampling_parameters.target_lateral_positions = params.sampling.frenet.target_lateral_positions;
  sampling_parameters.target_lateral_velocities = params.sampling.frenet.target_lateral_velocities;
  sampling_parameters.target_lateral_accelerations =
    params.sampling.frenet.target_lateral_accelerations;
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
  const autoware_auto_planning_msgs::msg::Path & path_msg, const bool smooth_path)
{
  std::vector<double> x;
  std::vector<double> y;
  if (smooth_path) {
    constexpr auto spline_dim = 3;
    Eigen::MatrixXd control_points(path_msg.points.size(), 2);
    for (auto i = 0lu; i < path_msg.points.size(); ++i) {
      const auto & point = path_msg.points[i];
      control_points(i, 0) = point.pose.position.x;
      control_points(i, 1) = point.pose.position.y;
    }
    control_points.transposeInPlace();
    const auto nb_knots = path_msg.points.size() + spline_dim + 3;
    Eigen::RowVectorXd knots(nb_knots);
    constexpr auto repeat_endknots = 3lu;
    const auto knot_step = 1.0 / static_cast<double>(nb_knots - 2 * repeat_endknots);
    auto i = 0lu;
    for (; i < repeat_endknots; ++i) knots[i] = 0.0;
    for (; i < nb_knots - repeat_endknots; ++i) knots[i] = knots[i - 1] + knot_step;
    for (; i < nb_knots; ++i) knots[i] = 1.0;
    const auto spline = Eigen::Spline<double, 2, spline_dim>(knots, control_points);
    x.reserve(path_msg.points.size());
    y.reserve(path_msg.points.size());
    const auto t_step = 1 / static_cast<double>(path_msg.points.size());
    for (auto t = 0.0; t < 1.0; t += t_step) {
      const auto p = spline(t);
      x.push_back(p.x());
      y.push_back(p.y());
    }
  } else {
    x.reserve(path_msg.points.size());
    y.reserve(path_msg.points.size());
    for (const auto & point : path_msg.points) {
      x.push_back(point.pose.position.x);
      y.push_back(point.pose.position.y);
    }
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
