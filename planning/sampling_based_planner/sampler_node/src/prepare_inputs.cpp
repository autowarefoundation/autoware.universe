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
#include "sampler_node/calculate_sampling_parameters.hpp"
#include "sampler_node/parameters.hpp"
#include "sampler_node/utils/occupancy_grid_to_polygons.hpp"

#include <Eigen/Core>
#include <autoware_control_toolbox/splines/bsplines_smoother.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/detail/intersects/interface.hpp>
#include <boost/geometry/geometry.hpp>

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

// TODO(Maxime CLEMENT):
// - Implement some strategies so generate the target s and target velocity
//  - determine if we should decel/accel/keep.
//  - use some min_decel profile from the min_velocity points to determine the "latest time to
//  slowdown" points.
//    1 - find all pairs of min (t, v)
//    2 - for each pair calculate the fn: t' -> v' where (v'-v)/(t-t') = max_decel * t
//    3 - keep the (t, v)
// - Implement a fn: target_s, current_vel, current_accel --> target_duration
frenet_planner::SamplingParameters prepareSamplingParameters(
  const sampler_common::Configuration & initial_configuration,
  const autoware_auto_planning_msgs::msg::Path & path, const double base_length,
  const double base_duration, const sampler_common::transform::Spline2D & path_spline,
  const Parameters & params)
{
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.resolution = params.sampling.resolution;
  calculateTargets(
    sampling_parameters, initial_configuration, path, path_spline, params, base_length,
    base_duration);
  return sampling_parameters;
}

sampler_common::transform::Spline2D preparePathSpline(
  const autoware_auto_planning_msgs::msg::Path & path_msg, const Parameters & params)
{
  std::vector<double> x;
  std::vector<double> y;
  x.reserve(path_msg.points.size());
  y.reserve(path_msg.points.size());
  if (params.preprocessing.smooth_reference) {
    const auto smoother = ns_splines::BSplineSmoother(
      path_msg.points.size(), params.preprocessing.control_points_ratio,
      params.preprocessing.smooth_weight);
    Eigen::MatrixXd raw_points(path_msg.points.size(), 2);
    for (auto i = 0lu; i < path_msg.points.size(); ++i) {
      const auto & p = path_msg.points[i].pose.position;
      raw_points.row(static_cast<Eigen::Index>(i)) = Eigen::Vector2d(p.x, p.y);
    }
    Eigen::MatrixXd smooth_points(path_msg.points.size(), 2);
    smoother.InterpolateInCoordinates(raw_points, smooth_points);
    for (auto i = 0; i < smooth_points.rows(); ++i) {
      x.push_back(smooth_points.row(i).x());
      y.push_back(smooth_points.row(i).y());
    }
  } else {
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
