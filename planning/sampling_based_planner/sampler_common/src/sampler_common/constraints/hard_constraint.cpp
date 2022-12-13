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

#include "sampler_common/constraints/hard_constraint.hpp"

#include "sampler_common/constraints/footprint.hpp"
#include "sampler_common/structures.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/intersects/interface.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/core/cs.hpp>

#include <algorithm>
#include <csignal>
#include <iostream>

namespace sampler_common::constraints
{
bool satisfyMinMax(const std::vector<double> & values, const double min, const double max)
{
  for (const auto value : values) {
    if (value < min || value > max) return false;
  }
  return true;
}

// TODO(Maxime CLEMENT): remove if unused
bool collideWithPolygons(const Path & path, const std::vector<Polygon> & polygons)
{
  for (const auto & polygon : polygons) {
    for (const auto & point : path.points) {
      // TODO(Maxime CLEMENT): arbitrary distance threshold
      if (boost::geometry::distance(point, polygon) < 1.0) {
        return true;
      }
    }
  }
  return false;
}

bool collideWithPolygons(const Polygon & footprint, const MultiPolygon & polygons)
{
  for (const auto & footprint_point : footprint.outer()) {
    if (boost::geometry::within(footprint_point, polygons)) {
      return true;
    }
  }
  return false;
}

bool withinPolygons(const Path & path, const std::vector<Polygon> & polygons)
{
  for (const auto & point : path.points) {
    bool within_at_least_one_poly = false;
    for (const auto & polygon : polygons) {
      if (boost::geometry::within(point, polygon.outer())) {
        within_at_least_one_poly = true;
        break;
      }
    }
    if (!within_at_least_one_poly) return false;
  }
  return true;
}

bool withinPolygons(const Polygon & footprint, const Polygon & polygons)
{
  return boost::geometry::within(footprint, polygons);
}

bool collideOverTime(const Trajectory & traj, const Constraints & constraints)
{
  for (const auto & obs : constraints.dynamic_obstacles) {
    const auto t = traj.resampleTimeFromZero(obs.time_step);
    for (auto i = 0lu; i < std::min(t.points.size(), obs.footprint_per_time.size()); ++i) {
      const auto ego_footprint = buildFootprintPolygon(t.points[i], t.yaws[i], constraints);
      if (boost::geometry::intersects(ego_footprint, obs.footprint_per_time[i])) return true;
    }
  }
  return false;
}

bool belowCollisionDistance(
  const Polygon & footprint, const MultiPolygon & polygons, const double min_dist)
{
  for (const auto & polygon : polygons) {
    if (boost::geometry::distance(footprint, polygon) < min_dist) return true;
  }
  return false;
}

void checkHardConstraints(Path & path, const Constraints & constraints)
{
  const Polygon footprint = buildFootprintPolygon(path, constraints);
  if (!footprint.outer().empty()) {
    if (belowCollisionDistance(
          footprint, constraints.obstacle_polygons, constraints.collision_distance_buffer)) {
      path.constraint_results.collision = false;
    }
    if (collideWithPolygons(footprint, constraints.drivable_polygons)) {
      path.constraint_results.drivable_area = false;
    }
  }
  if (!satisfyMinMax(
        path.curvatures, constraints.hard.min_curvature, constraints.hard.max_curvature)) {
    path.constraint_results.curvature = false;
  }
}

void checkHardConstraints(Trajectory & traj, const Constraints & constraints)
{
  Path & path = traj;
  checkHardConstraints(path, constraints);
  if (collideOverTime(traj, constraints)) traj.constraint_results.collision = false;
  checkVelocityConstraints(traj, constraints);
}

void checkVelocityConstraints(Trajectory & traj, const Constraints & constraints)
{
  if (!satisfyMinMax(
        traj.longitudinal_velocities, constraints.hard.min_velocity,
        constraints.hard.max_velocity)) {
    traj.constraint_results.velocity = false;
  }
  if (
    !satisfyMinMax(
      traj.longitudinal_accelerations, constraints.hard.min_acceleration,
      constraints.hard.max_acceleration) ||
    !satisfyMinMax(
      traj.lateral_accelerations, constraints.hard.min_acceleration,
      constraints.hard.max_acceleration)) {
    traj.constraint_results.acceleration = false;
  }
  // yaw rate // TODO(Maxime): move to other function
  for (auto i = 0lu; i + 1 < traj.curvatures.size(); ++i) {
    const auto yaw_rate = (traj.yaws[i + 1] - traj.yaws[i]) / (traj.times[i + 1] - traj.times[i]);
    if (yaw_rate > constraints.hard.max_yaw_rate) {
      traj.constraint_results.yaw_rate = false;
      break;
    }
  }
  // not going too fast at the end of the path
  if (!traj.longitudinal_velocities.empty()) {
    const auto dist_to_stop = constraints.distance_to_end - traj.lengths.back();
    if (dist_to_stop > 0.0) {
      const auto acc_to_stop =
        (traj.longitudinal_velocities.back() * traj.longitudinal_velocities.back()) /
        (2 * dist_to_stop);
      std::cout << dist_to_stop << " = " << constraints.distance_to_end << " - "
                << traj.lengths.back() << std::endl;
      std::cout << "\t" << acc_to_stop << std::endl;
      if (-acc_to_stop < constraints.hard.min_acceleration)
        traj.constraint_results.velocity = false;
    }
  }
}
}  // namespace sampler_common::constraints
