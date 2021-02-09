/*
 * Copyright 2021 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FRENET_PLANNER__STRUCTURES_HPP
#define FRENET_PLANNER__STRUCTURES_HPP

#include "frenet_planner/polynomials.hpp"

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <optional>
#include <vector>

namespace frenet_planner
{
using Point = boost::geometry::model::d2::point_xy<double>;
using Polygon = boost::geometry::model::polygon<Point>;

struct FrenetPoint
{
  FrenetPoint(double s_, double d_) : s(s_), d(d_) {}
  double s;
  double d;
};

struct Trajectory
{
  std::vector<Point> points{};
  std::vector<FrenetPoint> frenet_points{};
  std::vector<double> curvatures{};
  std::vector<double> yaws{};
  std::vector<double> longitudinal_velocities{};
  std::vector<double> longitudinal_accelerations{};
  std::vector<double> lateral_velocities{};
  std::vector<double> jerks{};
  std::vector<double> intervals{};
  std::vector<double> times{};
  std::optional<Polynomial> lateral_polynomial{};
  std::optional<Polynomial> longitudinal_polynomial{};
  double duration{};
  bool valid{};
  double cost{};

  Trajectory() { clear(); }

  void clear()
  {
    points.clear();
    frenet_points.clear();
    curvatures.clear();
    yaws.clear();
    longitudinal_velocities.clear();
    longitudinal_accelerations.clear();
    lateral_velocities.clear();
    jerks.clear();
    intervals.clear();
    times.clear();
    lateral_polynomial.reset();
    longitudinal_polynomial.reset();
    valid = true;
    cost = 0.0;
    duration = 0.0;
  }

  void reserve(const size_t size)
  {
    points.reserve(size);
    frenet_points.reserve(size);
    curvatures.reserve(size);
    yaws.reserve(size);
    longitudinal_velocities.reserve(size);
    longitudinal_accelerations.reserve(size);
    lateral_velocities.reserve(size);
    jerks.reserve(size);
    intervals.reserve(size);
    times.reserve(size);
  }

  [[nodiscard]] Trajectory extend(const Trajectory & traj) const
  {
    Trajectory extended_traj;
    extended_traj.points.insert(extended_traj.points.end(), points.begin(), points.end());
    extended_traj.points.insert(extended_traj.points.end(), traj.points.begin(), traj.points.end());
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), frenet_points.begin(), frenet_points.end());
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), traj.frenet_points.begin(), traj.frenet_points.end());
    extended_traj.curvatures.insert(
      extended_traj.curvatures.end(), curvatures.begin(), curvatures.end());
    extended_traj.curvatures.insert(
      extended_traj.curvatures.end(), traj.curvatures.begin(), traj.curvatures.end());
    extended_traj.yaws.insert(extended_traj.yaws.end(), yaws.begin(), yaws.end());
    extended_traj.yaws.insert(extended_traj.yaws.end(), traj.yaws.begin(), traj.yaws.end());
    extended_traj.longitudinal_velocities.insert(
      extended_traj.longitudinal_velocities.end(), longitudinal_velocities.begin(),
      longitudinal_velocities.end());
    extended_traj.longitudinal_velocities.insert(
      extended_traj.longitudinal_velocities.end(), traj.longitudinal_velocities.begin(),
      traj.longitudinal_velocities.end());
    extended_traj.longitudinal_accelerations.insert(
      extended_traj.longitudinal_accelerations.end(), longitudinal_accelerations.begin(),
      longitudinal_accelerations.end());
    extended_traj.longitudinal_accelerations.insert(
      extended_traj.longitudinal_accelerations.end(), traj.longitudinal_accelerations.begin(),
      traj.longitudinal_accelerations.end());
    extended_traj.lateral_velocities.insert(
      extended_traj.lateral_velocities.end(), lateral_velocities.begin(), lateral_velocities.end());
    extended_traj.lateral_velocities.insert(
      extended_traj.lateral_velocities.end(), traj.lateral_velocities.begin(),
      traj.lateral_velocities.end());
    extended_traj.jerks.insert(extended_traj.jerks.end(), jerks.begin(), jerks.end());
    extended_traj.jerks.insert(extended_traj.jerks.end(), traj.jerks.begin(), traj.jerks.end());
    extended_traj.intervals.insert(
      extended_traj.intervals.end(), intervals.begin(), intervals.end());
    extended_traj.intervals.insert(
      extended_traj.intervals.end(), traj.intervals.begin(), traj.intervals.end());
    extended_traj.times.insert(extended_traj.times.end(), times.begin(), times.end());
    extended_traj.times.insert(extended_traj.times.end(), traj.times.begin(), traj.times.end());
    extended_traj.duration = duration + traj.duration;
    // TODO(Maxime CLEMENT): direct copy from the 2nd trajectory. might need to be improved
    extended_traj.cost = traj.cost;
    extended_traj.lateral_polynomial = traj.lateral_polynomial;
    extended_traj.longitudinal_polynomial = traj.longitudinal_polynomial;
    extended_traj.valid = traj.valid;
    return extended_traj;
  }
};

struct FrenetState
{
  FrenetPoint position = {0, 0};
  double lateral_velocity{};
  double longitudinal_velocity{};
  double lateral_acceleration{};
  double longitudinal_acceleration{};
};

struct SamplingParameters
{
  std::vector<double> target_durations;
  std::vector<double> target_lateral_positions;
  std::vector<double> target_longitudinal_positions;
  std::vector<double> target_lateral_velocities;
  std::vector<double> target_longitudinal_velocities;
  std::vector<double> target_lateral_accelerations;
  std::vector<double> target_longitudinal_accelerations;
  double time_resolution;
};

struct Constraints
{
  struct
  {
    double lateral_deviation_weight;
    double longitudinal_deviation_weight;
    double velocity_deviation_weight;
    double jerk_weight;
    double curvature_weight;
  } soft;
  struct
  {
    double min_lateral_deviation;
    double max_lateral_deviation;
    double min_velocity;
    double max_velocity;
    double min_acceleration;
    double max_acceleration;
    double min_jerk;
    double max_jerk;
    double min_curvature;
    double max_curvature;
  } hard;
  struct
  {
    // TODO(Maxime CLEMENT): more complicated obstacle representations
    std::vector<Point> positions;
    double radius;
  } obstacles;
  std::vector<Polygon> obstacle_polygons;
};

struct Debug
{
  struct
  {
    size_t collision{};
    size_t curvature{};
    size_t velocity{};
    size_t lateral_deviation{};
  } nb_constraint_violations;
};
}  // namespace frenet_planner

#endif  // FRENET_PLANNER__STRUCTURES_HPP
