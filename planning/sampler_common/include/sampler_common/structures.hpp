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

#ifndef SAMPLER_COMMON__STRUCTURES_HPP
#define SAMPLER_COMMON__STRUCTURES_HPP

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <memory>
#include <vector>

namespace sampler_common
{
using Point = boost::geometry::model::d2::point_xy<double>;
using Polygon = boost::geometry::model::polygon<Point>;

struct FrenetPoint
{
  FrenetPoint(double s_, double d_) : s(s_), d(d_) {}
  double s;
  double d;
};

struct State
{
  Point pose{};
  double curvature{};
  double heading{};
};

/// @brief Path
struct Path
{
  std::vector<Point> points{};
  std::vector<double> curvatures{};
  std::vector<double> yaws{};
  std::vector<double> jerks{};
  std::vector<double> intervals{};  // TODO(Maxime CLEMENT): unnecessary ?
  bool valid = true;
  double cost{};

  Path() = default;

  virtual void clear()
  {
    points.clear();
    curvatures.clear();
    yaws.clear();
    jerks.clear();
    intervals.clear();
    valid = true;
    cost = 0.0;
  }

  virtual void reserve(const size_t size)
  {
    points.reserve(size);
    curvatures.reserve(size);
    yaws.reserve(size);
    jerks.reserve(size);
    intervals.reserve(size);
  }

  [[nodiscard]] Path extend(const Path & path) const
  {
    Path extended_path;
    extended_path.points.insert(extended_path.points.end(), points.begin(), points.end());
    extended_path.points.insert(extended_path.points.end(), path.points.begin(), path.points.end());
    extended_path.curvatures.insert(
      extended_path.curvatures.end(), curvatures.begin(), curvatures.end());
    extended_path.curvatures.insert(
      extended_path.curvatures.end(), path.curvatures.begin(), path.curvatures.end());
    extended_path.yaws.insert(extended_path.yaws.end(), yaws.begin(), yaws.end());
    extended_path.yaws.insert(extended_path.yaws.end(), path.yaws.begin(), path.yaws.end());
    extended_path.jerks.insert(extended_path.jerks.end(), jerks.begin(), jerks.end());
    extended_path.jerks.insert(extended_path.jerks.end(), path.jerks.begin(), path.jerks.end());
    extended_path.intervals.insert(
      extended_path.intervals.end(), intervals.begin(), intervals.end());
    extended_path.intervals.insert(
      extended_path.intervals.end(), path.intervals.begin(), path.intervals.end());
    // TODO(Maxime CLEMENT): direct copy from the 2nd pathectory. might need to be improved
    extended_path.cost = path.cost;
    extended_path.valid = path.valid;
    return extended_path;
  }

  // Return a pointer to allow overriding classes to return the appropriate type
  // Without pointer we are stuck with returning a Path
  [[nodiscard]] virtual Path * subset(const size_t from_idx, const size_t to_idx) const
  {
    auto * subpath = new Path();
    assert(to_idx >= from_idx);
    subpath->reserve(to_idx - from_idx);

    const auto copy_subset = [&](const auto & from, auto & to) {
      to.insert(to.end(), std::next(from.begin(), from_idx), std::next(from.begin(), to_idx));
    };
    copy_subset(points, subpath->points);
    copy_subset(curvatures, subpath->curvatures);
    copy_subset(intervals, subpath->intervals);
    copy_subset(yaws, subpath->yaws);
    copy_subset(intervals, subpath->intervals);
    // TODO(Maxime CLEMENT): jerk not yet computed
    return subpath;
  };
};

struct Trajectory : Path
{
  std::vector<double> longitudinal_velocities{};
  std::vector<double> longitudinal_accelerations{};
  std::vector<double> lateral_velocities{};
  std::vector<double> lateral_accelerations{};
  std::vector<double> times{};
  double duration = 0.0;

  Trajectory() = default;
  explicit Trajectory(const Path & path) : Path(path) {}

  void clear() override
  {
    Path::clear();
    longitudinal_velocities.clear();
    longitudinal_accelerations.clear();
    lateral_velocities.clear();
    lateral_accelerations.clear();
    times.clear();
    duration = 0.0;
  }

  void reserve(const size_t size) override
  {
    Path::reserve(size);
    longitudinal_velocities.reserve(size);
    longitudinal_accelerations.reserve(size);
    lateral_velocities.reserve(size);
    lateral_accelerations.reserve(size);
    times.reserve(size);
  }

  [[nodiscard]] Trajectory extend(const Trajectory & traj) const
  {
    Trajectory extended_traj(Path::extend(traj));
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
    extended_traj.lateral_accelerations.insert(
      extended_traj.lateral_accelerations.end(), lateral_accelerations.begin(),
      lateral_accelerations.end());
    extended_traj.lateral_accelerations.insert(
      extended_traj.lateral_accelerations.end(), traj.lateral_accelerations.begin(),
      traj.lateral_accelerations.end());
    extended_traj.times.insert(extended_traj.times.end(), times.begin(), times.end());
    extended_traj.times.insert(extended_traj.times.end(), traj.times.begin(), traj.times.end());
    extended_traj.duration = duration + traj.duration;
    return extended_traj;
  }

  [[nodiscard]] Trajectory * subset(const size_t from_idx, const size_t to_idx) const override
  {
    auto * subtraj = new Trajectory(*Path::subset(from_idx, to_idx));

    const auto copy_subset = [&](const auto & from, auto & to) {
      to.insert(to.end(), std::next(from.begin(), from_idx), std::next(from.begin(), to_idx));
    };

    copy_subset(lateral_velocities, subtraj->lateral_velocities);
    copy_subset(longitudinal_velocities, subtraj->longitudinal_velocities);
    copy_subset(longitudinal_accelerations, subtraj->longitudinal_accelerations);
    // copy(jerks, subtraj->jerks);
    copy_subset(times, subtraj->times);
    subtraj->duration = 0;
    for (const auto t : subtraj->times) {
      subtraj->duration += t;
    }
    return subtraj;
  }
};

struct Constraints
{
  struct
  {
    double lateral_deviation_weight;
    double longitudinal_deviation_weight;
    double length_weight;
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
}  // namespace sampler_common

#endif  // SAMPLER_COMMON__STRUCTURES_HPP
