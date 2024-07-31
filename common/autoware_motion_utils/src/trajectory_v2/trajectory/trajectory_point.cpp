// Copyright 2024 Tier IV, Inc.
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

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_point.hpp"

#include "autoware/motion_utils/trajectory_container/detail/types.hpp"
#include "autoware/motion_utils/trajectory_container/interpolator/cubic_spline.hpp"

namespace autoware::motion_utils::trajectory_container::trajectory
{
TrajectoryV2<geometry_msgs::msg::Point>::TrajectoryV2()
{
  set_xy_interpolator(interpolator::CubicSpline());
  set_z_interpolator(interpolator::CubicSpline());
}

TrajectoryV2<geometry_msgs::msg::Point> & TrajectoryV2<geometry_msgs::msg::Point>::build(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<double> xs, ys, zs;

  axis_.resize(points.size());
  axis_(0) = 0.0;
  xs.emplace_back(points[0].x);
  ys.emplace_back(points[0].y);
  zs.emplace_back(points[0].z);

  for (size_t i = 1; i < points.size(); ++i) {
    Eigen::Vector2d p0(points[i - 1].x, points[i - 1].y), p1(points[i].x, points[i].y);
    axis_(i) = axis_(i - 1) + (p1 - p0).norm();
    xs.emplace_back(points[i].x);
    ys.emplace_back(points[i].y);
    zs.emplace_back(points[i].z);
  }

  start_ = axis_(0);
  end_ = axis_(axis_.size() - 1);

  x_interpolator_->build(axis_, xs);
  y_interpolator_->build(axis_, ys);
  z_interpolator_->build(axis_, zs);

  return *this;
}

TrajectoryV2<geometry_msgs::msg::Point> &
TrajectoryV2<geometry_msgs::msg::Point>::set_xy_interpolator(
  const interpolator::Interpolator<double> & interpolator)
{
  x_interpolator_ = std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
  y_interpolator_ = std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
  return *this;
}

TrajectoryV2<geometry_msgs::msg::Point> &
TrajectoryV2<geometry_msgs::msg::Point>::set_z_interpolator(
  const interpolator::Interpolator<double> & interpolator)
{
  z_interpolator_ = std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
  return *this;
}

double TrajectoryV2<geometry_msgs::msg::Point>::length() const
{
  return end_ - start_;
}

geometry_msgs::msg::Point TrajectoryV2<geometry_msgs::msg::Point>::compute(const double & s) const
{
  geometry_msgs::msg::Point result;
  result.x = x_interpolator_->compute(s + start_);
  result.y = y_interpolator_->compute(s + start_);
  result.z = z_interpolator_->compute(s + start_);
  return result;
}

double TrajectoryV2<geometry_msgs::msg::Point>::direction(const double & s) const
{
  double dx = x_interpolator_->compute_first_derivative(s + start_);
  double dy = y_interpolator_->compute_first_derivative(s + start_);
  return std::atan2(dy, dx);
}

double TrajectoryV2<geometry_msgs::msg::Point>::curvature(const double & s) const
{
  double dx = x_interpolator_->compute_first_derivative(s + start_);
  double ddx = x_interpolator_->compute_second_derivative(s + start_);
  double dy = y_interpolator_->compute_first_derivative(s + start_);
  double ddy = y_interpolator_->compute_second_derivative(s + start_);
  return std::abs(dx * ddy - dy * ddx) / std::pow(dx * dx + dy * dy, 1.5);
}

using motion_utils::trajectory_container::detail::to_point;

template <typename InputPointType>
std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::nearest_with_constraint(
  const InputPointType & p, const ConstraintFunction & constraints) const
{
  Eigen::Vector2d point(to_point(p).x, to_point(p).y);
  std::vector<double> distances_from_segments;
  std::vector<double> lengthes_from_start_points;
  for (int i = 1; i < axis_.size(); ++i) {
    Eigen::Vector2d p0, p1;
    p0 << x_interpolator_->compute(axis_(i - 1)), y_interpolator_->compute(axis_(i - 1));
    p1 << x_interpolator_->compute(axis_(i)), y_interpolator_->compute(axis_(i));
    Eigen::Vector2d v = p1 - p0, w = point - p0;
    double c1 = w.dot(v), c2 = v.dot(v);
    double length_from_start_point, distance_from_segment;
    if (c1 <= 0) {
      length_from_start_point = axis_(i - 1);
      distance_from_segment = (point - p0).norm();
    } else if (c2 <= c1) {
      length_from_start_point = axis_(i);
      distance_from_segment = (point - p1).norm();
    } else {
      length_from_start_point = axis_(i - 1) + c1 / c2 * (p1 - p0).norm();
      distance_from_segment = (point - (p0 + (c1 / c2) * v)).norm();
    }
    if (constraints(length_from_start_point)) {
      distances_from_segments.push_back(distance_from_segment);
      lengthes_from_start_points.push_back(length_from_start_point);
    }
  }
  if (distances_from_segments.empty()) {
    return std::nullopt;
  }
  auto min_it = std::min_element(distances_from_segments.begin(), distances_from_segments.end());

  return lengthes_from_start_points[std::distance(distances_from_segments.begin(), min_it)];
}

template <typename InputPointType>
double TrajectoryV2<geometry_msgs::msg::Point>::nearest(const InputPointType & p) const
{
  auto s = nearest_with_constraint(p, [](const double &) { return true; });
  return *s;
}

template <typename InputPointType>
std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed_with_constraint(
  const InputPointType & start, const InputPointType & end,
  const ConstraintFunction & constraints) const
{
  Eigen::Vector2d line_start(to_point(start).x, to_point(start).y);
  Eigen::Vector2d line_end(to_point(end).x, to_point(end).y);
  Eigen::Vector2d line_dir = line_end - line_start;

  for (int i = 1; i < axis_.size(); ++i) {
    Eigen::Vector2d p0, p1;
    p0 << x_interpolator_->compute(axis_(i - 1)), y_interpolator_->compute(axis_(i - 1));
    p1 << x_interpolator_->compute(axis_(i)), y_interpolator_->compute(axis_(i));

    Eigen::Vector2d segment_dir = p1 - p0;

    double det = segment_dir.x() * line_dir.y() - segment_dir.y() * line_dir.x();

    if (std::abs(det) < 1e-10) {
      continue;
    }

    Eigen::Vector2d p0_to_line_start = line_start - p0;

    double t = (p0_to_line_start.x() * line_dir.y() - p0_to_line_start.y() * line_dir.x()) / det;
    double u =
      (p0_to_line_start.x() * segment_dir.y() - p0_to_line_start.y() * segment_dir.x()) / det;

    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
      double intersection_s = axis_(i - 1) + t * (axis_(i) - axis_(i - 1));
      if (constraints(intersection_s)) {
        return intersection_s;
      }
    }
  }

  return std::nullopt;
}

template <typename InputPointType>
std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed(
  const InputPointType & start, const InputPointType & end) const
{
  return crossed_with_constraint(start, end, [](const double &) { return true; });
}

std::vector<geometry_msgs::msg::Point> TrajectoryV2<geometry_msgs::msg::Point>::restore() const
{
  std::vector<geometry_msgs::msg::Point> points(axis_.size());
  std::transform(axis_.begin(), axis_.end(), points.begin(), [this](const auto & s) {
    geometry_msgs::msg::Point p;
    p.x = x_interpolator_->compute(s);
    p.y = y_interpolator_->compute(s);
    p.z = z_interpolator_->compute(s);
    return p;
  });
  return points;
}

// Explicit instantiation for nearest_with_constraint
template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::nearest_with_constraint(
  const geometry_msgs::msg::Point & p, const ConstraintFunction & constraints) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::nearest_with_constraint(
  const geometry_msgs::msg::Pose & p, const ConstraintFunction & constraints) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::nearest_with_constraint(
  const Eigen::Ref<const Eigen::Vector2d> & p, const ConstraintFunction & constraints) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::nearest_with_constraint(
  const autoware_planning_msgs::msg::PathPoint & p, const ConstraintFunction & constraints) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::nearest_with_constraint(
  const tier4_planning_msgs::msg::PathPointWithLaneId & p,
  const ConstraintFunction & constraints) const;

// Explicit instantiation for nearest
template double TrajectoryV2<geometry_msgs::msg::Point>::nearest(
  const geometry_msgs::msg::Point & p) const;

template double TrajectoryV2<geometry_msgs::msg::Point>::nearest(
  const geometry_msgs::msg::Pose & p) const;

template double TrajectoryV2<geometry_msgs::msg::Point>::nearest(
  const Eigen::Ref<const Eigen::Vector2d> & p) const;

template double TrajectoryV2<geometry_msgs::msg::Point>::nearest(
  const autoware_planning_msgs::msg::PathPoint & p) const;

template double TrajectoryV2<geometry_msgs::msg::Point>::nearest(
  const tier4_planning_msgs::msg::PathPointWithLaneId & p) const;

// Explicit instantiation for crossed_with_constraint
template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed_with_constraint(
  const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & end,
  const ConstraintFunction & constraints) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed_with_constraint(
  const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & end,
  const ConstraintFunction & constraints) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed_with_constraint(
  const Eigen::Ref<const Eigen::Vector2d> & start, const Eigen::Ref<const Eigen::Vector2d> & end,
  const ConstraintFunction & constraints) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed_with_constraint(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end, const ConstraintFunction & constraints) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed_with_constraint(
  const tier4_planning_msgs::msg::PathPointWithLaneId & start,
  const tier4_planning_msgs::msg::PathPointWithLaneId & end,
  const ConstraintFunction & constraints) const;

// Explicit instantiation for crossed
template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed(
  const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & end) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed(
  const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & end) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed(
  const Eigen::Ref<const Eigen::Vector2d> & start,
  const Eigen::Ref<const Eigen::Vector2d> & end) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end) const;

template std::optional<double> TrajectoryV2<geometry_msgs::msg::Point>::crossed(
  const tier4_planning_msgs::msg::PathPointWithLaneId & start,
  const tier4_planning_msgs::msg::PathPointWithLaneId & end) const;

}  // namespace autoware::motion_utils::trajectory_container::trajectory
