// Copyright 2024 TIER IV, Inc.
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

#include "autoware/motion_utils/trajectory_container/trajectory/detail/utils.hpp"

#include <rclcpp/logging.hpp>

#include <Eigen/src/Core/util/Meta.h>
#include <fmt/format.h>

#include <cmath>
#include <cstddef>

namespace autoware::motion_utils::trajectory_container::trajectory
{

using PointType = geometry_msgs::msg::Point;

TrajectoryContainer<PointType>::TrajectoryContainer(
  const std::shared_ptr<interpolator::Interpolator<double>> & x_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & y_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & z_interpolator)
: x_interpolator_(x_interpolator), y_interpolator_(y_interpolator), z_interpolator_(z_interpolator)
{
}

bool TrajectoryContainer<PointType>::build(const std::vector<PointType> & points)
{
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;

  axis_.resize(static_cast<Eigen::Index>(points.size()));
  axis_(0) = 0.0;
  xs.emplace_back(points[0].x);
  ys.emplace_back(points[0].y);
  zs.emplace_back(points[0].z);

  for (size_t i = 1; i < points.size(); ++i) {
    Eigen::Vector2d p0(points[i - 1].x, points[i - 1].y);
    Eigen::Vector2d p1(points[i].x, points[i].y);
    axis_(static_cast<Eigen::Index>(i)) =
      axis_(static_cast<Eigen::Index>(i - 1)) + (p1 - p0).norm();
    xs.emplace_back(points[i].x);
    ys.emplace_back(points[i].y);
    zs.emplace_back(points[i].z);
  }

  start_ = axis_(0);
  end_ = axis_(axis_.size() - 1);

  bool is_valid = true;
  is_valid &= x_interpolator_->build(axis_, xs);
  is_valid &= y_interpolator_->build(axis_, ys);
  is_valid &= z_interpolator_->build(axis_, zs);

  return is_valid;
}

double TrajectoryContainer<PointType>::clamp(const double & s, bool show_warning) const
{
  if ((s < 0 || s > length()) && show_warning) {
    RCLCPP_WARN(
      rclcpp::get_logger("TrajectoryContainer"),
      "The arc length %f is out of the trajectory length %f", s, length());
  }
  return std::clamp(s, 0.0, length()) + start_;
}

double TrajectoryContainer<PointType>::length() const
{
  return end_ - start_;
}

PointType TrajectoryContainer<PointType>::compute(double s) const
{
  s = clamp(s, true);
  PointType result;
  result.x = x_interpolator_->compute(s);
  result.y = y_interpolator_->compute(s);
  result.z = z_interpolator_->compute(s);
  return result;
}

double TrajectoryContainer<PointType>::direction(double s) const
{
  s = clamp(s, true);
  double dx = x_interpolator_->compute_first_derivative(s);
  double dy = y_interpolator_->compute_first_derivative(s);
  return std::atan2(dy, dx);
}

double TrajectoryContainer<PointType>::curvature(double s) const
{
  s = clamp(s, true);
  double dx = x_interpolator_->compute_first_derivative(s);
  double ddx = x_interpolator_->compute_second_derivative(s);
  double dy = y_interpolator_->compute_first_derivative(s);
  double ddy = y_interpolator_->compute_second_derivative(s);
  return std::abs(dx * ddy - dy * ddx) / std::pow(dx * dx + dy * dy, 1.5);
}

std::vector<PointType> TrajectoryContainer<PointType>::restore(const size_t & min_points) const
{
  auto axis = detail::crop_axis(axis_, start_, end_);
  axis = detail::fill_axis(axis, static_cast<Eigen::Index>(min_points));
  std::vector<PointType> points;
  points.reserve(axis.size());
  for (const auto & s : axis) {
    PointType p;
    p.x = x_interpolator_->compute(s);
    p.y = y_interpolator_->compute(s);
    p.z = z_interpolator_->compute(s);
    points.emplace_back(p);
  }
  return points;
}

}  // namespace autoware::motion_utils::trajectory_container::trajectory
