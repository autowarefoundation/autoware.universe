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

#include "autoware/trajectory/pose.hpp"

#include "autoware/trajectory/detail/helpers.hpp"
#include "autoware/trajectory/interpolator/spherical_linear.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <vector>

namespace autoware::trajectory
{
using PointType = geometry_msgs::msg::Pose;

Trajectory<PointType>::Trajectory()
: orientation_interpolator_(std::make_shared<interpolator::SphericalLinear>())
{
}

Trajectory<PointType>::Trajectory(const Trajectory & rhs)
: BaseClass(rhs), orientation_interpolator_(rhs.orientation_interpolator_->clone())
{
}

Trajectory<PointType> & Trajectory<PointType>::operator=(const Trajectory & rhs)
{
  if (this != &rhs) {
    BaseClass::operator=(rhs);
    orientation_interpolator_ = rhs.orientation_interpolator_->clone();
  }
  return *this;
}

bool Trajectory<PointType>::build(const std::vector<PointType> & points)
{
  std::vector<geometry_msgs::msg::Point> path_points;
  std::vector<geometry_msgs::msg::Quaternion> orientations;
  path_points.reserve(points.size());
  orientations.reserve(points.size());
  for (const auto & point : points) {
    path_points.emplace_back(point.position);
    orientations.emplace_back(point.orientation);
  }

  bool is_valid = true;
  is_valid &= BaseClass::build(path_points);
  is_valid &= orientation_interpolator_->build(bases_, orientations);
  return is_valid;
}

std::vector<double> Trajectory<PointType>::get_internal_bases() const
{
  auto bases = detail::crop_bases(bases_, start_, end_);
  std::transform(
    bases.begin(), bases.end(), bases.begin(), [this](const double & s) { return s - start_; });
  return bases;
}

PointType Trajectory<PointType>::compute(double s) const
{
  PointType result;
  result.position = BaseClass::compute(s);
  s = clamp(s);
  result.orientation = orientation_interpolator_->compute(s);
  return result;
}

void Trajectory<PointType>::align_orientation_with_trajectory_direction()
{
  std::vector<geometry_msgs::msg::Quaternion> aligned_orientations;
  for (const auto & s : bases_) {
    const double azimuth = this->azimuth(s);
    const double elevation = this->elevation(s);
    const geometry_msgs::msg::Quaternion current_orientation =
      orientation_interpolator_->compute(s);
    tf2::Quaternion current_orientation_tf2(
      current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w);
    current_orientation_tf2.normalize();
    const tf2::Vector3 x_axis(1.0, 0.0, 0.0);
    const tf2::Vector3 current_x_axis = tf2::quatRotate(current_orientation_tf2, x_axis);

    const tf2::Vector3 desired_x_axis(
      std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth),
      std::sin(elevation));
    const tf2::Vector3 rotation_axis = current_x_axis.cross(desired_x_axis).normalized();
    const double dot_product = current_x_axis.dot(desired_x_axis);
    const double rotation_angle = std::acos(dot_product);

    const tf2::Quaternion delta_q(rotation_axis, rotation_angle);
    const tf2::Quaternion aligned_orientation_tf2 =
      (delta_q * current_orientation_tf2).normalized();

    geometry_msgs::msg::Quaternion aligned_orientation;
    aligned_orientation.x = aligned_orientation_tf2.x();
    aligned_orientation.y = aligned_orientation_tf2.y();
    aligned_orientation.z = aligned_orientation_tf2.z();
    aligned_orientation.w = aligned_orientation_tf2.w();

    aligned_orientations.emplace_back(aligned_orientation);
  }
  const bool success = orientation_interpolator_->build(bases_, aligned_orientations);
  if (!success) {
    throw std::runtime_error(
      "Failed to build orientation interpolator.");  // This Exception should not be thrown.
  }
}

std::vector<PointType> Trajectory<PointType>::restore(const size_t & min_points) const
{
  auto bases = get_internal_bases();
  bases = detail::fill_bases(bases, min_points);
  std::vector<PointType> points;
  points.reserve(bases.size());
  for (const auto & s : bases) {
    points.emplace_back(compute(s));
  }
  return points;
}

}  // namespace autoware::trajectory
