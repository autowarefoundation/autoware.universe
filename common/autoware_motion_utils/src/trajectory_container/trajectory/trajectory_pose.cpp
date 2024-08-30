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

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_pose.hpp"

namespace autoware::motion_utils::trajectory_container::trajectory
{

using PointType = geometry_msgs::msg::Pose;

TrajectoryContainer<PointType>::TrajectoryContainer(
  const std::shared_ptr<interpolator::Interpolator<double>> & x_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & y_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & z_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & orientation_x_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & orientation_y_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & orientation_z_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & orientation_w_interpolator)
: BaseClass(x_interpolator, y_interpolator, z_interpolator),
  orientation_x_interpolator_(orientation_x_interpolator),
  orientation_y_interpolator_(orientation_y_interpolator),
  orientation_z_interpolator_(orientation_z_interpolator),
  orientation_w_interpolator_(orientation_w_interpolator)
{
}

bool TrajectoryContainer<PointType>::build(const std::vector<PointType> & points)
{
  std::vector<geometry_msgs::msg::Point> path_points;
  path_points.reserve(points.size());
  for (const auto & point : points) {
    path_points.emplace_back(point.position);
  }

  bool is_valid = true;
  is_valid &= BaseClass::build(path_points);

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  std::vector<double> ws;

  for (const auto & point : points) {
    xs.emplace_back(point.orientation.x);
    ys.emplace_back(point.orientation.y);
    zs.emplace_back(point.orientation.z);
    ws.emplace_back(point.orientation.w);
  }

  is_valid &= orientation_x_interpolator_->build(axis_, xs);
  is_valid &= orientation_y_interpolator_->build(axis_, ys);
  is_valid &= orientation_z_interpolator_->build(axis_, zs);
  is_valid &= orientation_w_interpolator_->build(axis_, ws);

  return is_valid;
}

PointType TrajectoryContainer<PointType>::compute(double s) const
{
  PointType result;
  result.position = BaseClass::compute(s);
  s = clamp(s);
  result.orientation.x = orientation_x_interpolator_->compute(s);
  result.orientation.y = orientation_y_interpolator_->compute(s);
  result.orientation.z = orientation_z_interpolator_->compute(s);
  result.orientation.w = orientation_w_interpolator_->compute(s);
  return result;
}

std::vector<PointType> TrajectoryContainer<PointType>::restore(const size_t & min_points) const
{
  auto axis = detail::crop_axis(axis_, start_, end_);
  axis = detail::fill_axis(axis, static_cast<Eigen::Index>(min_points));
  std::vector<PointType> points;
  points.reserve(axis.size());
  for (const auto & s : axis) {
    PointType p;
    p.position = BaseClass::compute(s);
    p.orientation.x = orientation_x_interpolator_->compute(s);
    p.orientation.y = orientation_y_interpolator_->compute(s);
    p.orientation.z = orientation_z_interpolator_->compute(s);
    p.orientation.w = orientation_w_interpolator_->compute(s);
    points.emplace_back(p);
  }
  return points;
}

}  // namespace autoware::motion_utils::trajectory_container::trajectory
