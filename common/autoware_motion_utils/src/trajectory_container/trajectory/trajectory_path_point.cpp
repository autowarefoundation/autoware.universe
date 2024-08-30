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

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_path_point.hpp"

#include "autoware/motion_utils/trajectory_container/trajectory/detail/utils.hpp"

#include <autoware_planning_msgs/msg/path_point.hpp>

#include <cstddef>

namespace autoware::motion_utils::trajectory_container::trajectory
{

using PointType = autoware_planning_msgs::msg::PathPoint;

TrajectoryContainer<PointType>::TrajectoryContainer(
  const std::shared_ptr<interpolator::Interpolator<double>> & x_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & y_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & z_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & orientation_x_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & orientation_y_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & orientation_z_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & orientation_w_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & longitudinal_velocity_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & lateral_velocity_interpolator,
  const std::shared_ptr<interpolator::Interpolator<double>> & heading_rate_interpolator)
: BaseClass(
    x_interpolator, y_interpolator, z_interpolator, orientation_x_interpolator,
    orientation_y_interpolator, orientation_z_interpolator, orientation_w_interpolator),
  longitudinal_velocity_mps(longitudinal_velocity_interpolator),
  lateral_velocity_mps(lateral_velocity_interpolator),
  heading_rate_rps(heading_rate_interpolator)
{
}

bool TrajectoryContainer<PointType>::build(const std::vector<PointType> & points)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<double> longitudinal_velocity_mps_values;
  std::vector<double> lateral_velocity_mps_values;
  std::vector<double> heading_rate_rps_values;

  for (const auto & point : points) {
    poses.emplace_back(point.pose);
    longitudinal_velocity_mps_values.emplace_back(point.longitudinal_velocity_mps);
    lateral_velocity_mps_values.emplace_back(point.lateral_velocity_mps);
    heading_rate_rps_values.emplace_back(point.heading_rate_rps);
  }

  bool is_valid = true;

  is_valid &= TrajectoryContainer<geometry_msgs::msg::Pose>::build(poses);
  is_valid &= this->longitudinal_velocity_mps.build(axis_, longitudinal_velocity_mps_values);
  is_valid &= this->lateral_velocity_mps.build(axis_, lateral_velocity_mps_values);
  is_valid &= this->heading_rate_rps.build(axis_, heading_rate_rps_values);

  return is_valid;
}

PointType TrajectoryContainer<PointType>::compute(double s) const
{
  PointType result;
  result.pose = TrajectoryContainer<geometry_msgs::msg::Pose>::compute(s);
  s = clamp(s);
  result.longitudinal_velocity_mps = static_cast<float>(this->longitudinal_velocity_mps.compute(s));
  result.lateral_velocity_mps = static_cast<float>(this->lateral_velocity_mps.compute(s));
  result.heading_rate_rps = static_cast<float>(this->heading_rate_rps.compute(s));
  return result;
}

std::vector<PointType> TrajectoryContainer<PointType>::restore(const size_t & min_points) const
{
  auto get_axis = [](const auto & manipulated_interpolated_array) {
    return manipulated_interpolated_array.get_data().first;
  };

  auto axis = detail::merge_vectors(
    axis_, get_axis(this->longitudinal_velocity_mps), get_axis(this->lateral_velocity_mps),
    get_axis(this->heading_rate_rps));

  axis = detail::crop_axis(axis, start_, end_);
  axis = detail::fill_axis(axis, static_cast<Eigen::Index>(min_points));

  std::vector<PointType> points;
  points.reserve(axis.size());
  for (const auto & s : axis) {
    PointType p;
    p.pose = TrajectoryContainer<geometry_msgs::msg::Pose>::compute(s);
    p.longitudinal_velocity_mps = static_cast<float>(this->longitudinal_velocity_mps.compute(s));
    p.lateral_velocity_mps = static_cast<float>(this->lateral_velocity_mps.compute(s));
    p.heading_rate_rps = static_cast<float>(this->heading_rate_rps.compute(s));
    points.emplace_back(p);
  }

  return points;
}
}  // namespace autoware::motion_utils::trajectory_container::trajectory
