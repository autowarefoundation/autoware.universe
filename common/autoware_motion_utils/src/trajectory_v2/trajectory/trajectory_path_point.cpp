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

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_path_point.hpp"

#include "autoware/motion_utils/trajectory_container/detail/merge_vector.hpp"
#include "autoware/motion_utils/trajectory_container/interpolator/zero_order_hold.hpp"

namespace autoware::motion_utils::trajectory_container::trajectory
{

TrajectoryV2<PathPoint>::TrajectoryV2() : BaseClass()
{
  set_longitudinal_velocity_mps_interpolator(interpolator::ZeroOrderHold<double>());
  set_lateral_velocity_mps_interpolator(interpolator::ZeroOrderHold<double>());
  set_heading_rate_rps_interpolator(interpolator::ZeroOrderHold<double>());
}

TrajectoryV2<PathPoint> & TrajectoryV2<PathPoint>::set_longitudinal_velocity_mps_interpolator(
  const interpolator::Interpolator<double> & interpolator)
{
  longitudinal_velocity_mps.set_interpolator(interpolator);
  return *this;
}

TrajectoryV2<PathPoint> & TrajectoryV2<PathPoint>::set_lateral_velocity_mps_interpolator(
  const interpolator::Interpolator<double> & interpolator)
{
  lateral_velocity_mps.set_interpolator(interpolator);
  return *this;
}

TrajectoryV2<PathPoint> & TrajectoryV2<PathPoint>::set_heading_rate_rps_interpolator(
  const interpolator::Interpolator<double> & interpolator)
{
  heading_rate_rps.set_interpolator(interpolator);
  return *this;
}

TrajectoryV2<PathPoint> & TrajectoryV2<PathPoint>::build(const std::vector<PathPoint> points)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<double> longitudinal_velocity_mps_values, lateral_velocity_mps_values,
    heading_rate_rps_values;

  for (const auto & point : points) {
    poses.emplace_back(point.pose);
    longitudinal_velocity_mps_values.emplace_back(point.longitudinal_velocity_mps);
    lateral_velocity_mps_values.emplace_back(point.lateral_velocity_mps);
    heading_rate_rps_values.emplace_back(point.heading_rate_rps);
  }

  TrajectoryV2<geometry_msgs::msg::Pose>::build(poses);
  this->longitudinal_velocity_mps.build(axis_, longitudinal_velocity_mps_values);
  this->lateral_velocity_mps.build(axis_, lateral_velocity_mps_values);
  this->heading_rate_rps.build(axis_, heading_rate_rps_values);

  return *this;
}

PathPoint TrajectoryV2<PathPoint>::compute(const double & s) const
{
  PathPoint result;
  result.pose = TrajectoryV2<geometry_msgs::msg::Pose>::compute(s);
  result.longitudinal_velocity_mps = longitudinal_velocity_mps.compute(s);
  result.lateral_velocity_mps = lateral_velocity_mps.compute(s);
  result.heading_rate_rps = heading_rate_rps.compute(s);
  return result;
}

std::vector<PathPoint> TrajectoryV2<PathPoint>::restore() const
{
  auto new_axis = trajectory_container::detail::merge_vectors(
    axis_, longitudinal_velocity_mps.axis_, lateral_velocity_mps.axis_, heading_rate_rps.axis_);

  std::vector<PathPoint> points(new_axis.size());
  std::transform(new_axis.begin(), new_axis.end(), points.begin(), [this](const auto & s) {
    PathPoint p;
    p.pose = TrajectoryV2<geometry_msgs::msg::Pose>::compute(s);
    p.longitudinal_velocity_mps = longitudinal_velocity_mps.compute(s);
    p.lateral_velocity_mps = lateral_velocity_mps.compute(s);
    p.heading_rate_rps = heading_rate_rps.compute(s);
    return p;
  });

  return points;
}

}  // namespace autoware::motion_utils::trajectory_container::trajectory
