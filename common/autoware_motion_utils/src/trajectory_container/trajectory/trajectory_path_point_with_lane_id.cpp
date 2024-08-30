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

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_path_point_with_lane_id.hpp"

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_point.hpp"

#include <autoware_planning_msgs/msg/path_point.hpp>

#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{

using PointType = tier4_planning_msgs::msg::PathPointWithLaneId;

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
  const std::shared_ptr<interpolator::Interpolator<double>> & heading_rate_interpolator,
  const std::shared_ptr<interpolator::Interpolator<LaneIdType>> & lane_id_interpolator)
: BaseClass(
    x_interpolator, y_interpolator, z_interpolator, orientation_x_interpolator,
    orientation_y_interpolator, orientation_z_interpolator, orientation_w_interpolator,
    longitudinal_velocity_interpolator, lateral_velocity_interpolator, heading_rate_interpolator),
  lane_ids(lane_id_interpolator)
{
}

bool TrajectoryContainer<PointType>::build(const std::vector<PointType> & points)
{
  std::vector<autoware_planning_msgs::msg::PathPoint> path_points;
  std::vector<std::vector<int64_t>> lane_ids_values;

  for (const auto & point : points) {
    path_points.emplace_back(point.point);
    lane_ids_values.emplace_back(point.lane_ids);
  }
  bool is_valid = true;
  is_valid &= BaseClass::build(path_points);
  is_valid &= lane_ids.build(axis_, lane_ids_values);
  return is_valid;
}

PointType TrajectoryContainer<PointType>::compute(double s) const
{
  PointType result;
  result.point = BaseClass::compute(s);
  s = clamp(s);
  result.lane_ids = lane_ids.compute(s);
  return result;
}

std::vector<PointType> TrajectoryContainer<PointType>::restore(const size_t & min_points) const
{
  auto get_axis = [](const auto & manipulated_interpolated_array) {
    return manipulated_interpolated_array.get_data().first;
  };

  auto axis = detail::merge_vectors(
    axis_, get_axis(this->longitudinal_velocity_mps), get_axis(this->lateral_velocity_mps),
    get_axis(this->heading_rate_rps), get_axis(this->lane_ids));

  axis = detail::crop_axis(axis, start_, end_);
  axis = detail::fill_axis(axis, static_cast<Eigen::Index>(min_points));

  std::vector<PointType> points;
  points.reserve(axis.size());
  for (const auto & s : axis) {
    PointType p;
    p.point = BaseClass::compute(s);
    p.lane_ids = lane_ids.compute(s);
    points.emplace_back(p);
  }
  return points;
}

}  // namespace autoware::motion_utils::trajectory_container::trajectory
