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

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_path_point_with_lane_id.hpp"

#include "autoware/motion_utils/trajectory_container/detail/merge_vector.hpp"
#include "autoware/motion_utils/trajectory_container/interpolator/zero_order_hold.hpp"

namespace autoware::motion_utils::trajectory_container::trajectory
{

TrajectoryContainer<PathPointWithLaneId>::TrajectoryContainer() : BaseClass()
{
  set_lane_ids_interpolator(interpolator::ZeroOrderHold<std::vector<int64_t>>());
}

TrajectoryContainer<PathPointWithLaneId> &
TrajectoryContainer<PathPointWithLaneId>::set_lane_ids_interpolator(
  const interpolator::Interpolator<std::vector<int64_t>> & interpolator)
{
  lane_ids.set_interpolator(interpolator);
  return *this;
}

TrajectoryContainer<PathPointWithLaneId> & TrajectoryContainer<PathPointWithLaneId>::build(
  const std::vector<PathPointWithLaneId> & points)
{
  std::vector<PathPoint> path_points;
  std::vector<std::vector<int64_t>> lane_ids_values;

  for (const auto & point : points) {
    path_points.emplace_back(point.point);
    lane_ids_values.emplace_back(point.lane_ids);
  }

  BaseClass::build(path_points);
  this->lane_ids.build(axis_, lane_ids_values);

  return *this;
}

PathPointWithLaneId TrajectoryContainer<PathPointWithLaneId>::compute(const double & s) const
{
  PathPointWithLaneId result;
  result.point = BaseClass::compute(s);
  result.lane_ids = lane_ids.compute(s);
  return result;
}

std::vector<PathPointWithLaneId> TrajectoryContainer<PathPointWithLaneId>::restore() const
{
  auto new_axis = trajectory_container::detail::merge_vectors(
    axis_, lane_ids.axis_, longitudinal_velocity_mps.axis_, lateral_velocity_mps.axis_,
    heading_rate_rps.axis_);

  std::vector<PathPointWithLaneId> points(new_axis.size());
  std::transform(new_axis.begin(), new_axis.end(), points.begin(), [this](const auto & s) {
    PathPointWithLaneId p;
    p.point = BaseClass::compute(s);
    p.lane_ids = lane_ids.compute(s);
    return p;
  });

  return points;
}

}  // namespace autoware::motion_utils::trajectory_container::trajectory
