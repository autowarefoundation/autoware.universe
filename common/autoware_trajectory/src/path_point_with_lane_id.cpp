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

#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include "autoware/trajectory/detail/utils.hpp"
#include "autoware/trajectory/interpolator/stairstep.hpp"

#include <memory>
#include <vector>

namespace autoware::trajectory
{

using PointType = tier4_planning_msgs::msg::PathPointWithLaneId;

Trajectory<PointType>::Trajectory()
: lane_ids(
    detail::InterpolatedArray<LaneIdType>(std::make_shared<interpolator::Stairstep<LaneIdType>>()))
{
}

Trajectory<PointType>::Trajectory(const Trajectory & rhs)
: Trajectory<autoware_planning_msgs::msg::PathPoint>(rhs)
{
  lane_ids = rhs.lane_ids;
}

Trajectory<PointType> & Trajectory<PointType>::operator=(const Trajectory & rhs)
{
  if (this != &rhs) {
    BaseClass::operator=(rhs);
    lane_ids = rhs.lane_ids;
  }
  return *this;
}

bool Trajectory<PointType>::build(const std::vector<PointType> & points)
{
  std::vector<autoware_planning_msgs::msg::PathPoint> path_points;
  std::vector<std::vector<int64_t>> lane_ids_values;

  for (const auto & point : points) {
    path_points.emplace_back(point.point);
    lane_ids_values.emplace_back(point.lane_ids);
  }
  bool is_valid = true;
  is_valid &= BaseClass::build(path_points);
  is_valid &= lane_ids.build(bases_, lane_ids_values);
  return is_valid;
}

std::vector<double> Trajectory<PointType>::get_internal_bases() const
{
  auto get_bases = [](const auto & interpolated_array) {
    auto [bases, values] = interpolated_array.get_data();
    return bases;
  };

  auto bases = detail::merge_vectors(
    bases_, get_bases(this->longitudinal_velocity_mps), get_bases(this->lateral_velocity_mps),
    get_bases(this->heading_rate_rps), get_bases(this->lane_ids));

  bases = detail::crop_bases(bases, start_, end_);

  std::transform(
    bases.begin(), bases.end(), bases.begin(), [this](const double & s) { return s - start_; });
  return bases;
}

PointType Trajectory<PointType>::compute(double s) const
{
  PointType result;
  result.point = BaseClass::compute(s);
  s = clamp(s);
  result.lane_ids = lane_ids.compute(s);
  return result;
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
