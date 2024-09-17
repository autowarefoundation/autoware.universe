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

#include "autoware/lane_departure_checker/util.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>

namespace autoware::lane_departure_checker
{
TrajectoryPoints cutTrajectory(const TrajectoryPoints & trajectory, const double length)
{
  TrajectoryPoints cut;

  double total_length = 0.0;
  cut.push_back(trajectory.front());
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto & point = trajectory.at(i);

    const auto p1 = autoware::universe_utils::fromMsg(cut.back().pose.position);
    const auto p2 = autoware::universe_utils::fromMsg(point.pose.position);
    const auto points_distance = boost::geometry::distance(p1.to_2d(), p2.to_2d());

    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0) {
      break;
    }

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated = p1 + remain_distance * (p2 - p1).normalized();

      TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = point.pose.orientation;

      cut.push_back(p);
      break;
    }

    cut.push_back(point);
    total_length += points_distance;
  }

  return cut;
}
}  // namespace autoware::lane_departure_checker
