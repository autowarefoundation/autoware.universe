// Copyright 2023 The Autoware Foundation
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

#ifndef PLANNING_FOLLOWER_TEST_UTILS_HPP_
#define PLANNING_FOLLOWER_TEST_UTILS_HPP_

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <memory>
#include <string>

namespace test_utils
{
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromRPY;

geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = createPoint(x, y, z);
  p.orientation = createQuaternionFromRPY(roll, pitch, yaw);
  return p;
}

template <class T>
T generateTrajectory(
  const size_t num_points, const double point_interval, const double vel = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0)
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = vel;
    traj.points.push_back(p);
  }

  return traj;
}

template <class T>
T generateOverlappingPointTrajectory(
  const size_t num_points, const double point_interval, const size_t overlap_point_idx = 0,
  const double vel = 0.0, const double init_theta = 0.0, const double delta_theta = 0.0)
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = vel;
    traj.points.push_back(p);
  }

  Point value_to_insert = traj.points[overlap_point_idx];
  traj.points.insert(traj.points.begin() + overlap_point_idx + 1, value_to_insert);

  return traj;
}

}  // namespace test_utils

#endif  // PLANNING_FOLLOWER_TEST_UTILS_HPP_
