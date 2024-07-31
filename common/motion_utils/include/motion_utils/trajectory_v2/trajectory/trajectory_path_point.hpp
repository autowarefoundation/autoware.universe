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

#ifndef MOTION_UTILS__TRAJECTORY_V2__TRAJECTORY__TRAJECTORY_PATH_POINT_HPP_
#define MOTION_UTILS__TRAJECTORY_V2__TRAJECTORY__TRAJECTORY_PATH_POINT_HPP_

#include "motion_utils/trajectory_v2/detail/manipulable_interpolated_array.hpp"
#include "motion_utils/trajectory_v2/trajectory/trajectory.hpp"
#include "motion_utils/trajectory_v2/trajectory/trajectory_point.hpp"
#include "motion_utils/trajectory_v2/trajectory/trajectory_pose.hpp"

#include "autoware_planning_msgs/msg/path_point.hpp"

#include <memory>
#include <vector>

namespace motion_utils::trajectory_v2::trajectory
{

using autoware_planning_msgs::msg::PathPoint;
using motion_utils::trajectory_v2::detail::ManipulableInterpolatedArray;

/**
 * @brief Trajectory class for PathPoint
 */
template <>
class TrajectoryV2<PathPoint> : public TrajectoryV2<geometry_msgs::msg::Pose>,
                                public detail::CropTrajectoryImpl<PathPoint>
{
  friend class CropTrajectoryImpl<PathPoint>;
  using BaseClass = TrajectoryV2<geometry_msgs::msg::Pose>;

public:
  ManipulableInterpolatedArray<double> longitudinal_velocity_mps;
  ManipulableInterpolatedArray<double> lateral_velocity_mps;
  ManipulableInterpolatedArray<double> heading_rate_rps;

  /**
   * @brief Constructor
   */
  TrajectoryV2();

  /**
   * @brief Set the interpolator for longitudinal velocity
   * @param interpolator Interpolator object
   * @return Reference to this object
   */
  TrajectoryV2 & set_longitudinal_velocity_mps_interpolator(
    const interpolator::Interpolator<double> & interpolator);

  /**
   * @brief Set the interpolator for lateral velocity
   * @param interpolator Interpolator object
   * @return Reference to this object
   */
  TrajectoryV2 & set_lateral_velocity_mps_interpolator(
    const interpolator::Interpolator<double> & interpolator);

  /**
   * @brief Set the interpolator for heading rate
   * @param interpolator Interpolator object
   * @return Reference to this object
   */
  TrajectoryV2 & set_heading_rate_rps_interpolator(
    const interpolator::Interpolator<double> & interpolator);

  /**
   * @brief Build trajectory from path points
   * @param points Vector of path points
   * @return Reference to this object
   */
  TrajectoryV2 & build(const std::vector<PathPoint> points);

  /**
   * @brief Compute the path point on the trajectory at a given s value
   * @param s Arc length
   * @return Path point on the trajectory
   */
  PathPoint compute(const double & s) const;

  /**
   * @brief Restore the trajectory path points
   * @return Vector of path points
   */
  std::vector<PathPoint> restore() const;

  using CropTrajectoryImpl<PathPoint>::crop;
  using CropTrajectoryImpl<PathPoint>::cropInPlace;
};

}  // namespace motion_utils::trajectory_v2::trajectory

#endif  // MOTION_UTILS__TRAJECTORY_V2__TRAJECTORY__TRAJECTORY_PATH_POINT_HPP_
