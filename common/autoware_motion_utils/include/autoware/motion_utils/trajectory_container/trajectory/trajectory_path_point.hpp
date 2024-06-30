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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_HPP_

#include "autoware/motion_utils/trajectory_container/detail/manipulable_interpolated_array.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_point.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_pose.hpp"

#include "autoware_planning_msgs/msg/path_point.hpp"

#include <memory>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{

using autoware_planning_msgs::msg::PathPoint;
using motion_utils::trajectory_container::detail::ManipulableInterpolatedArray;

/**
 * @brief Trajectory class for PathPoint
 */
template <>
class TrajectoryContainer<PathPoint>
: public TrajectoryContainer<geometry_msgs::msg::Pose>,
  public detail::CropTrajectoryImpl<TrajectoryContainer<PathPoint>>,
  public detail::SetXYZInterpolatorImpl<TrajectoryContainer<PathPoint>>,
  public detail::SetOrientationInterpolatorImpl<TrajectoryContainer<PathPoint>>,
  public detail::SetLongitudinalVelocityInterpolatorImpl<TrajectoryContainer<PathPoint>>,
  public detail::SetLateralVelocityInterpolatorImpl<TrajectoryContainer<PathPoint>>,
  public detail::SetHeadingRateInterpolatorImpl<TrajectoryContainer<PathPoint>>
{
  friend class detail::CropTrajectoryImpl<TrajectoryContainer<PathPoint>>;
  friend class detail::SetXYZInterpolatorImpl<TrajectoryContainer<PathPoint>>;
  friend class detail::SetOrientationInterpolatorImpl<TrajectoryContainer<PathPoint>>;
  using BaseClass = TrajectoryContainer<geometry_msgs::msg::Pose>;

public:
  ManipulableInterpolatedArray<double> longitudinal_velocity_mps;  //!< Longitudinal velocity in m/s
  ManipulableInterpolatedArray<double> lateral_velocity_mps;       //!< Lateral velocity in m/s
  ManipulableInterpolatedArray<double> heading_rate_rps;           //!< Heading rate in rad/s

  /**
   * @brief Constructor
   */
  TrajectoryContainer();
  /**
   * @brief Build trajectory from path points
   * @param points Vector of path points
   * @return Reference to this object
   */
  TrajectoryContainer & build(const std::vector<PathPoint> points);

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

  using detail::CropTrajectoryImpl<TrajectoryContainer<PathPoint>>::crop;
  using detail::CropTrajectoryImpl<TrajectoryContainer<PathPoint>>::cropInPlace;
  using detail::SetXYZInterpolatorImpl<TrajectoryContainer<PathPoint>>::set_xy_interpolator;
  using detail::SetXYZInterpolatorImpl<TrajectoryContainer<PathPoint>>::set_z_interpolator;
  using detail::SetOrientationInterpolatorImpl<
    TrajectoryContainer<PathPoint>>::set_orientation_interpolator;
};

}  // namespace autoware::motion_utils::trajectory_container::trajectory

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_HPP_
