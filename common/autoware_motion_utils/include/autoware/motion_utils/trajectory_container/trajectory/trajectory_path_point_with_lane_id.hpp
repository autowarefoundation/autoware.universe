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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_WITH_LANE_ID_HPP_  // NOLINT
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_WITH_LANE_ID_HPP_  // NOLINT
#include "autoware/motion_utils/trajectory_container/detail/manipulable_interpolated_array.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_path_point.hpp"

#include "tier4_planning_msgs/msg/path_point_with_lane_id.hpp"

#include <memory>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{
using tier4_planning_msgs::msg::PathPointWithLaneId;

/**
 * @brief Trajectory class for PathPointWithLaneId
 */
template <>
class TrajectoryContainer<PathPointWithLaneId>
: public TrajectoryContainer<PathPoint>,
  public detail::CropTrajectoryImpl<TrajectoryContainer<PathPointWithLaneId>>,
  public detail::SetXYZInterpolatorImpl<TrajectoryContainer<PathPointWithLaneId>>,
  public detail::SetOrientationInterpolatorImpl<TrajectoryContainer<PathPointWithLaneId>>,
  public detail::SetLongitudinalVelocityInterpolatorImpl<TrajectoryContainer<PathPointWithLaneId>>,
  public detail::SetLateralVelocityInterpolatorImpl<TrajectoryContainer<PathPointWithLaneId>>,
  public detail::SetHeadingRateInterpolatorImpl<TrajectoryContainer<PathPointWithLaneId>>,
  public detail::SetLaneIdsInterpolatorImpl<TrajectoryContainer<PathPointWithLaneId>>
{
  friend class detail::CropTrajectoryImpl<TrajectoryContainer<PathPointWithLaneId>>;
  friend class detail::SetXYZInterpolatorImpl<TrajectoryContainer<PathPointWithLaneId>>;
  friend class detail::SetOrientationInterpolatorImpl<TrajectoryContainer<PathPointWithLaneId>>;

  using BaseClass = TrajectoryContainer<PathPoint>;

public:
  ManipulableInterpolatedArray<std::vector<int64_t>> lane_ids;

  /**
   * @brief Constructor
   */
  TrajectoryContainer();

  /**
   * @brief Build trajectory from path points with lane ids
   * @param points Vector of path points with lane ids
   * @return Reference to this object
   */
  TrajectoryContainer & build(const std::vector<PathPointWithLaneId> & points);

  /**
   * @brief Compute the path point on the trajectory at a given s value
   * @param s Arc length
   * @return Path point on the trajectory
   */
  PathPointWithLaneId compute(const double & s) const;

  /**
   * @brief Restore the trajectory path points with lane ids
   * @return Vector of path points with lane ids
   */
  std::vector<PathPointWithLaneId> restore() const;

  using detail::CropTrajectoryImpl<TrajectoryContainer<PathPointWithLaneId>>::crop;
  using detail::SetXYZInterpolatorImpl<
    TrajectoryContainer<PathPointWithLaneId>>::set_xy_interpolator;
  using detail::SetXYZInterpolatorImpl<
    TrajectoryContainer<PathPointWithLaneId>>::set_z_interpolator;
  using detail::SetOrientationInterpolatorImpl<
    TrajectoryContainer<PathPointWithLaneId>>::set_orientation_interpolator;
  using detail::SetLongitudinalVelocityInterpolatorImpl<
    TrajectoryContainer<PathPointWithLaneId>>::set_longitudinal_velocity_mps_interpolator;
  using detail::SetLateralVelocityInterpolatorImpl<
    TrajectoryContainer<PathPointWithLaneId>>::set_lateral_velocity_mps_interpolator;
  using detail::SetHeadingRateInterpolatorImpl<
    TrajectoryContainer<PathPointWithLaneId>>::set_heading_rate_rps_interpolator;
};

}  // namespace autoware::motion_utils::trajectory_container::trajectory

// clang-format off
#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_WITH_LANE_ID_HPP_  // NOLINT
// clang-format on
