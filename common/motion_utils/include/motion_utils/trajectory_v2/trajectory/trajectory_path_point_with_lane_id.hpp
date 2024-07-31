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

#ifndef MOTION_UTILS__TRAJECTORY_V2__TRAJECTORY__TRAJECTORY_PATH_POINT_WITH_LANE_ID_HPP_
#define MOTION_UTILS__TRAJECTORY_V2__TRAJECTORY__TRAJECTORY_PATH_POINT_WITH_LANE_ID_HPP_
#include "motion_utils/trajectory_v2/detail/manipulable_interpolated_array.hpp"
#include "motion_utils/trajectory_v2/trajectory/trajectory.hpp"
#include "motion_utils/trajectory_v2/trajectory/trajectory_path_point.hpp"

#include "tier4_planning_msgs/msg/path_point_with_lane_id.hpp"

#include <memory>
#include <vector>

namespace motion_utils::trajectory_v2::trajectory
{
using motion_utils::trajectory_v2::detail::ManipulableInterpolatedArray;
using tier4_planning_msgs::msg::PathPointWithLaneId;

/**
 * @brief Trajectory class for PathPointWithLaneId
 */
template <>
class TrajectoryV2<PathPointWithLaneId> : public TrajectoryV2<PathPoint>,
                                          public detail::CropTrajectoryImpl<PathPointWithLaneId>
{
  friend class CropTrajectoryImpl<PathPointWithLaneId>;
  using BaseClass = TrajectoryV2<PathPoint>;

public:
  ManipulableInterpolatedArray<std::vector<int64_t>> lane_ids;

  /**
   * @brief Constructor
   */
  TrajectoryV2();

  /**
   * @brief Set the interpolator for lane ids
   * @param interpolator Interpolator object
   * @return Reference to this object
   */

  TrajectoryV2 & set_lane_ids_interpolator(
    const interpolator::Interpolator<std::vector<int64_t>> & interpolator);

  /**
   * @brief Build trajectory from path points with lane ids
   * @param points Vector of path points with lane ids
   * @return Reference to this object
   */
  TrajectoryV2 & build(const std::vector<PathPointWithLaneId> & points);

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
};

}  // namespace motion_utils::trajectory_v2::trajectory

#endif  // MOTION_UTILS__TRAJECTORY_V2__TRAJECTORY__TRAJECTORY_PATH_POINT_WITH_LANE_ID_HPP_
