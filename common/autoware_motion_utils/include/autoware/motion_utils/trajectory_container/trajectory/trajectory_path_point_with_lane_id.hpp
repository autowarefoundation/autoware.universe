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

// clang-format off
#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_WITH_LANE_ID_HPP_  // NOLINT
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_WITH_LANE_ID_HPP_  // NOLINT
// clang-format on

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_path_point.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_point.hpp"

#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{
template <>
class TrajectoryContainer<tier4_planning_msgs::msg::PathPointWithLaneId>
: public TrajectoryContainer<autoware_planning_msgs::msg::PathPoint>,
  public detail::CropTrajectoryImpl<
    TrajectoryContainer<tier4_planning_msgs::msg::PathPointWithLaneId>>
{
  friend class detail::CropTrajectoryImpl<
    TrajectoryContainer<tier4_planning_msgs::msg::PathPointWithLaneId>>;

  using BaseClass = TrajectoryContainer<autoware_planning_msgs::msg::PathPoint>;
  using PointType = tier4_planning_msgs::msg::PathPointWithLaneId;
  using CropTrajectoryImpl =
    detail::CropTrajectoryImpl<TrajectoryContainer<tier4_planning_msgs::msg::PathPointWithLaneId>>;
  using LaneIdType = std::vector<int64_t>;
  template <typename ValueType>
  using ArrayType = detail::ManipulableInterpolatedArray<ValueType>;

public:
  ArrayType<LaneIdType> lane_ids;  //!< Lane ID

  TrajectoryContainer(
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
    const std::shared_ptr<interpolator::Interpolator<LaneIdType>> & lane_id_interpolator);

  /**
   * @brief Build the trajectory from the points
   * @param points Vector of points
   * @return True if the build is successful
   */
  bool build(const std::vector<PointType> & points);

  /**
   * @brief Compute the point on the trajectory at a given s value
   * @param s Arc length
   * @return Point on the trajectory
   */
  [[nodiscard]] PointType compute(double s) const;

  /**
   * @brief Restore the trajectory points
   * @param min_points Minimum number of points
   * @return Vector of points
   */
  [[nodiscard]] std::vector<PointType> restore(const size_t & min_points = 100) const;

  using CropTrajectoryImpl::crop;
  using CropTrajectoryImpl::crop_in_place;
};
}  // namespace autoware::motion_utils::trajectory_container::trajectory

// clang-format off
#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_WITH_LANE_ID_HPP_  // NOLINT
// clang-format on
