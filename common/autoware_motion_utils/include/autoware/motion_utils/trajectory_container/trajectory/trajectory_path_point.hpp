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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_HPP_

#include "autoware/motion_utils/trajectory_container/trajectory/detail/manipulable_interpolated_array.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_pose.hpp"

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{

template <>
class TrajectoryContainer<autoware_planning_msgs::msg::PathPoint>
: public TrajectoryContainer<geometry_msgs::msg::Pose>,
  public detail::CropTrajectoryImpl<TrajectoryContainer<autoware_planning_msgs::msg::PathPoint>>
{
  friend class detail::CropTrajectoryImpl<
    TrajectoryContainer<autoware_planning_msgs::msg::PathPoint>>;

  using BaseClass = TrajectoryContainer<geometry_msgs::msg::Pose>;
  using PointType = autoware_planning_msgs::msg::PathPoint;
  template <typename ValueType>
  using ArrayType = detail::ManipulableInterpolatedArray<ValueType>;
  using CropTrajectoryImpl = detail::CropTrajectoryImpl<TrajectoryContainer<PointType>>;

public:
  ArrayType<double> longitudinal_velocity_mps;  //!< Longitudinal velocity in m/s
  ArrayType<double> lateral_velocity_mps;       //!< Lateral velocity in m/s
  ArrayType<double> heading_rate_rps;           //!< Heading rate in rad/s};

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
    const std::shared_ptr<interpolator::Interpolator<double>> & heading_rate_interpolator);

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

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_PATH_POINT_HPP_
