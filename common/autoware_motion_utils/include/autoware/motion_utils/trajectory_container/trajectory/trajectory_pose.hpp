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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POSE_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POSE_HPP_

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_point.hpp"

#include <memory>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{

/**
 * @brief Trajectory class for geometry_msgs::msg::Pose
 */
template <>
class TrajectoryContainer<geometry_msgs::msg::Pose>
: public TrajectoryContainer<geometry_msgs::msg::Point>,
  public detail::CropTrajectoryImpl<TrajectoryContainer<geometry_msgs::msg::Pose>>
{
  friend class detail::CropTrajectoryImpl<TrajectoryContainer<geometry_msgs::msg::Pose>>;

  using BaseClass = TrajectoryContainer<geometry_msgs::msg::Point>;
  using PointType = geometry_msgs::msg::Pose;
  using CropTrajectoryImpl = detail::CropTrajectoryImpl<TrajectoryContainer<PointType>>;

private:
  std::shared_ptr<interpolator::Interpolator<double>> orientation_x_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_y_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_z_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_w_interpolator_;

public:
  /**
   * @brief Constructor
   */
  TrajectoryContainer(
    const std::shared_ptr<interpolator::Interpolator<double>> & x_interpolator,
    const std::shared_ptr<interpolator::Interpolator<double>> & y_interpolator,
    const std::shared_ptr<interpolator::Interpolator<double>> & z_interpolator,
    const std::shared_ptr<interpolator::Interpolator<double>> & orientation_x_interpolator,
    const std::shared_ptr<interpolator::Interpolator<double>> & orientation_y_interpolator,
    const std::shared_ptr<interpolator::Interpolator<double>> & orientation_z_interpolator,
    const std::shared_ptr<interpolator::Interpolator<double>> & orientation_w_interpolator);

  bool build(const std::vector<PointType> & points);

  /**
   * @brief Compute the pose on the trajectory at a given s value
   * @param s Arc length
   * @return Pose on the trajectory
   */
  [[nodiscard]] PointType compute(double s) const;

  /**
   * @brief Restore the trajectory poses
   * @return Vector of poses
   */
  [[nodiscard]] std::vector<PointType> restore(const size_t & min_points = 100) const;

  using CropTrajectoryImpl::crop;
  using CropTrajectoryImpl::crop_in_place;
};

}  // namespace autoware::motion_utils::trajectory_container::trajectory

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POSE_HPP_
