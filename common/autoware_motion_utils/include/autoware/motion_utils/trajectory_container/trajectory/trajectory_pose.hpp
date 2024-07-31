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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POSE_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POSE_HPP_

#include "autoware/motion_utils/trajectory_container/interpolator/interpolator.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_point.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{

/**
 * @brief Trajectory class for geometry_msgs::msg::Pose
 */
template <>
class TrajectoryV2<geometry_msgs::msg::Pose>
: public TrajectoryV2<geometry_msgs::msg::Point>,
  public detail::CropTrajectoryImpl<geometry_msgs::msg::Pose>
{
  friend class CropTrajectoryImpl<geometry_msgs::msg::Pose>;
  using BaseClass = TrajectoryV2<geometry_msgs::msg::Point>;

private:
  std::shared_ptr<interpolator::Interpolator<double>> orientation_x_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_y_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_z_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_w_interpolator_;

public:
  /**
   * @brief Constructor
   */
  TrajectoryV2();

  /**
   * @brief Set the interpolator for orientation
   * @param interpolator Interpolator object
   * @return Reference to this object
   */
  TrajectoryV2 & set_orientation_interpolator(
    const interpolator::Interpolator<double> & interpolator);

  /**
   * @brief Build trajectory from poses
   * @param points Vector of poses
   * @return Reference to this object
   */
  TrajectoryV2 & build(const std::vector<geometry_msgs::msg::Pose> & points);

  /**
   * @brief Compute the pose on the trajectory at a given s value
   * @param s Arc length
   * @return Pose on the trajectory
   */
  geometry_msgs::msg::Pose compute(const double & s) const;

  /**
   * @brief Restore the trajectory poses
   * @return Vector of poses
   */
  std::vector<geometry_msgs::msg::Pose> restore() const;

  using CropTrajectoryImpl<geometry_msgs::msg::Pose>::crop;
  using CropTrajectoryImpl<geometry_msgs::msg::Pose>::cropInPlace;
};

}  // namespace autoware::motion_utils::trajectory_container::trajectory

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POSE_HPP_
