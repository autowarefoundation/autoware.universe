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

#include "autoware/motion_utils/trajectory_container/interpolator/cubic_spline.hpp"
#include "autoware/motion_utils/trajectory_container/interpolator/interpolator.hpp"
#include "autoware/motion_utils/trajectory_container/interpolator/spherical_linear.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_point.hpp"

#include <geometry_msgs/msg/quaternion.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{

/**
 * @brief Trajectory class for geometry_msgs::msg::Pose
 */
template <>
class TrajectoryContainer<geometry_msgs::msg::Pose>
: public TrajectoryContainer<geometry_msgs::msg::Point>
{
  using BaseClass = TrajectoryContainer<geometry_msgs::msg::Point>;
  using PointType = geometry_msgs::msg::Pose;

protected:
  std::shared_ptr<interpolator::InterpolatorInterface<geometry_msgs::msg::Quaternion>>
    orientation_interpolator_;  //!< Interpolator for orientations

public:
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

  /**
   * @brief Align the orientation with the direction
   */
  void align_orientation_with_trajectory_direction();

  class Builder
  {
  private:
    std::unique_ptr<TrajectoryContainer> trajectory_;

  public:
    Builder()
    {
      trajectory_ = std::make_unique<TrajectoryContainer>();
      set_xy_interpolator<interpolator::CubicSpline>();
      set_z_interpolator<interpolator::Linear>();
      set_orientation_interpolator<interpolator::SphericalLinear>();
    }

    template <class InterpolatorType, class... Args>
    Builder & set_xy_interpolator(Args &&... args)
    {
      trajectory_->x_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      trajectory_->y_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_z_interpolator(Args &&... args)
    {
      trajectory_->z_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_orientation_interpolator(Args &&... args)
    {
      trajectory_->orientation_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    std::optional<TrajectoryContainer> build(const std::vector<PointType> & points)
    {
      if (trajectory_->build(points)) {
        auto result = std::make_optional<TrajectoryContainer>(std::move(*trajectory_));
        trajectory_.reset();
        return result;
      }
      return std::nullopt;
    }
  };
};

}  // namespace autoware::motion_utils::trajectory_container::trajectory

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POSE_HPP_
