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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__DETAIL__CROP_IMPL_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__DETAIL__CROP_IMPL_HPP_

#include "autoware_auto_common/helper_functions/crtp.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace autoware::motion_utils::trajectory_container::trajectory::detail
{

/**
 * @brief Helper class to crop a trajectory in place or by returning a cropped copy.
 * @tparam TrajectoryType Type of the trajectory.
 */
template <typename TrajectoryType>
class CropTrajectoryImpl : public autoware::common::helper_functions::crtp<
                             TrajectoryType, CropTrajectoryImpl<TrajectoryType>>
{
public:
  /**
   * @brief Crops the trajectory in place.
   * @param start Start position.
   * @param end End position.
   * @return Reference to the cropped trajectory.
   */
  TrajectoryType & crop_in_place(const double & start, const double & end)
  {
    double length = this->impl().length();
    this->impl().start_ += start;
    this->impl().end_ -= length - end;
    return this->impl();
  }

  /**
   * @brief Returns a cropped copy of the trajectory.
   * @param start Start position.
   * @param end End position.
   * @return Cropped trajectory.
   */
  TrajectoryType crop(const double & start, const double & end) const
  {
    TrajectoryType trajectory = this->impl();
    trajectory.crop_in_place(start, end);
    return trajectory;
  }
};

}  // namespace autoware::motion_utils::trajectory_container::trajectory::detail

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__DETAIL__CROP_IMPL_HPP_
