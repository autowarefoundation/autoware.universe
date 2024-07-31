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

#ifndef MOTION_UTILS__TRAJECTORY_V2__TRAJECTORY__TRAJECTORY_HPP_
#define MOTION_UTILS__TRAJECTORY_V2__TRAJECTORY__TRAJECTORY_HPP_

namespace motion_utils::trajectory_v2::trajectory
{

/**
 * @brief Class for trajectory generation.
 * @tparam PointType Type of the points in the trajectory.
 */
template <typename PointType>
class TrajectoryV2
{
};

namespace detail
{

/**
 * @brief Helper class to crop a trajectory in place or by returning a cropped copy.
 * @tparam PointType Type of the points in the trajectory.
 */
template <typename PointType>
class CropTrajectoryImpl
{
protected:
  using TrajectoryType = TrajectoryV2<PointType>;

private:
  /**
   * @brief Casts this object to the derived type.
   * @return Reference to the derived type.
   */
  TrajectoryType & derived() { return *static_cast<TrajectoryType *>(this); }

  /**
   * @brief Casts this object to the const derived type.
   * @return Const reference to the derived type.
   */
  const TrajectoryType & derived() const { return *static_cast<const TrajectoryType *>(this); }

public:
  /**
   * @brief Crops the trajectory in place.
   * @param start Start position.
   * @param end End position.
   * @return Reference to the cropped trajectory.
   */
  TrajectoryType & cropInPlace(const double & start, const double & end)
  {
    derived().start_ = start;
    derived().end_ = end;
    return derived();
  }

  /**
   * @brief Returns a cropped copy of the trajectory.
   * @param start Start position.
   * @param end End position.
   * @return Cropped trajectory.
   */
  TrajectoryType crop(const double & start, const double & end) const
  {
    TrajectoryType trajectory = derived();
    trajectory.cropInPlace(start, end);
    return trajectory;
  }
};

}  // namespace detail

}  // namespace motion_utils::trajectory_v2::trajectory

#endif  // MOTION_UTILS__TRAJECTORY_V2__TRAJECTORY__TRAJECTORY_HPP_
