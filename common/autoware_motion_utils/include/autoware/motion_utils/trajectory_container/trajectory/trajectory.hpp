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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_HPP_

#include "autoware_auto_common/helper_functions/crtp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <fmt/format.h>

#include <memory>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{

/**
 * @brief Class for trajectory generation.
 * @tparam PointType Type of the points in the trajectory.
 */
template <typename PointType>
class TrajectoryContainer;

namespace detail
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
  TrajectoryType & cropInPlace(const double & start, const double & end)
  {
    this->impl().start_ += start;
    this->impl().end_ = this->impl().start_ + end;
    try {
      this->impl().validate_s(this->impl().start_);
    } catch (const std::exception & e) {
      std::cerr << fmt::format(
                     "Failed to crop trajectory: start={} end={} error={}", start, end, e.what())
                << std::endl;
    }
    try {
      this->impl().validate_s(this->impl().end_);
    } catch (const std::exception & e) {
      std::cerr << fmt::format(
                     "Failed to crop trajectory: start={} end={} error={}", start, end, e.what())
                << std::endl;
    }
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
    trajectory.cropInPlace(start, end);
    return trajectory;
  }
};

/**
 * @brief Helper class to set XYZ interpolators for the trajectory.
 * @tparam TrajectoryType Type of the trajectory.
 */
template <typename TrajectoryType>
class SetXYZInterpolatorImpl : public autoware::common::helper_functions::crtp<
                                 TrajectoryType, SetXYZInterpolatorImpl<TrajectoryType>>
{
public:
  /**
   * @brief Sets the XY interpolator.
   * @param interpolator Interpolator object.
   * @return Reference to the trajectory with XY interpolator set.
   */
  TrajectoryType & set_xy_interpolator(const interpolator::Interpolator<double> & interpolator)
  {
    this->impl().x_interpolator_ =
      std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
    this->impl().y_interpolator_ =
      std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
    return this->impl();
  }

  /**
   * @brief Sets the Z interpolator.
   * @param interpolator Interpolator object.
   * @return Reference to the trajectory with Z interpolator set.
   */
  TrajectoryType & set_z_interpolator(const interpolator::Interpolator<double> & interpolator)
  {
    this->impl().z_interpolator_ =
      std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
    return this->impl();
  }
};

/**
 * @brief Helper class to set orientation interpolators for the trajectory.
 * @tparam TrajectoryType Type of the trajectory.
 */
template <typename TrajectoryType>
class SetOrientationInterpolatorImpl
: public autoware::common::helper_functions::crtp<
    TrajectoryType, SetOrientationInterpolatorImpl<TrajectoryType>>
{
public:
  /**
   * @brief Sets the orientation interpolator.
   * @param interpolator Interpolator object.
   * @return Reference to the trajectory with orientation interpolator set.
   */
  TrajectoryType & set_orientation_interpolator(
    const interpolator::Interpolator<double> & interpolator)
  {
    this->impl().orientation_x_interpolator_ =
      std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
    this->impl().orientation_y_interpolator_ =
      std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
    this->impl().orientation_z_interpolator_ =
      std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
    this->impl().orientation_w_interpolator_ =
      std::shared_ptr<interpolator::Interpolator<double>>(interpolator.clone());
    return this->impl();
  }
};

/**
 * @brief Helper class to set longitudinal velocity interpolators for the trajectory.
 * @tparam TrajectoryType Type of the trajectory.
 */
template <typename TrajectoryType>
class SetLongitudinalVelocityInterpolatorImpl
: public autoware::common::helper_functions::crtp<
    TrajectoryType, SetLongitudinalVelocityInterpolatorImpl<TrajectoryType>>
{
public:
  /**
   * @brief Sets the longitudinal velocity interpolator.
   * @param interpolator Interpolator object.
   * @return Reference to the trajectory with longitudinal velocity interpolator set.
   */
  TrajectoryType & set_longitudinal_velocity_mps_interpolator(
    const interpolator::Interpolator<double> & interpolator)
  {
    this->impl().longitudinal_velocity_mps.set_interpolator(interpolator);
    return this->impl();
  }
};

/**
 * @brief Helper class to set lateral velocity interpolators for the trajectory.
 * @tparam TrajectoryType Type of the trajectory.
 */
template <typename TrajectoryType>
class SetLateralVelocityInterpolatorImpl
: public autoware::common::helper_functions::crtp<
    TrajectoryType, SetLateralVelocityInterpolatorImpl<TrajectoryType>>
{
public:
  /**
   * @brief Sets the lateral velocity interpolator.
   * @param interpolator Interpolator object.
   * @return Reference to the trajectory with lateral velocity interpolator set.
   */
  TrajectoryType & set_lateral_velocity_mps_interpolator(
    const interpolator::Interpolator<double> & interpolator)
  {
    this->impl().lateral_velocity_mps.set_interpolator(interpolator);
    return this->impl();
  }
};

/**
 * @brief Helper class to set heading rate interpolators for the trajectory.
 * @tparam TrajectoryType Type of the trajectory.
 */
template <typename TrajectoryType>
class SetHeadingRateInterpolatorImpl
: public autoware::common::helper_functions::crtp<
    TrajectoryType, SetHeadingRateInterpolatorImpl<TrajectoryType>>
{
public:
  /**
   * @brief Sets the heading rate interpolator.
   * @param interpolator Interpolator object.
   * @return Reference to the trajectory with heading rate interpolator set.
   */
  TrajectoryType & set_heading_rate_rps_interpolator(
    const interpolator::Interpolator<double> & interpolator)
  {
    this->impl().heading_rate_rps.set_interpolator(interpolator);
    return this->impl();
  }
};

/**
 * @brief Helper class to set lane IDs interpolators for the trajectory.
 * @tparam TrajectoryType Type of the trajectory.
 */
template <typename TrajectoryType>
class SetLaneIdsInterpolatorImpl : public autoware::common::helper_functions::crtp<
                                     TrajectoryType, SetLaneIdsInterpolatorImpl<TrajectoryType>>
{
public:
  /**
   * @brief Sets the lane IDs interpolator.
   * @param interpolator Interpolator object.
   * @return Reference to the trajectory with lane IDs interpolator set.
   */
  TrajectoryType & set_lane_ids_interpolator(
    const interpolator::Interpolator<std::vector<int64_t>> & interpolator)
  {
    this->impl().lane_ids.set_interpolator(interpolator);
    return this->impl();
  }
};

}  // namespace detail

}  // namespace autoware::motion_utils::trajectory_container::trajectory

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_HPP_
